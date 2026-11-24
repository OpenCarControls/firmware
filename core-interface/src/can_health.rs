//! CAN bus health, reported to the app over BLE (`SystemState.can_bus_health`).
//!
//! Each board CAN driver counts what it receives over a short window ([`CanRxCounts`])
//! and hands the window to [`report_can_bus_window`] when it ends. This module keeps
//! running totals per bus, judges each window ([`can_bus_status`]) and publishes the
//! result with [`publish_can_bus_health_task`]. Boards never build protos; vehicles
//! never see any of it.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use crate::channels::MAX_CAN_BUSES;
use crate::proto;

/// RX counters over one window of a bus driver. Saturating, never wrapping.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CanRxCounts {
    /// Frames received.
    pub received: u32,
    /// Frames the vehicle's filters accepted.
    pub matched: u32,
    /// Receive errors (bit, stuff, form, CRC, acknowledge...).
    pub errors: u32,
    /// Receive queue overflows (frames dropped).
    pub overruns: u32,
    /// Times the controller went bus-off.
    pub bus_offs: u32,
}

impl CanRxCounts {
    pub const fn new() -> Self {
        Self {
            received: 0,
            matched: 0,
            errors: 0,
            overruns: 0,
            bus_offs: 0,
        }
    }

    pub fn record_frame(&mut self, matched: bool) {
        self.received = self.received.saturating_add(1);
        if matched {
            self.matched = self.matched.saturating_add(1);
        }
    }

    pub fn record_error(&mut self) {
        self.errors = self.errors.saturating_add(1);
    }

    pub fn record_overrun(&mut self) {
        self.overruns = self.overruns.saturating_add(1);
    }

    pub fn record_bus_off(&mut self) {
        self.bus_offs = self.bus_offs.saturating_add(1);
    }

    /// Returns the counters and starts a new window.
    pub fn take(&mut self) -> Self {
        core::mem::take(self)
    }

    fn add(&mut self, other: &Self) {
        self.received = self.received.saturating_add(other.received);
        self.matched = self.matched.saturating_add(other.matched);
        self.errors = self.errors.saturating_add(other.errors);
        self.overruns = self.overruns.saturating_add(other.overruns);
        self.bus_offs = self.bus_offs.saturating_add(other.bus_offs);
    }
}

/// Judges one window of a bus. A bus-off outranks everything; errors with no frame at
/// all usually mean a wrong bitrate, a CAN FD bus or a wiring fault.
pub fn can_bus_status(window: &CanRxCounts) -> proto::CanBusStatus {
    use proto::CanBusStatus as S;
    if window.bus_offs > 0 {
        S::BusOff
    } else if window.received == 0 && window.errors > 0 {
        S::NoValidFrames
    } else if window.errors > 0 || window.overruns > 0 {
        S::Degraded
    } else if window.received == 0 {
        S::Silent
    } else {
        S::Ok
    }
}

/// What is known about one bus: totals since boot and the last window's verdict.
#[derive(Debug, Clone, Copy, PartialEq)]
struct BusHealth {
    totals: CanRxCounts,
    status: proto::CanBusStatus,
    listen_only: Option<bool>,
}

impl BusHealth {
    fn to_proto(self) -> proto::CanBusHealth {
        proto::CanBusHealth {
            status: Some(self.status as i32),
            listen_only: self.listen_only,
            frames_received: Some(self.totals.received),
            frames_matched: Some(self.totals.matched),
            rx_errors: Some(self.totals.errors),
            overruns: Some(self.totals.overruns),
            bus_offs: Some(self.totals.bus_offs),
        }
    }
}

type Buses = [Option<BusHealth>; MAX_CAN_BUSES];

static BUSES: Mutex<CriticalSectionRawMutex, RefCell<Buses>> =
    Mutex::new(RefCell::new([None; MAX_CAN_BUSES]));

/// Wakes the publisher when a bus reports, or when the app subscribes.
static CAN_HEALTH_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Records the end of one RX window of `bus_id`. Called by board CAN drivers, every few
/// seconds and right after a bus-off. `listen_only` is whether the controller ran in
/// hardware listen-only mode, or `None` for a driver that has no such mode. Never
/// blocks. Ignores a bus id beyond [`MAX_CAN_BUSES`].
pub fn report_can_bus_window(bus_id: u8, window: &CanRxCounts, listen_only: Option<bool>) {
    let reported = BUSES.lock(|buses| {
        let mut buses = buses.borrow_mut();
        let Some(slot) = buses.get_mut(bus_id as usize) else {
            return false;
        };
        let mut totals = slot.map(|bus| bus.totals).unwrap_or_default();
        totals.add(window);
        *slot = Some(BusHealth {
            totals,
            status: can_bus_status(window),
            listen_only,
        });
        true
    });
    if reported {
        CAN_HEALTH_CHANGED.signal(());
    }
}

/// The health of every bus that has reported, by bus id.
pub fn can_bus_health() -> alloc::collections::BTreeMap<u32, proto::CanBusHealth> {
    BUSES.lock(|buses| {
        buses
            .borrow()
            .iter()
            .enumerate()
            .filter_map(|(id, bus)| Some((id as u32, bus.as_ref()?.to_proto())))
            .collect()
    })
}

/// A state update carrying only [`can_bus_health`].
pub fn can_bus_health_state_update(timestamp_ms: u64) -> proto::DeviceToApp {
    proto::DeviceToApp {
        timestamp_ms,
        platform_id: crate::platform_id(),
        payload: Some(proto::device_to_app::Payload::StateUpdate(
            proto::StateUpdate {
                system_state: Some(proto::SystemState {
                    can_bus_health: can_bus_health(),
                    ..Default::default()
                }),
                vehicle_state: None,
            },
        )),
    }
}

/// How often the bus health is sent at most while no bus changes status, and at least
/// while nothing changes at all (a message sent as the app subscribes can be lost).
/// Matches the boards' RX report window, so the counters reach the app about as often
/// as they are reported.
pub const CAN_BUS_HEALTH_REPEAT_MS: u64 = 5_000;

/// The statuses last queued for the app (per bus: status and listen-only), and when.
type Summary = [Option<(proto::CanBusStatus, Option<bool>)>; MAX_CAN_BUSES];

static LAST_PUBLISHED: Mutex<CriticalSectionRawMutex, RefCell<Option<(Summary, u64)>>> =
    Mutex::new(RefCell::new(None));

fn summary() -> Summary {
    BUSES.lock(|buses| {
        buses
            .borrow()
            .map(|bus| bus.map(|b| (b.status, b.listen_only)))
    })
}

/// Queues [`can_bus_health_state_update`] on BLE when a bus changed status (or appeared)
/// since the last one queued, or when that was [`CAN_BUS_HEALTH_REPEAT_MS`] or more ago.
/// Counters alone don't make it due sooner, so a busy bus doesn't flood the link.
/// Nothing is sent before a bus has reported. `try_send`: never blocks; a full queue or a
/// link that is down leaves it unsent, to retry on the next call. Returns whether a
/// message was queued.
pub fn publish_can_bus_health(timestamp_ms: u64) -> bool {
    let summary = summary();
    if summary.iter().all(Option::is_none) {
        return false;
    }
    let due = LAST_PUBLISHED.lock(|last| match last.borrow().as_ref() {
        Some((sent, sent_at)) => {
            *sent != summary || timestamp_ms.saturating_sub(*sent_at) >= CAN_BUS_HEALTH_REPEAT_MS
        }
        None => true,
    });
    if !due {
        return false;
    }
    let msg = can_bus_health_state_update(timestamp_ms);
    if !crate::routing::try_send_to(crate::types::Transport::Ble, msg) {
        return false;
    }
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = Some((summary, timestamp_ms)));
    true
}

/// Sends the bus health again even if unchanged, e.g. when the app subscribes.
pub fn resend_can_bus_health() {
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = None);
    CAN_HEALTH_CHANGED.signal(());
}

/// Forgets every bus and what was published, for tests.
pub fn reset_can_bus_health_for_tests() {
    BUSES.lock(|buses| *buses.borrow_mut() = [None; MAX_CAN_BUSES]);
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = None);
}

/// Publishes the bus health over BLE when a bus reports, and checks once a second, which
/// sends the periodic repeat and retries a send that found the queue full.
#[embassy_executor::task]
pub async fn publish_can_bus_health_task() {
    loop {
        publish_can_bus_health(Instant::now().as_millis());
        let _ = embassy_futures::select::select(
            CAN_HEALTH_CHANGED.wait(),
            Timer::after(Duration::from_secs(1)),
        )
        .await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proto::CanBusStatus as S;

    fn counts(received: u32, errors: u32, overruns: u32, bus_offs: u32) -> CanRxCounts {
        CanRxCounts {
            received,
            matched: received,
            errors,
            overruns,
            bus_offs,
        }
    }

    #[test]
    fn status_of_a_window() {
        assert_eq!(can_bus_status(&counts(0, 0, 0, 0)), S::Silent);
        assert_eq!(can_bus_status(&counts(10, 0, 0, 0)), S::Ok);
        assert_eq!(can_bus_status(&counts(10, 1, 0, 0)), S::Degraded);
        assert_eq!(can_bus_status(&counts(10, 0, 1, 0)), S::Degraded);
        assert_eq!(can_bus_status(&counts(0, 5, 0, 0)), S::NoValidFrames);
        assert_eq!(can_bus_status(&counts(10, 5, 1, 1)), S::BusOff);
        assert_eq!(can_bus_status(&counts(0, 0, 0, 1)), S::BusOff);
    }

    #[test]
    fn counts_record_and_take() {
        let mut c = CanRxCounts::new();
        c.record_frame(true);
        c.record_frame(false);
        c.record_error();
        c.record_overrun();
        c.record_bus_off();
        assert_eq!(c.take(), counts(2, 1, 1, 1).with_matched(1));
        assert_eq!(c, CanRxCounts::new());
    }

    #[test]
    fn counts_saturate_instead_of_wrapping() {
        let mut c = CanRxCounts {
            received: u32::MAX,
            ..CanRxCounts::new()
        };
        c.record_frame(false);
        assert_eq!(c.received, u32::MAX);
        let mut total = c;
        total.add(&c);
        assert_eq!(total.received, u32::MAX);
    }

    impl CanRxCounts {
        fn with_matched(mut self, matched: u32) -> Self {
            self.matched = matched;
            self
        }
    }
}
