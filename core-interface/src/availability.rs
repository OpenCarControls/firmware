//! Which of the vehicle's commands the device accepts right now, for the app to grey out.
//!
//! The vehicle crate evaluates each command's requirements (`requirements.rs`) against its
//! state and reports the result with [`report_command_availability`], about once a second.
//! This module keeps the latest report and publishes it to the app over BLE like the CAN
//! access state: when it changes, repeated while unchanged (a message sent as the app
//! subscribes can be lost), and again on subscribe.
//!
//! The app must never see "available" for a command the device would reject, so what is
//! published is never more optimistic than what the device would answer:
//! - nothing is published until the vehicle has reported;
//! - while the device is read-only, every command is unavailable, whatever was reported;
//! - a report older than [`REPORT_MAX_AGE_MS`] (the vehicle stopped reporting) counts as
//!   unavailable too.
//!
//! MQTT apps are told too, for the basic commands only (advanced commands are BLE only), at
//! the same rate as the MQTT state: every [`COMMAND_AVAILABILITY_REPEAT_MS`] while a client is
//! active, once per MQTT idle period otherwise. MQTT commands are not authenticated yet, so the
//! device rejects every one; until that changes they are all reported unavailable over MQTT.
//!
//! Commands are identified by their field number in the vehicle's command oneof, so this
//! module stays vehicle-agnostic.

use alloc::collections::BTreeMap;
use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use crate::channels::{BLE_TX_CHANNEL, MQTT_TX_CHANNEL};
use crate::proto;
use crate::proto::CommandAvailabilityState as State;

/// How often the availability is sent again while unchanged, like
/// [`CAN_ACCESS_REPEAT_MS`](crate::CAN_ACCESS_REPEAT_MS).
pub const COMMAND_AVAILABILITY_REPEAT_MS: u64 = 5_000;

/// A report older than this is not trusted: the vehicle reports about once a second, so a
/// report this old means it stopped, and the commands count as unavailable.
pub const REPORT_MAX_AGE_MS: u64 = 5_000;

/// The two command sets of a vehicle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandKind {
    Basic,
    Advanced,
}

#[derive(Default)]
struct Report {
    basic: BTreeMap<u32, State>,
    advanced: BTreeMap<u32, State>,
    /// When the vehicle last reported, `None` until it has.
    at_ms: Option<u64>,
}

static REPORT: Mutex<CriticalSectionRawMutex, RefCell<Report>> = Mutex::new(RefCell::new(Report {
    basic: BTreeMap::new(),
    advanced: BTreeMap::new(),
    at_ms: None,
}));

static CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The last state queued for the app and when, so unchanged states aren't sent more often
/// than every [`COMMAND_AVAILABILITY_REPEAT_MS`].
static LAST_PUBLISHED: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<(proto::CommandAvailability, u64)>>,
> = Mutex::new(RefCell::new(None));

/// The last state queued for MQTT and when, see [`LAST_PUBLISHED`].
static LAST_MQTT_PUBLISHED: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<(proto::CommandAvailability, u64)>>,
> = Mutex::new(RefCell::new(None));

/// Reports the availability of every command of `kind`, replacing the previous report for
/// that kind. `now_ms` is the report time (`Instant::now().as_millis()`).
pub fn report_command_availability_at(kind: CommandKind, entries: &[(u32, State)], now_ms: u64) {
    let changed = REPORT.lock(|report| {
        let mut report = report.borrow_mut();
        let map: BTreeMap<u32, State> = entries.iter().copied().collect();
        let target = match kind {
            CommandKind::Basic => &mut report.basic,
            CommandKind::Advanced => &mut report.advanced,
        };
        let changed = *target != map;
        *target = map;
        report.at_ms = Some(now_ms);
        changed
    });
    if changed {
        CHANGED.signal(());
    }
}

/// [`report_command_availability_at`] now.
pub fn report_command_availability(kind: CommandKind, entries: &[(u32, State)]) {
    report_command_availability_at(kind, entries, Instant::now().as_millis());
}

/// What the app is told at `now_ms`, or `None` while the vehicle has not reported.
pub fn command_availability_at(now_ms: u64) -> Option<proto::CommandAvailability> {
    let (mut basic, mut advanced, at_ms) = REPORT.lock(|report| {
        let report = report.borrow();
        (report.basic.clone(), report.advanced.clone(), report.at_ms)
    });
    let at_ms = at_ms?;
    let stale = now_ms.saturating_sub(at_ms) > REPORT_MAX_AGE_MS;
    if stale || crate::can::is_can_read_only() {
        for state in basic.values_mut().chain(advanced.values_mut()) {
            // A command a car doesn't have stays unsupported; everything else is held back.
            if *state != State::Unsupported {
                *state = State::Unavailable;
            }
        }
    }
    Some(proto::CommandAvailability {
        basic: basic.into_iter().map(|(k, v)| (k, v as i32)).collect(),
        advanced: advanced.into_iter().map(|(k, v)| (k, v as i32)).collect(),
    })
}

/// [`command_availability_at`] now.
pub fn command_availability() -> Option<proto::CommandAvailability> {
    command_availability_at(Instant::now().as_millis())
}

/// A BLE state update carrying only the system state with the availability.
pub fn command_availability_state_update(
    timestamp_ms: u64,
    availability: proto::CommandAvailability,
) -> proto::DeviceToApp {
    proto::DeviceToApp {
        timestamp_ms,
        platform_id: crate::platform_id(),
        payload: Some(proto::device_to_app::Payload::StateUpdate(
            proto::StateUpdate {
                system_state: Some(proto::SystemState {
                    command_availability: Some(availability),
                    ..Default::default()
                }),
                vehicle_state: None,
            },
        )),
    }
}

/// Queues the availability on BLE if it changed since the last one queued, or if that was
/// [`COMMAND_AVAILABILITY_REPEAT_MS`] or more ago. `try_send`: never blocks; a full queue
/// leaves it unsent, to retry on the next call. Returns whether a message was queued.
pub fn publish_command_availability(timestamp_ms: u64) -> bool {
    let Some(availability) = command_availability_at(timestamp_ms) else {
        return false;
    };
    let due = LAST_PUBLISHED.lock(|last| match last.borrow().as_ref() {
        Some((sent, sent_at)) => {
            *sent != availability
                || timestamp_ms.saturating_sub(*sent_at) >= COMMAND_AVAILABILITY_REPEAT_MS
        }
        None => true,
    });
    if !due {
        return false;
    }
    let msg = command_availability_state_update(timestamp_ms, availability.clone());
    if BLE_TX_CHANNEL.try_send(msg).is_err() {
        return false;
    }
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = Some((availability, timestamp_ms)));
    true
}

/// What an MQTT app is told, given what a BLE app is told: the basic commands only, and none
/// of them available while MQTT commands are rejected as unauthenticated.
fn mqtt_view(ble: &proto::CommandAvailability) -> proto::CommandAvailability {
    let mut basic = ble.basic.clone();
    if !crate::dispatch::MQTT_COMMANDS_AUTHENTICATED {
        for state in basic.values_mut() {
            if *state != State::Unsupported as i32 {
                *state = State::Unavailable as i32;
            }
        }
    }
    proto::CommandAvailability {
        basic,
        advanced: BTreeMap::new(),
    }
}

/// What an MQTT app is told at `now_ms`, or `None` while the vehicle has not reported.
pub fn mqtt_command_availability_at(now_ms: u64) -> Option<proto::CommandAvailability> {
    command_availability_at(now_ms).map(|ble| mqtt_view(&ble))
}

/// Queues the availability on MQTT like [`publish_command_availability`] does on BLE, at the
/// MQTT state's rate: while a client is active when it changed or [`COMMAND_AVAILABILITY_REPEAT_MS`]
/// passed, otherwise once per MQTT idle period. `try_send`: never blocks the MQTT driver's
/// queue; a full queue leaves it unsent, to retry on the next call.
pub fn publish_command_availability_mqtt(timestamp_ms: u64) -> bool {
    let Some(availability) = mqtt_command_availability_at(timestamp_ms) else {
        return false;
    };
    let active = crate::routing::is_mqtt_active((timestamp_ms / 1000) as u32);
    let due = LAST_MQTT_PUBLISHED.lock(|last| match last.borrow().as_ref() {
        None => true,
        Some((sent, sent_at)) => {
            let elapsed = timestamp_ms.saturating_sub(*sent_at);
            if active {
                *sent != availability || elapsed >= COMMAND_AVAILABILITY_REPEAT_MS
            } else {
                elapsed >= u64::from(crate::routing::MQTT_IDLE_RATE_S) * 1_000
            }
        }
    });
    if !due {
        return false;
    }
    let msg = command_availability_state_update(timestamp_ms, availability.clone());
    if MQTT_TX_CHANNEL.try_send(msg).is_err() {
        return false;
    }
    LAST_MQTT_PUBLISHED.lock(|last| *last.borrow_mut() = Some((availability, timestamp_ms)));
    true
}

/// Sends the availability again even if unchanged, e.g. when the app subscribes.
pub fn resend_command_availability() {
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = None);
    LAST_MQTT_PUBLISHED.lock(|last| *last.borrow_mut() = None);
    CHANGED.signal(());
}

/// Forgets the report and what was published. For use in tests only.
pub fn reset_command_availability_for_tests() {
    REPORT.lock(|report| *report.borrow_mut() = Report::default());
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = None);
    LAST_MQTT_PUBLISHED.lock(|last| *last.borrow_mut() = None);
    CHANGED.reset();
}

/// Publishes the availability over BLE and MQTT whenever it changes, and every
/// [`COMMAND_AVAILABILITY_REPEAT_MS`] while it doesn't. Also checks once a second, which
/// retries a send that found the queue full and picks up a read-only flip or an expired
/// report.
#[embassy_executor::task]
pub async fn publish_command_availability_task() {
    loop {
        let now_ms = Instant::now().as_millis();
        publish_command_availability(now_ms);
        publish_command_availability_mqtt(now_ms);
        let _ =
            embassy_futures::select::select(CHANGED.wait(), Timer::after(Duration::from_secs(1)))
                .await;
    }
}
