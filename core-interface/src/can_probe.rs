//! The identity probe: the one transmit allowed before the car is identified (see
//! `docs/can.md`, "Identity probe").
//!
//! Some cars only reveal what identifies them (a serial number, say) when asked with a
//! diagnostic request. A vehicle declares that request as an [`IdentityProbe`]: one
//! bus, and the exact frames it may send, each with its ID. While a [`ProbeWindow`] is
//! open, the board takes the bus out of listen-only mode and sends each of those frames
//! at most once, byte for byte; it drops anything else. Opening one is only allowed
//! while the platform is confirmed, the exact car isn't, no logging session runs, and
//! the bus is awake.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};
use portable_atomic::{AtomicU8, AtomicU32, Ordering};

use embedded_can::{ExtendedId, Id, StandardId};

use crate::can_tx::CanTxFrame;
use crate::channels::MAX_CAN_BUSES;
use crate::types::CanFrame;

/// How many windows can be opened per boot, whatever the outcome.
pub const MAX_PROBE_ATTEMPTS: u8 = 5;
/// How long a window stays open at most. Covers a response-pending answer (5 s, see
/// `isotp.rs`) before the flow control is sent.
pub const PROBE_WINDOW_MS: u64 = 6_000;
/// How long to wait after opening a window before sending, for the controller to leave
/// listen-only mode. A frame sent earlier is dropped.
pub const PROBE_SETTLE_MS: u64 = 50;
/// A bus counts as awake when it received a frame this recently. A frame sent on a
/// sleeping bus wakes the car up (seen on an EV6).
pub const BUS_AWAKE_MS: u64 = 1_000;
/// The most frames a probe can declare.
pub const MAX_PROBE_FRAMES: usize = 4;

/// A diagnostic request a vehicle declares at build time.
pub struct IdentityProbe {
    pub bus_id: u8,
    /// Every frame the probe may send, exactly, with the ID it is sent on: the request and
    /// the flow control (on the same ID for a request to one ECU; to the answering ECU's
    /// own ID after a broadcast request like OBD's 0x7DF).
    pub frames: &'static [(Id, [u8; 8])],
}

/// An 11-bit ID, for declaring a probe in a `const`. Fails to build for a value that
/// doesn't fit in 11 bits.
pub const fn standard_id(raw: u16) -> Id {
    match StandardId::new(raw) {
        Some(id) => Id::Standard(id),
        None => panic!("not an 11-bit CAN ID"),
    }
}

/// A 29-bit ID, for declaring a probe in a `const`. Fails to build for a value that
/// doesn't fit in 29 bits.
pub const fn extended_id(raw: u32) -> Id {
    match ExtendedId::new(raw) {
        Some(id) => Id::Extended(id),
        None => panic!("not a 29-bit CAN ID"),
    }
}

/// Why a window wasn't opened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProbeRefused {
    /// The platform isn't confirmed, or the exact car already is.
    NotNeeded,
    /// A CAN logging session has the buses.
    Logging,
    /// No frame received on the bus in the last [`BUS_AWAKE_MS`].
    BusAsleep,
    /// Another window is open.
    Busy,
    /// [`MAX_PROBE_ATTEMPTS`] already used since boot.
    NoAttemptsLeft,
    /// No such bus, a CAN-FD bus, no frames or too many.
    Invalid,
}

struct Window {
    probe: &'static IdentityProbe,
    opened_at: Instant,
    relocks: u32,
    /// Bit `i` set once `probe.frames[i]` has been sent.
    sent: u8,
}

static WINDOW: Mutex<CriticalSectionRawMutex, RefCell<Option<Window>>> =
    Mutex::new(RefCell::new(None));
static ATTEMPTS: AtomicU8 = AtomicU8::new(0);
/// Counts `set_can_read_only(true)` calls (bus-off, a logging session, the vehicle): a
/// window opened before one is closed.
static RELOCKS: AtomicU32 = AtomicU32::new(0);
/// When each bus last received a frame, in ms since boot plus 1, wrapping (0: never).
static LAST_RX_MS: [AtomicU32; MAX_CAN_BUSES] = [const { AtomicU32::new(0) }; MAX_CAN_BUSES];

/// Records that `bus_id` received a frame (called for every frame by `log_can_frame`).
pub(crate) fn note_can_rx(bus_id: u8) {
    if let Some(last) = LAST_RX_MS.get(bus_id as usize) {
        last.store(ms_plus_one(Instant::now()), Ordering::Relaxed);
    }
}

fn ms_plus_one(t: Instant) -> u32 {
    (t.as_millis() as u32).wrapping_add(1)
}

pub(crate) fn note_relock() {
    RELOCKS.fetch_add(1, Ordering::Relaxed);
}

fn is_bus_awake(bus_id: u8, now: Instant) -> bool {
    let last = LAST_RX_MS[bus_id as usize].load(Ordering::Relaxed);
    last != 0 && ms_plus_one(now).wrapping_sub(last) as u64 <= BUS_AWAKE_MS
}

/// Whether the probe is still needed and allowed, whatever the window.
fn probe_allowed() -> Result<(), ProbeRefused> {
    if !crate::can::is_car_identity_confirmed() || crate::is_car_fingerprint_confirmed() {
        return Err(ProbeRefused::NotNeeded);
    }
    if crate::is_can_logging() {
        return Err(ProbeRefused::Logging);
    }
    Ok(())
}

/// Opens a window for `probe`. Each call uses one of the [`MAX_PROBE_ATTEMPTS`].
/// Wait [`PROBE_SETTLE_MS`] before sending; the window closes when it is dropped, after
/// [`PROBE_WINDOW_MS`], or when the buses are relocked.
pub fn open_identity_probe(probe: &'static IdentityProbe) -> Result<ProbeWindow, ProbeRefused> {
    let bus = probe.bus_id as usize;
    if bus >= MAX_CAN_BUSES
        || crate::is_can_fd_bus(probe.bus_id)
        || probe.frames.is_empty()
        || probe.frames.len() > MAX_PROBE_FRAMES
    {
        return Err(ProbeRefused::Invalid);
    }
    probe_allowed()?;
    let now = Instant::now();
    if !is_bus_awake(probe.bus_id, now) {
        return Err(ProbeRefused::BusAsleep);
    }
    WINDOW.lock(|w| {
        let mut w = w.borrow_mut();
        if w.is_some() {
            return Err(ProbeRefused::Busy);
        }
        if ATTEMPTS.load(Ordering::Relaxed) >= MAX_PROBE_ATTEMPTS {
            return Err(ProbeRefused::NoAttemptsLeft);
        }
        ATTEMPTS.fetch_add(1, Ordering::Relaxed);
        *w = Some(Window {
            probe,
            opened_at: now,
            relocks: RELOCKS.load(Ordering::Relaxed),
            sent: 0,
        });
        Ok(())
    })?;
    crate::can::notify_can_read_only_changed();
    Ok(ProbeWindow { _private: () })
}

/// An open probe window; dropping it closes the window.
pub struct ProbeWindow {
    _private: (),
}

impl Drop for ProbeWindow {
    fn drop(&mut self) {
        WINDOW.lock(|w| *w.borrow_mut() = None);
        crate::can::notify_can_read_only_changed();
    }
}

/// Runs `f` on the window if one is open on `bus_id` and still valid.
fn with_open_window<R>(bus_id: u8, now: Instant, f: impl FnOnce(&mut Window) -> R) -> Option<R> {
    probe_allowed().ok()?;
    WINDOW.lock(|w| {
        let mut w = w.borrow_mut();
        let window = w.as_mut().filter(|w| {
            w.probe.bus_id == bus_id
                && now.saturating_duration_since(w.opened_at)
                    < Duration::from_millis(PROBE_WINDOW_MS)
                && w.relocks == RELOCKS.load(Ordering::Relaxed)
        })?;
        Some(f(window))
    })
}

/// Returns `true` if the controller of `bus_id` must run in listen-only mode: the bus is
/// read-only ([`crate::is_can_bus_read_only`]) and no probe window is open on it. Board
/// drivers use this for the controller mode.
pub fn is_can_bus_listen_only(bus_id: u8) -> bool {
    is_can_bus_listen_only_at(bus_id, Instant::now())
}

fn is_can_bus_listen_only_at(bus_id: u8, now: Instant) -> bool {
    crate::is_can_bus_read_only(bus_id) && with_open_window(bus_id, now, |_| ()).is_none()
}

/// Returns `true` if `frame` may be sent on `bus_id`, from its class (see `can_tx.rs`).
/// A platform frame: the bus is read-write, or the open probe window declared this exact
/// frame (ID and data) and hasn't sent it yet (it then counts as sent). A command frame:
/// the bus is read-write and the phone whose command it is is still validated for this
/// car; never through a probe window. Board drivers call it for every outbound frame,
/// right before sending it, and drop the frame when it returns `false`.
pub fn allow_can_tx(bus_id: u8, frame: &CanTxFrame) -> bool {
    match frame.command_device_id() {
        Some(device_id) => {
            frame.bus_id() == bus_id
                && !crate::is_can_bus_read_only(bus_id)
                && crate::phone_trust::is_phone_validated(device_id)
        }
        None => allow_platform_tx_at(bus_id, frame.frame(), Instant::now()),
    }
}

fn allow_platform_tx_at(bus_id: u8, frame: &CanFrame, now: Instant) -> bool {
    if !crate::is_can_bus_read_only(bus_id) {
        return true;
    }
    if frame.bus_id != bus_id || frame.dlc != 8 {
        return false;
    }
    with_open_window(bus_id, now, |w| {
        let unsent = w.probe.frames.iter().enumerate().find(|(i, (id, data))| {
            *id == frame.id && *data == frame.data && w.sent & (1 << i) == 0
        });
        match unsent {
            Some((i, _)) => {
                w.sent |= 1 << i;
                true
            }
            None => false,
        }
    })
    .unwrap_or(false)
}

/// Forgets the attempts, the window and the bus activity. Tests only.
pub fn reset_identity_probe_for_tests() {
    WINDOW.lock(|w| *w.borrow_mut() = None);
    ATTEMPTS.store(0, Ordering::Relaxed);
    for last in &LAST_RX_MS {
        last.store(0, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static PROBE: IdentityProbe = IdentityProbe {
        bus_id: 0,
        frames: &[
            (
                standard_id(0x780),
                crate::isotp::uds_single_frame(&[0x22, 0xF1, 0x8C], 0x00),
            ),
            (standard_id(0x780), crate::isotp::iso_tp_flow_control(0x00)),
        ],
    };

    fn ready() {
        reset_identity_probe_for_tests();
        crate::reset_can_access_for_tests();
        crate::revoke_car_identity();
        crate::confirm_car_identity();
        note_can_rx(0);
    }

    #[test]
    fn a_window_expires() {
        ready();
        let window = open_identity_probe(&PROBE).unwrap();
        let (id, data) = PROBE.frames[0];
        let request = CanFrame {
            bus_id: 0,
            id,
            data,
            dlc: 8,
        };
        let late = Instant::now() + Duration::from_millis(PROBE_WINDOW_MS);
        assert!(is_can_bus_listen_only_at(0, late));
        assert!(!allow_platform_tx_at(0, &request, late));
        assert!(
            allow_platform_tx_at(0, &request, Instant::now()),
            "still open now"
        );
        drop(window);
        crate::revoke_car_identity();
    }

    #[test]
    fn a_bus_is_asleep_without_recent_frames() {
        ready();
        let now = Instant::now();
        assert!(is_bus_awake(0, now));
        assert!(!is_bus_awake(
            0,
            now + Duration::from_millis(BUS_AWAKE_MS + 1)
        ));
        assert!(!is_bus_awake(1, now), "bus 1 never received anything");
        crate::revoke_car_identity();
    }
}
