use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use portable_atomic::{AtomicBool, Ordering};

use crate::channels::MAX_CAN_BUSES;
use crate::types::{CanFilter, CanFrame};

// ── CAN Read-Only Mode ────────────────────────────────────────────────────────

/// When `true`, the CAN buses are read-only: board drivers whose controller supports
/// it (ESP TWAI and MCP2515) run in hardware listen-only mode, so the device never
/// drives the bus at all (no TX, no ACKs, no error frames), and every board driver
/// silently drops outbound TX frames. Defaults to `true` at boot so nothing can reach
/// the bus until the vehicle crate explicitly unlocks it after validating the
/// connected car.
pub(crate) static CAN_READ_ONLY: AtomicBool = AtomicBool::new(true);

/// Wakes board CAN drivers when the flag changes, so they can switch controller mode
/// immediately instead of polling. The atomic stays the source of truth for the TX check.
static CAN_READ_ONLY_WATCH: Watch<CriticalSectionRawMutex, bool, MAX_CAN_BUSES> =
    Watch::new_with(true);

/// Receiver type returned by [`can_read_only_receiver`].
pub type CanReadOnlyReceiver = Receiver<'static, CriticalSectionRawMutex, bool, MAX_CAN_BUSES>;

/// Returns a receiver whose `changed()` resolves whenever `set_can_read_only` is called.
/// One receiver per CAN bus driver; `None` if more than `MAX_CAN_BUSES` are requested.
/// A new receiver reports the current value as changed once, so always re-check
/// `is_can_read_only()` after a wakeup rather than trusting the payload.
pub fn can_read_only_receiver() -> Option<CanReadOnlyReceiver> {
    CAN_READ_ONLY_WATCH.receiver()
}

/// Returns `true` if the CAN buses are currently in read-only mode.
///
/// Vehicle tasks should check this before deciding whether to attempt a
/// CAN write; if `true`, pushes to the bus's `CAN_TX_CHANNELS` entry will be accepted by
/// the channel but silently dropped by the board driver loop.
///
/// Also `true` for as long as a CAN logging session is active (see
/// [`crate::is_can_logging`]), whatever the vehicle asks for.
pub fn is_can_read_only() -> bool {
    CAN_READ_ONLY.load(Ordering::Relaxed) || crate::can_log::is_can_logging()
}

/// Enables or disables CAN read-only mode.
///
/// Board drivers poll this flag and switch their controller between listen-only and
/// normal mode within a few milliseconds. On ESP TWAI the switch resets the
/// controller, so frames arriving during the switch are lost.
///
/// Call `set_can_read_only(false)` from a vehicle task once inbound CAN frames
/// have been validated to confirm the connected car matches this firmware, after
/// `confirm_car_identity()`.
/// Call `set_can_read_only(true)` to re-engage the lock if an error or
/// inconsistent data is detected at any point.
///
/// Leaving read-only mode (`enabled == false`) is refused, and the bus stays locked,
/// until the vehicle has confirmed the car's identity with [`confirm_car_identity`].
/// An unlock request while a CAN logging session is active is ignored too, not deferred:
/// the vehicle has to validate the car again once the session is over.
/// Returns the resulting read-only state.
pub fn set_can_read_only(enabled: bool) -> bool {
    let enabled = enabled || crate::can_log::is_can_logging() || !is_car_identity_confirmed();
    CAN_READ_ONLY.store(enabled, Ordering::Relaxed);
    notify_can_read_only_changed();
    enabled
}

/// Wakes the board CAN drivers so they re-read [`is_can_read_only`].
pub(crate) fn notify_can_read_only_changed() {
    CAN_READ_ONLY_WATCH.sender().send(is_can_read_only());
}

// ── Car identity ──────────────────────────────────────────────────────────────

/// Set by the vehicle once it has confirmed, from what the car itself reports (VIN),
/// that it is plugged into a car it supports. Defaults to `false` at boot, so no
/// vehicle can transmit before checking.
static CAR_IDENTITY_CONFIRMED: AtomicBool = AtomicBool::new(false);

/// Returns `true` once the vehicle has confirmed the connected car.
pub fn is_car_identity_confirmed() -> bool {
    CAR_IDENTITY_CONFIRMED.load(Ordering::Relaxed)
}

/// Records that the connected car is one the vehicle supports. This only makes
/// `set_can_read_only(false)` possible; it does not unlock the bus by itself.
pub fn confirm_car_identity() {
    CAR_IDENTITY_CONFIRMED.store(true, Ordering::Relaxed);
}

/// Forgets the confirmation and re-locks the bus. Call it when the car's identity
/// changes or can't be trusted any more (a different VIN, inconsistent data).
pub fn revoke_car_identity() {
    CAR_IDENTITY_CONFIRMED.store(false, Ordering::Relaxed);
    set_can_read_only(true);
}

/// Checks a VIN a car reported against the VINs a vehicle supports.
/// `prefixes` are the leading characters to accept: 3 for a manufacturer (the WMI,
/// e.g. `"KNA"`), longer to pin a model or a single car. Returns `false` for anything
/// that isn't a well-formed VIN (17 characters, digits and capitals except I, O, Q).
pub fn vin_matches(vin: &[u8], prefixes: &[&str]) -> bool {
    vin.len() == 17
        && vin.iter().all(|c| {
            (c.is_ascii_digit() || c.is_ascii_uppercase()) && !matches!(c, b'I' | b'O' | b'Q')
        })
        && prefixes
            .iter()
            .any(|p| !p.is_empty() && vin.starts_with(p.as_bytes()))
}

// ── CAN Filter ────────────────────────────────────────────────────────────────

/// Returns the raw numeric value of a CAN identifier.
pub(crate) fn id_raw(id: embedded_can::Id) -> u32 {
    match id {
        embedded_can::Id::Standard(sid) => sid.as_raw() as u32,
        embedded_can::Id::Extended(eid) => eid.as_raw(),
    }
}

/// Software acceptance filter. Returns `true` if `frame` matches at least one
/// entry in `filters` on the same `bus_id`:
///   `(frame_id_raw & mask) == (filter_id_raw & mask)`
///
/// This is the software second-pass used by both board drivers. MCP2515 also
/// applies a hardware first-pass programmed from the same `filters` list;
/// TWAI uses accept-all hardware and relies solely on this function.
pub fn passes_filter(frame: &CanFrame, filters: &[CanFilter]) -> bool {
    let frame_raw = id_raw(frame.id);
    filters
        .iter()
        .filter(|f| f.bus_id == frame.bus_id)
        .any(|f| (frame_raw & f.mask) == (id_raw(f.id) & f.mask))
}
