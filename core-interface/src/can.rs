use portable_atomic::{AtomicBool, Ordering};

use crate::types::{CanFilter, CanFrame};

// ── CAN Read-Only Mode ────────────────────────────────────────────────────────

/// When `true`, the CAN buses are read-only: board drivers whose controller supports
/// it (ESP TWAI and MCP2515) run in hardware listen-only mode, so the device never
/// drives the bus at all (no TX, no ACKs, no error frames), and every board driver
/// silently drops outbound TX frames. Defaults to `true` at boot so nothing can reach
/// the bus until the vehicle crate explicitly unlocks it after validating the
/// connected car.
static CAN_READ_ONLY: AtomicBool = AtomicBool::new(true);

/// Returns `true` if the CAN buses are currently in read-only mode.
///
/// Vehicle tasks should check this before deciding whether to attempt a
/// CAN write; if `true`, pushes to `CAN_TX_CHANNEL` will be accepted by
/// the channel but silently dropped by the board driver loop.
pub fn is_can_read_only() -> bool {
    CAN_READ_ONLY.load(Ordering::Relaxed)
}

/// Enables or disables CAN read-only mode.
///
/// Board drivers poll this flag and switch their controller between listen-only and
/// normal mode within a few milliseconds. On ESP TWAI the switch resets the
/// controller, so frames arriving during the switch are lost.
///
/// Call `set_can_read_only(false)` from a vehicle task once inbound CAN frames
/// have been validated to confirm the connected car matches this firmware.
/// Call `set_can_read_only(true)` to re-engage the lock if an error or
/// inconsistent data is detected at any point.
pub fn set_can_read_only(enabled: bool) {
    CAN_READ_ONLY.store(enabled, Ordering::Relaxed);
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
