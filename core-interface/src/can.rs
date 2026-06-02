use core::sync::atomic::{AtomicBool, Ordering};

use crate::types::{CanFilter, CanFrame};

// ── CAN Read-Only Mode ────────────────────────────────────────────────────────

/// When `true`, the board CAN driver loops silently drop all outbound TX frames
/// instead of transmitting them on the bus. Defaults to `true` at boot so no
/// CAN frame can be sent until the vehicle crate explicitly unlocks the bus
/// after validating the connected car.
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
