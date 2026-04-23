use alloc::vec::Vec;

// ── Shared Types ──────────────────────────────────────────────────────────────

/// Identifies which transport protocol an inbound command arrived on, allowing
/// the vehicle task to route the response back to the same transport.
#[derive(Clone, Copy)]
pub enum Transport {
    Ble,
    Mqtt,
}

/// A decoded inbound command payload forwarded to a vehicle task.
pub struct InboundCommand {
    /// Original `message_id` from `AppToDevice`, for matching the response.
    pub message_id: u64,
    /// Which transport delivered this command.
    pub transport: Transport,
    /// Optional transport-level device identifier (BLE source device).
    pub source_device_id: Vec<u8>,
    /// Raw encoded bytes of the vehicle-specific command proto message.
    pub bytes: Vec<u8>,
}

/// A CAN frame received from or to be sent on a CAN bus.
pub struct CanFrame {
    /// Index into the `[[can_buses]]` config array (0-based). Identifies which
    /// physical bus this frame belongs to.
    pub bus_id: u8,
    /// The CAN identifier (standard 11-bit or extended 29-bit).
    pub id: embedded_can::Id,
    /// Frame payload bytes.
    pub data: [u8; 8],
    /// Data length code — number of valid bytes in `data` (0–8).
    pub dlc: u8,
}

/// A CAN hardware acceptance filter. The frame passes if:
///   `(frame_id_raw & mask) == (filter_id_raw & mask)`
/// where `mask` bit = 1 means that bit must match.
/// Use `mask = 0x7FF` for exact standard-ID match, `mask = u32::MAX` for exact
/// extended-ID match, `mask = 0` to accept every frame on this bus.
pub struct CanFilter {
    /// Which bus this filter applies to (matches `CanFrame::bus_id`).
    pub bus_id: u8,
    /// The CAN identifier pattern to match against.
    pub id: embedded_can::Id,
    /// Acceptance mask — bits set to 1 must match between frame ID and filter ID.
    pub mask: u32,
}

/// Encoded vehicle state produced by the vehicle crate and consumed by
/// `publish_state_task` to build outbound `DeviceToApp` messages.
pub struct VehicleStatePayload {
    /// Encoded `BasicState` bytes — sent over both BLE and MQTT.
    pub basic: Vec<u8>,
    /// Encoded `AdvancedState` bytes — sent over BLE only.
    pub advanced: Vec<u8>,
}

/// Raw CAN frame captured by a board driver before vehicle-specific filtering,
/// forwarded to `CAN_DEBUG_RX_CHANNEL` when CAN debug streaming is active.
pub struct CanRawCapture {
    /// Device uptime in milliseconds at the moment of frame reception — set by
    /// the board driver so inter-frame timing within a batch is accurate.
    pub timestamp_ms: u64,
    /// 0-based bus index identifying which physical CAN bus this frame arrived on.
    pub bus_id: u8,
    /// The CAN frame identifier (standard or extended).
    pub id: embedded_can::Id,
    /// Raw frame payload bytes (always 8 bytes; only `dlc` bytes are valid).
    pub data: [u8; 8],
    /// Number of valid bytes in `data` (0–8).
    pub dlc: u8,
}

/// A single CAN debug blocklist entry. A frame is excluded if:
///   `(frame_raw_id & mask) == (can_id & mask)` AND `is_extended_id` matches the frame type.
///
/// Applies across all buses being observed — no per-bus scoping.
pub struct CanDebugFilter {
    /// The CAN identifier value to match against.
    pub can_id: u32,
    /// `true` to target extended (29-bit) IDs; `false` for standard (11-bit) IDs.
    pub is_extended_id: bool,
    /// Acceptance mask. See `passes_filter` docs for semantics.
    pub mask: u32,
}
