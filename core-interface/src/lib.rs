#![cfg_attr(not(test), no_std)]
extern crate alloc;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.core.v1.rs"));
}

mod can;
mod can_debug;
mod channels;
mod dispatch;
mod lifecycle;
mod pairing;
mod routing;
mod types;

pub mod ble;

use core::sync::atomic::{AtomicU32, Ordering};

// ── Platform ID ───────────────────────────────────────────────────────────────

pub(crate) static PLATFORM_ID: AtomicU32 = AtomicU32::new(0);

/// Must be called once at boot, before spawning any tasks, with the CRC32
/// platform_id from the vehicle's `meta.toml` (injected by xtask).
pub fn init(platform_id: u32) {
    PLATFORM_ID.store(platform_id, Ordering::Relaxed);
}

/// Returns the platform ID stored at boot. Useful for building outbound
/// `DeviceToApp` messages outside of `core-interface` (e.g. board-level BLE
/// transport tasks that need to emit system events).
pub fn platform_id() -> u32 {
    PLATFORM_ID.load(Ordering::Relaxed)
}

// ── Public re-exports ─────────────────────────────────────────────────────────

// Types
pub use types::{
    CanDebugFilter, CanFilter, CanFrame, CanRawCapture, InboundCommand, Transport,
    VehicleStatePayload,
};

// Channels
pub use channels::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_RX_CHANNEL, BLE_TX_CHANNEL, CAN_DEBUG_RX_CHANNEL,
    CAN_RX_CHANNEL, CAN_TX_CHANNEL, CMD_RESP_CHANNEL, MQTT_RX_CHANNEL, MQTT_TX_CHANNEL,
    SYSTEM_COMMAND_CHANNEL, VEHICLE_STATE_CHANNEL,
};

// CAN filter & read-only
pub use can::{is_can_read_only, passes_filter, set_can_read_only};

// CAN debug
pub use can_debug::{
    can_debug_wants_bus, debug_dropped_count, debug_filter_count, increment_can_debug_dropped,
    is_can_debug_active, publish_can_debug_task, publish_single_debug_batch,
};

// Pairing registry
pub use pairing::{
    add_paired_phone, ble_max_bonded_phones, clear_paired_phones, is_phone_paired,
    list_paired_phones, paired_phone_count, remove_paired_phone, set_ble_max_bonded_phones,
};

// Pairing lifecycle state
pub use lifecycle::{close_pairing_window, is_pairing_window_open, open_pairing_window_for};

// Dispatch tasks
pub use dispatch::{
    handle_ble_message, handle_mqtt_message, process_ble_commands_task, process_mqtt_commands_task,
    reset_ble_controller_lease_for_tests, set_ble_controller_lease_ttl_s,
};

// Routing tasks
pub use routing::{
    publish_single_state, publish_state_task, record_mqtt_activity,
    reset_mqtt_throttle_for_tests, route_responses_task, route_single_response,
};
