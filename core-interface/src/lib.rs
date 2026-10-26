#![cfg_attr(not(test), no_std)]
extern crate alloc;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.core.v1.rs"));
}

mod can;
mod can_access;
mod can_log;
mod channels;
mod dispatch;
mod lifecycle;
mod pairing;
mod routing;
#[cfg(debug_assertions)]
mod simulation;
mod types;

pub mod ble;

use portable_atomic::{AtomicU32, Ordering};

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
pub use types::{CanFilter, CanFrame, InboundCommand, Transport, VehicleStatePayload};

// Channels
pub use channels::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_RX_CHANNEL, BLE_TX_CHANNEL, CAN_RX_CHANNEL,
    CAN_TX_CHANNELS, CMD_RESP_CHANNEL, MAX_CAN_BUSES, MQTT_RX_CHANNEL, MQTT_TX_CHANNEL,
    SYSTEM_COMMAND_CHANNEL, VEHICLE_STATE_CHANNEL, can_tx_channel, send_can_frame,
};

// CAN filter & read-only
pub use can::{
    CanReadOnlyReceiver, can_read_only_receiver, confirm_car_identity, declare_can_fd_bus,
    is_can_bus_read_only, is_can_fd_bus, is_can_read_only, is_car_identity_confirmed,
    passes_filter, revoke_car_identity, set_can_read_only, vin_matches,
};

// CAN read-write policy (car fingerprint, owner approval, dev lock)
pub use can_access::{
    CAN_ACCESS_REPEAT_MS, CanAccessError, CarDigest, approve_can_read_write, can_access_state,
    can_access_state_update, can_read_only_reason, can_read_write_approval,
    can_read_write_approval_to_store, confirm_car_fingerprint, declare_production_build,
    has_can_dev_write_lock, is_can_dev_write_unlocked, is_car_fingerprint_confirmed,
    publish_can_access_state, publish_can_access_task, resend_can_access_state,
    reset_can_access_for_tests, restore_can_read_write_approval, revoke_can_read_write_approval,
    set_can_dev_write_unlocked, try_take_can_read_write_approval_to_store,
};

// CAN logging (SavvyCAN / GVRET)
pub use can_log::{
    CAN_LOG_CHANNEL, CanLogFrame, CanLogSession, GvretCommand, GvretParser, SessionEnd,
    can_log_dropped_count, encode_gvret_frame, is_can_logging, log_can_frame, run_can_log_session,
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
    ble_state_update, latest_ble_state, publish_single_state, publish_state_task,
    record_mqtt_activity, reset_latest_ble_state_for_tests, reset_mqtt_throttle_for_tests,
    route_responses_task, route_single_response,
};

// Simulation
#[cfg(debug_assertions)]
pub use simulation::{is_simulation_enabled, set_simulation_enabled};
