#![cfg_attr(not(test), no_std)]
extern crate alloc;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.core.v1.rs"));
}

mod availability;
mod can;
mod can_access;
mod can_log;
mod can_probe;
mod can_tx;
mod channels;
mod dedupe;
mod dispatch;
mod isotp;
mod lifecycle;
mod pairing;
mod phone_trust;
mod requirements;
mod routing;
#[cfg(debug_assertions)]
mod simulation;
mod types;
mod validation;

pub mod ble;

use core::cell::Cell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
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

// ── MQTT client ID ────────────────────────────────────────────────────────────

static MQTT_CLIENT_ID: Mutex<CriticalSectionRawMutex, Cell<Option<&'static str>>> =
    Mutex::new(Cell::new(None));

/// Called once at boot by the generated `main.rs` of a build with MQTT, with the
/// `network.mqtt.client_id` its topics are built from. The app learns it over BLE
/// to use the same topics.
pub fn set_mqtt_client_id(client_id: &'static str) {
    MQTT_CLIENT_ID.lock(|id| id.set(Some(client_id)));
}

/// The device's MQTT client ID, or `None` when it has no MQTT.
pub(crate) fn mqtt_client_id() -> Option<&'static str> {
    MQTT_CLIENT_ID.lock(Cell::get)
}

// ── Public re-exports ─────────────────────────────────────────────────────────

// Types
pub use types::{CanFilter, CanFrame, InboundCommand, Transport, VehicleStatePayload};

// Channels
pub use channels::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_RX_CHANNEL, BLE_STATE_TX, BLE_TX_CHANNEL,
    CAN_RX_CHANNEL, CAN_TX_CHANNELS, CMD_RESP_CHANNEL, MAX_CAN_BUSES, MQTT_RX_CHANNEL,
    MQTT_STATE_TX, MQTT_TX_CHANNEL, SYSTEM_COMMAND_CHANNEL, VEHICLE_STATE_CHANNEL, can_tx_channel,
    send_can_frame,
};

// CAN TX classes (platform frames, command frames)
pub use can_tx::{CanTxFrame, CommandAuthorization};

// CAN filter & read-only
pub use can::{
    CanReadOnlyReceiver, can_read_only_receiver, confirm_car_identity, declare_can_fd_bus,
    is_can_bus_read_only, is_can_fd_bus, is_can_read_only, is_car_identity_confirmed,
    passes_filter, revoke_car_identity, set_can_read_only, vin_matches,
};

// Identity probe (the one transmit before the car is identified) and its UDS client
pub use can_probe::{
    BUS_AWAKE_MS, IdentityProbe, MAX_PROBE_ATTEMPTS, MAX_PROBE_FRAMES, PROBE_SETTLE_MS,
    PROBE_WINDOW_MS, ProbeRefused, ProbeWindow, allow_can_tx, extended_id, is_can_bus_listen_only,
    open_identity_probe, reset_identity_probe_for_tests, standard_id,
};
pub use isotp::{
    ISO_TP_MAX_PAYLOAD, IsoTpSender, UDS_MAX_REQUEST, UDS_MAX_RESPONSE, UdsError, UdsRequest,
    UdsStep, iso_tp_flow_control, uds_single_frame,
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

// Command availability
pub use availability::{
    COMMAND_AVAILABILITY_REPEAT_MS, CommandKind, REPORT_MAX_AGE_MS, ble_command_availability_at,
    command_availability, command_availability_at, command_availability_state_update,
    mqtt_command_availability_at, publish_command_availability, publish_command_availability_mqtt,
    publish_command_availability_task, report_command_availability, report_command_availability_at,
    resend_command_availability, reset_command_availability_for_tests,
};

// Command requirements
pub use requirements::{
    ActivityClock, CommandError, CommandRequirements, Rejection, RepeatGuard, Requirement,
    Snapshot, rejection_response, respond_to_command, try_send_response,
};

// Pairing registry
pub use pairing::{
    add_paired_phone, ble_max_bonded_phones, clear_paired_phones, is_phone_paired,
    list_paired_phones, paired_phone_count, remove_paired_phone, set_ble_max_bonded_phones,
};

// Phone trust (validated phones, the connected BLE phone)
#[cfg(any(test, feature = "test-utils"))]
pub use phone_trust::validate_phone_for_tests;
pub use phone_trust::{
    car_has_validated_phone, encode_phone_validations, forget_phone_validation, is_phone_validated,
    phone_access_state, phone_trust_level, phone_validations_to_store, prune_phone_validations,
    reset_phone_trust_for_tests, restore_phone_validations, set_ble_peer,
    try_take_phone_validations_to_store,
};

// Phone validation sessions
pub use validation::{
    CODE_LENGTH, MAX_CODE_ATTEMPTS, VALIDATION_COOLDOWN_MS, VALIDATION_RESULT_KEEP_MS,
    VALIDATION_SESSION_MS, ValidationChallenge, ValidationError, ValidationKind, ValidationMethod,
    complete_validation_action, complete_validation_action_at, current_validation_challenge,
    declare_validation_methods, reset_phone_validation_for_tests, set_random_source,
    tick_validation, validation_challenge_receiver,
};

// Pairing lifecycle state
pub use lifecycle::{close_pairing_window, is_pairing_window_open, open_pairing_window_for};

// Duplicate commands
pub use dedupe::{
    DEDUPE_CAPACITY, DEDUPE_WINDOW_MS, RecentCommands, reset_recent_commands_for_tests,
};

// Dispatch tasks
pub use dispatch::{
    handle_ble_message, handle_mqtt_message, process_ble_commands_task, process_mqtt_commands_task,
    reset_ble_controller_lease_for_tests, set_ble_controller_lease_ttl_s,
};

// Routing tasks
pub use routing::{
    ble_state_update, is_link_up, latest_ble_state, publish_single_state, publish_state_task,
    record_mqtt_activity, reset_latest_ble_state_for_tests, reset_links_for_tests,
    reset_mqtt_throttle_for_tests, route_responses_task, route_single_response, set_link_up,
};

// Simulation
#[cfg(debug_assertions)]
pub use simulation::{is_simulation_enabled, set_simulation_enabled};
