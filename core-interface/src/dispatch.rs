use alloc::string::String;
use alloc::vec::Vec;
use core::sync::atomic::Ordering;

use crate::can_debug;
use crate::channels::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_RX_CHANNEL, BLE_TX_CHANNEL, MQTT_RX_CHANNEL,
    SYSTEM_COMMAND_CHANNEL,
};
use crate::proto;
use crate::types::{CanDebugFilter, InboundCommand, Transport};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

static BLE_CONTROLLER_LEASE_TTL_S: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(15);

#[derive(Default)]
struct BleControllerLease {
    owner_device_id: Vec<u8>,
    expires_at_ms: u64,
}

static BLE_CONTROLLER_LEASE: Mutex<CriticalSectionRawMutex, BleControllerLease> =
    Mutex::new(BleControllerLease {
        owner_device_id: Vec::new(),
        expires_at_ms: 0,
    });

pub fn set_ble_controller_lease_ttl_s(seconds: u32) {
    if seconds > 0 {
        BLE_CONTROLLER_LEASE_TTL_S.store(seconds, Ordering::Relaxed);
    }
}

async fn ble_controller_allows(source_device_id: &[u8], now_ms: u64) -> bool {
    if source_device_id.is_empty() {
        return false;
    }
    let ttl_ms = (BLE_CONTROLLER_LEASE_TTL_S.load(Ordering::Relaxed) as u64) * 1_000;
    let mut lease = BLE_CONTROLLER_LEASE.lock().await;
    if lease.owner_device_id.is_empty()
        || now_ms >= lease.expires_at_ms
        || lease.owner_device_id.as_slice() == source_device_id
    {
        lease.owner_device_id.clear();
        lease.owner_device_id.extend_from_slice(source_device_id);
        lease.expires_at_ms = now_ms.saturating_add(ttl_ms);
        return true;
    }
    false
}

async fn send_not_controller_response(message_id: u64) {
    BLE_TX_CHANNEL
        .sender()
        .send(proto::DeviceToApp {
            timestamp_ms: embassy_time::Instant::now().as_millis(),
            platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
            payload: Some(proto::device_to_app::Payload::CommandResponse(
                proto::CommandResponse {
                    message_id,
                    success: false,
                    error_message: String::from(
                        "command rejected: active controller lease held by another phone",
                    ),
                    response_data: None,
                    status_code: proto::CommandStatusCode::RejectedNotController as i32,
                },
            )),
        })
        .await;
}

pub async fn reset_ble_controller_lease_for_tests() {
    let mut lease = BLE_CONTROLLER_LEASE.lock().await;
    lease.owner_device_id.clear();
    lease.expires_at_ms = 0;
}

// ── BLE Dispatcher ────────────────────────────────────────────────────────────

/// Processes a single `AppToDevice` message arriving from the BLE driver.
/// Validates `platform_id` and routes each payload variant to the correct channel:
/// - `system_command`:
///   - `restart_command` → `SYSTEM_COMMAND_CHANNEL` (board handles it)
///   - `set_can_debug_enabled` → updates CAN debug state via `can_debug::enable/disable`
///   - `update_can_debug_filters` → replaces blocklist via `can_debug::update_can_debug_filters`
/// - `basic_command_bytes` → `BASIC_CMD_CHANNEL` (Transport::Ble)
/// - `advanced_command_bytes` → `ADVANCED_CMD_CHANNEL` (Transport::Ble)
/// Messages with a mismatched `platform_id` are silently dropped.
pub async fn handle_ble_message(msg: proto::AppToDevice) {
    if msg.platform_id != crate::PLATFORM_ID.load(Ordering::Relaxed) {
        return;
    }
    let source_device_id = msg.source_device_id.clone();
    match msg.payload {
        Some(proto::app_to_device::Payload::SystemCommand(cmd)) => match cmd.action {
            Some(proto::system_command::Action::RestartCommand(restart)) => {
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::RestartCommand(restart)),
                    })
                    .await;
            }
            Some(proto::system_command::Action::SetCanDebugEnabled(req)) => {
                if req.enabled {
                    can_debug::enable_can_debug(&req.bus_ids).await;
                } else {
                    can_debug::disable_can_debug();
                }
            }
            Some(proto::system_command::Action::UpdateCanDebugFilters(req)) => {
                let new: Vec<CanDebugFilter> = req
                    .filters
                    .into_iter()
                    .map(|f| CanDebugFilter {
                        can_id: f.can_id,
                        is_extended_id: f.is_extended_id,
                        mask: f.mask,
                    })
                    .collect();
                can_debug::update_can_debug_filters(new).await;
            }
            Some(proto::system_command::Action::ListPairedPhones(cmd)) => {
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::ListPairedPhones(cmd)),
                    })
                    .await;
            }
            Some(proto::system_command::Action::RemovePairedPhone(cmd)) => {
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::RemovePairedPhone(cmd)),
                    })
                    .await;
            }
            Some(proto::system_command::Action::ClearPairedPhones(cmd)) => {
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::ClearPairedPhones(cmd)),
                    })
                    .await;
            }
            Some(proto::system_command::Action::UpsertPairedPhone(cmd)) => {
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::UpsertPairedPhone(cmd)),
                    })
                    .await;
            }
            None => {}
        },
        Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) => {
            if !ble_controller_allows(&source_device_id, embassy_time::Instant::now().as_millis())
                .await
            {
                send_not_controller_response(msg.message_id).await;
                return;
            }
            BASIC_CMD_CHANNEL
                .sender()
                .send(InboundCommand {
                    message_id: msg.message_id,
                    transport: Transport::Ble,
                    source_device_id,
                    bytes,
                })
                .await;
        }
        Some(proto::app_to_device::Payload::AdvancedCommandBytes(bytes)) => {
            if !ble_controller_allows(&source_device_id, embassy_time::Instant::now().as_millis())
                .await
            {
                send_not_controller_response(msg.message_id).await;
                return;
            }
            ADVANCED_CMD_CHANNEL
                .sender()
                .send(InboundCommand {
                    message_id: msg.message_id,
                    transport: Transport::Ble,
                    source_device_id,
                    bytes,
                })
                .await;
        }
        // Heartbeat is an MQTT-only concept — silently ignored on BLE.
        Some(proto::app_to_device::Payload::Heartbeat(_)) => {}
        None => {}
    }
}

#[embassy_executor::task]
pub async fn process_ble_commands_task() {
    let receiver = BLE_RX_CHANNEL.receiver();
    loop {
        handle_ble_message(receiver.receive().await).await;
    }
}

// ── MQTT Dispatcher ───────────────────────────────────────────────────────────

/// Processes a single `AppToDevice` message arriving from the MQTT driver.
/// Validates `platform_id` and routes `basic_command_bytes` →
/// `BASIC_CMD_CHANNEL` (Transport::Mqtt).
/// `SystemCommand` and `advanced_command_bytes` are silently dropped — both
/// are restricted to BLE only.
/// `heartbeat` carries no payload and is not routed; its presence alone resets
/// the MQTT activity window so state updates continue at full rate.
pub async fn handle_mqtt_message(msg: proto::AppToDevice) {
    if msg.platform_id != crate::PLATFORM_ID.load(Ordering::Relaxed) {
        return;
    }
    // Any valid inbound MQTT message (command or app heartbeat) keeps the
    // activity window alive so state updates flow at full rate.
    crate::routing::record_mqtt_activity(
        (embassy_time::Instant::now().as_millis() / 1000) as u32,
    );
    if let Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) = msg.payload {
        BASIC_CMD_CHANNEL
            .sender()
            .send(InboundCommand {
                message_id: msg.message_id,
                transport: Transport::Mqtt,
                source_device_id: Vec::new(),
                bytes,
            })
            .await;
    }
}

#[embassy_executor::task]
pub async fn process_mqtt_commands_task() {
    let receiver = MQTT_RX_CHANNEL.receiver();
    loop {
        handle_mqtt_message(receiver.receive().await).await;
    }
}
