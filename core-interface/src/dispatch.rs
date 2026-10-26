use alloc::string::String;
use alloc::vec::Vec;
use portable_atomic::Ordering;

use crate::channels::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_RX_CHANNEL, BLE_TX_CHANNEL, MQTT_RX_CHANNEL,
    MQTT_TX_CHANNEL, SYSTEM_COMMAND_CHANNEL,
};
use crate::proto;
use crate::types::{InboundCommand, Transport};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

static BLE_CONTROLLER_LEASE_TTL_S: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(15);

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

/// Answers a CAN read-write policy command. `Err` carries the reason shown to the user.
async fn send_system_command_response(message_id: u64, result: Result<(), &'static str>) {
    let (success, error_message, status_code) = match result {
        Ok(()) => (true, String::new(), proto::CommandStatusCode::Ok),
        Err(reason) => (
            false,
            String::from(reason),
            proto::CommandStatusCode::Failed,
        ),
    };
    BLE_TX_CHANNEL
        .sender()
        .send(proto::DeviceToApp {
            timestamp_ms: embassy_time::Instant::now().as_millis(),
            platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
            payload: Some(proto::device_to_app::Payload::CommandResponse(
                proto::CommandResponse {
                    message_id,
                    success,
                    error_message,
                    response_data: None,
                    status_code: status_code as i32,
                },
            )),
        })
        .await;
}

/// The CAN read-write policy commands change whether the device may transmit on the
/// car's bus, so only a bonded phone may send them. `source_device_id` is the one the
/// transport stamped, never the app's: on ESP, a phone that isn't bonded can still write
/// while the pairing window is open.
const NOT_BONDED: &str = "command rejected: only a bonded phone can change CAN read-write mode";

async fn handle_can_read_write_approval(
    source_device_id: &[u8],
    approved: bool,
) -> Result<(), &'static str> {
    if !crate::pairing::is_phone_paired(source_device_id).await {
        return Err(NOT_BONDED);
    }
    if approved {
        crate::can_access::approve_can_read_write().map_err(|e| e.message())
    } else {
        crate::can_access::revoke_can_read_write_approval();
        Ok(())
    }
}

async fn handle_can_dev_write_unlock(
    source_device_id: &[u8],
    unlocked: bool,
) -> Result<(), &'static str> {
    if !crate::pairing::is_phone_paired(source_device_id).await {
        return Err(NOT_BONDED);
    }
    crate::can_access::set_can_dev_write_unlocked(unlocked).map_err(|e| e.message())
}

/// Vehicle commands act on the car, so they are refused while the device must not
/// transmit (the CAN read-write policy, see `can_access.rs`), with the reason. Refusing
/// them here means every vehicle, the simulator included, reports a read-only device
/// truthfully instead of claiming a command it never sent worked. System commands
/// (approval, dev lock, pairing) are not gated.
fn read_only_rejection(message_id: u64) -> Option<proto::DeviceToApp> {
    let reason = crate::can_access::can_read_only_reason()?;
    Some(proto::DeviceToApp {
        timestamp_ms: embassy_time::Instant::now().as_millis(),
        platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
        payload: Some(proto::device_to_app::Payload::CommandResponse(
            proto::CommandResponse {
                message_id,
                success: false,
                error_message: alloc::format!(
                    "command rejected: the device is read-only: {reason}"
                ),
                response_data: None,
                status_code: proto::CommandStatusCode::Failed as i32,
            },
        )),
    })
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
///   - `set_can_debug_enabled` / `update_can_debug_filters` → dropped (deprecated: the
///     BLE CAN debug stream is gone, CAN logging goes to SavvyCAN instead)
///   - `set_can_read_write_approval` / `set_can_dev_write_unlock` → handled here (the CAN
///     read-write policy), from a bonded phone only, and answered with a `CommandResponse`
/// - `basic_command_bytes` → `BASIC_CMD_CHANNEL` (Transport::Ble)
/// - `advanced_command_bytes` → `ADVANCED_CMD_CHANNEL` (Transport::Ble)
///
/// Basic and advanced commands are answered with a failed `CommandResponse` instead,
/// giving the reason, while the device is CAN read-only.
///
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
            #[allow(deprecated)]
            Some(proto::system_command::Action::SetCanDebugEnabled(_))
            | Some(proto::system_command::Action::UpdateCanDebugFilters(_)) => {}
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
            Some(proto::system_command::Action::SetCanReadWriteApproval(cmd)) => {
                let result = handle_can_read_write_approval(&source_device_id, cmd.approved).await;
                send_system_command_response(msg.message_id, result).await;
            }
            Some(proto::system_command::Action::SetCanDevWriteUnlock(cmd)) => {
                let result = handle_can_dev_write_unlock(&source_device_id, cmd.unlocked).await;
                send_system_command_response(msg.message_id, result).await;
            }
            None => {}
        },
        Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) => {
            if let Some(rejection) = read_only_rejection(msg.message_id) {
                BLE_TX_CHANNEL.sender().send(rejection).await;
                return;
            }
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
            if let Some(rejection) = read_only_rejection(msg.message_id) {
                BLE_TX_CHANNEL.sender().send(rejection).await;
                return;
            }
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

/// Whether commands arriving over MQTT are authenticated. They are not: anyone
/// with the broker credentials (which ship in the app) could publish one, or
/// replay a captured one. Until they are (contracts#8, firmware#16), basic
/// commands from MQTT are rejected. Remove the gate together with this flag.
const MQTT_COMMANDS_AUTHENTICATED: bool = false;

/// Answers a rejected MQTT command. `try_send`: the MQTT driver must never be
/// stalled by a full queue, and the app can retry or notice the missing reply.
fn send_mqtt_unauthenticated_response(message_id: u64) {
    let _ = MQTT_TX_CHANNEL.try_send(proto::DeviceToApp {
        timestamp_ms: embassy_time::Instant::now().as_millis(),
        platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
        payload: Some(proto::device_to_app::Payload::CommandResponse(
            proto::CommandResponse {
                message_id,
                success: false,
                error_message: String::from(
                    "command rejected: commands over MQTT are not authenticated yet, use BLE",
                ),
                response_data: None,
                status_code: proto::CommandStatusCode::Failed as i32,
            },
        )),
    });
}

/// Processes a single `AppToDevice` message arriving from the MQTT driver.
/// Validates `platform_id` and routes `basic_command_bytes` →
/// `BASIC_CMD_CHANNEL` (Transport::Mqtt) once MQTT commands are authenticated;
/// until then they are answered with a failed `CommandResponse` instead.
/// `SystemCommand` and `advanced_command_bytes` are silently dropped — both
/// are restricted to BLE only. That includes the CAN read-write approval and the dev
/// lock: MQTT can never approve or unlock read-write mode.
/// `heartbeat` carries no payload and is not routed; its presence alone resets
/// the MQTT activity window so state updates continue at full rate.
pub async fn handle_mqtt_message(msg: proto::AppToDevice) {
    if msg.platform_id != crate::PLATFORM_ID.load(Ordering::Relaxed) {
        return;
    }
    // Any valid inbound MQTT message (command or app heartbeat) keeps the
    // activity window alive so state updates flow at full rate.
    crate::routing::record_mqtt_activity((embassy_time::Instant::now().as_millis() / 1000) as u32);
    if let Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) = msg.payload {
        if !MQTT_COMMANDS_AUTHENTICATED {
            send_mqtt_unauthenticated_response(msg.message_id);
            return;
        }
        if let Some(rejection) = read_only_rejection(msg.message_id) {
            let _ = MQTT_TX_CHANNEL.try_send(rejection);
            return;
        }
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
