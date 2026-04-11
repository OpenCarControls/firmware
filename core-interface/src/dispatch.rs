use alloc::vec::Vec;
use core::sync::atomic::Ordering;

use crate::can_debug;
use crate::channels::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_RX_CHANNEL, MQTT_RX_CHANNEL,
    SYSTEM_COMMAND_CHANNEL,
};
use crate::proto;
use crate::types::{CanDebugFilter, InboundCommand, Transport};

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
    match msg.payload {
        Some(proto::app_to_device::Payload::SystemCommand(cmd)) => {
            match cmd.action {
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
                None => {}
            }
        }
        Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) => {
            BASIC_CMD_CHANNEL
                .sender()
                .send(InboundCommand {
                    message_id: msg.message_id,
                    transport: Transport::Ble,
                    bytes,
                })
                .await;
        }
        Some(proto::app_to_device::Payload::AdvancedCommandBytes(bytes)) => {
            ADVANCED_CMD_CHANNEL
                .sender()
                .send(InboundCommand {
                    message_id: msg.message_id,
                    transport: Transport::Ble,
                    bytes,
                })
                .await;
        }
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
pub async fn handle_mqtt_message(msg: proto::AppToDevice) {
    if msg.platform_id != crate::PLATFORM_ID.load(Ordering::Relaxed) {
        return;
    }
    if let Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) = msg.payload {
        BASIC_CMD_CHANNEL
            .sender()
            .send(InboundCommand {
                message_id: msg.message_id,
                transport: Transport::Mqtt,
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
