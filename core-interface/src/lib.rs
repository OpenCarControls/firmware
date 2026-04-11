#![no_std]
extern crate alloc;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.core.v1.rs"));
}

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;

// ── Platform ID ───────────────────────────────────────────────────────────────

static PLATFORM_ID: AtomicU32 = AtomicU32::new(0);

/// Must be called once at boot, before spawning any tasks, with the CRC32
/// platform_id from the vehicle's `meta.toml` (injected by xtask).
pub fn init(platform_id: u32) {
    PLATFORM_ID.store(platform_id, Ordering::Relaxed);
}

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

// ── Static Channels ───────────────────────────────────────────────────────────

/// Outbound state/response messages toward the BLE driver task.
pub static BLE_TX_CHANNEL: Channel<CriticalSectionRawMutex, proto::DeviceToApp, 4> =
    Channel::new();
/// Inbound commands arriving from the BLE driver task.
pub static BLE_RX_CHANNEL: Channel<CriticalSectionRawMutex, proto::AppToDevice, 4> =
    Channel::new();
/// Outbound state/response messages toward the MQTT driver task.
pub static MQTT_TX_CHANNEL: Channel<CriticalSectionRawMutex, proto::DeviceToApp, 4> =
    Channel::new();
/// Inbound commands arriving from the MQTT driver task.
pub static MQTT_RX_CHANNEL: Channel<CriticalSectionRawMutex, proto::AppToDevice, 4> =
    Channel::new();
/// Decoded system commands (restart, etc.) for the board to act on.
pub static SYSTEM_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, proto::SystemCommand, 1> =
    Channel::new();
/// Basic vehicle commands routed to the vehicle task. Sent from BLE and MQTT.
pub static BASIC_CMD_CHANNEL: Channel<CriticalSectionRawMutex, InboundCommand, 4> =
    Channel::new();
/// Advanced vehicle commands routed to the vehicle task. BLE only.
pub static ADVANCED_CMD_CHANNEL: Channel<CriticalSectionRawMutex, InboundCommand, 4> =
    Channel::new();
/// Command responses from the vehicle task, tagged with the originating transport.
pub static CMD_RESP_CHANNEL: Channel<
    CriticalSectionRawMutex,
    (Transport, proto::CommandResponse),
    4,
> = Channel::new();
/// Vehicle state updates produced by the vehicle task, consumed by `publish_state_task`.
pub static VEHICLE_STATE_CHANNEL: Channel<CriticalSectionRawMutex, VehicleStatePayload, 4> =
    Channel::new();
/// CAN frames received by a board CAN driver task and forwarded to the vehicle task.
/// Each frame carries a `bus_id` identifying which physical bus it arrived on.
pub static CAN_RX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();
/// CAN frames produced by the vehicle task and forwarded to the board CAN driver
/// task for transmission. Each frame carries the `bus_id` of the target bus.
pub static CAN_TX_CHANNEL: Channel<CriticalSectionRawMutex, CanFrame, 16> = Channel::new();

// ── Command Dispatcher Tasks ──────────────────────────────────────────────────

/// Receives `AppToDevice` messages from the BLE driver, validates `platform_id`,
/// and routes each payload:
/// - `system_command` → `SYSTEM_COMMAND_CHANNEL`
/// - `basic_command_bytes` → `BASIC_CMD_CHANNEL` (Transport::Ble)
/// - `advanced_command_bytes` → `ADVANCED_CMD_CHANNEL` (Transport::Ble)
#[embassy_executor::task]
pub async fn process_ble_commands_task() {
    let receiver = BLE_RX_CHANNEL.receiver();
    loop {
        let msg = receiver.receive().await;
        if msg.platform_id != PLATFORM_ID.load(Ordering::Relaxed) {
            continue;
        }
        match msg.payload {
            Some(proto::app_to_device::Payload::SystemCommand(cmd)) => {
                SYSTEM_COMMAND_CHANNEL.sender().send(cmd).await;
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
}

/// Receives `AppToDevice` messages from the MQTT driver, validates `platform_id`,
/// and routes `basic_command_bytes` → `BASIC_CMD_CHANNEL` (Transport::Mqtt).
/// `SystemCommand` and `advanced_command_bytes` are silently dropped — both
/// are restricted to BLE only.
#[embassy_executor::task]
pub async fn process_mqtt_commands_task() {
    let receiver = MQTT_RX_CHANNEL.receiver();
    loop {
        let msg = receiver.receive().await;
        if msg.platform_id != PLATFORM_ID.load(Ordering::Relaxed) {
            continue;
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
}

// ── Response & State Router Tasks ─────────────────────────────────────────────

/// Reads command responses from `CMD_RESP_CHANNEL` and routes each as a
/// `DeviceToApp(command_response)` to BLE or MQTT based on the transport tag.
///
/// `timestamp_ms` reflects device uptime. SNTP wall-clock sync is future work.
#[embassy_executor::task]
pub async fn route_responses_task() {
    let receiver = CMD_RESP_CHANNEL.receiver();
    loop {
        let (transport, response) = receiver.receive().await;
        let msg = proto::DeviceToApp {
            timestamp_ms: Instant::now().as_millis(),
            platform_id: PLATFORM_ID.load(Ordering::Relaxed),
            payload: Some(proto::device_to_app::Payload::CommandResponse(response)),
        };
        match transport {
            Transport::Ble => BLE_TX_CHANNEL.sender().send(msg).await,
            Transport::Mqtt => MQTT_TX_CHANNEL.sender().send(msg).await,
        }
    }
}

/// Reads `VehicleStatePayload` from `VEHICLE_STATE_CHANNEL` and publishes two
/// outbound `DeviceToApp(state_update)` messages per update:
/// - BLE: full `VehicleState` (basic + advanced bytes)
/// - MQTT: basic only (`advanced_state_bytes` omitted — BLE-exclusive)
///
/// `timestamp_ms` reflects device uptime. SNTP wall-clock sync is future work.
#[embassy_executor::task]
pub async fn publish_state_task() {
    let receiver = VEHICLE_STATE_CHANNEL.receiver();
    loop {
        let payload = receiver.receive().await;
        let now = Instant::now().as_millis();
        let pid = PLATFORM_ID.load(Ordering::Relaxed);

        // BLE: full state (basic + advanced)
        BLE_TX_CHANNEL
            .sender()
            .send(proto::DeviceToApp {
                timestamp_ms: now,
                platform_id: pid,
                payload: Some(proto::device_to_app::Payload::StateUpdate(
                    proto::StateUpdate {
                        system_state: None,
                        vehicle_state: Some(proto::VehicleState {
                            basic_state_bytes: payload.basic.clone(),
                            advanced_state_bytes: payload.advanced,
                        }),
                    },
                )),
            })
            .await;

        // MQTT: basic only (advanced is BLE-exclusive)
        MQTT_TX_CHANNEL
            .sender()
            .send(proto::DeviceToApp {
                timestamp_ms: now,
                platform_id: pid,
                payload: Some(proto::device_to_app::Payload::StateUpdate(
                    proto::StateUpdate {
                        system_state: None,
                        vehicle_state: Some(proto::VehicleState {
                            basic_state_bytes: payload.basic,
                            advanced_state_bytes: alloc::vec![],
                        }),
                    },
                )),
            })
            .await;
    }
}
