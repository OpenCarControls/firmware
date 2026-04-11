#![cfg_attr(not(test), no_std)]
extern crate alloc;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.core.v1.rs"));
}

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
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

// ── CAN Filter ────────────────────────────────────────────────────────────────

/// Returns the raw numeric value of a CAN identifier.
fn id_raw(id: embedded_can::Id) -> u32 {
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

/// Processes a single `AppToDevice` message arriving from the BLE driver.
/// Validates `platform_id` and routes each payload variant to the correct channel:
/// - `system_command` → `SYSTEM_COMMAND_CHANNEL`
/// - `basic_command_bytes` → `BASIC_CMD_CHANNEL` (Transport::Ble)
/// - `advanced_command_bytes` → `ADVANCED_CMD_CHANNEL` (Transport::Ble)
/// Messages with a mismatched `platform_id` are silently dropped.
pub async fn handle_ble_message(msg: proto::AppToDevice) {
    if msg.platform_id != PLATFORM_ID.load(Ordering::Relaxed) {
        return;
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

#[embassy_executor::task]
pub async fn process_ble_commands_task() {
    let receiver = BLE_RX_CHANNEL.receiver();
    loop {
        handle_ble_message(receiver.receive().await).await;
    }
}

/// Processes a single `AppToDevice` message arriving from the MQTT driver.
/// Validates `platform_id` and routes `basic_command_bytes` →
/// `BASIC_CMD_CHANNEL` (Transport::Mqtt).
/// `SystemCommand` and `advanced_command_bytes` are silently dropped — both
/// are restricted to BLE only.
pub async fn handle_mqtt_message(msg: proto::AppToDevice) {
    if msg.platform_id != PLATFORM_ID.load(Ordering::Relaxed) {
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

// ── Response & State Router Tasks ─────────────────────────────────────────────

/// Routes a single command response to BLE or MQTT based on the transport tag.
/// `timestamp_ms` is passed explicitly so callers (and tests) control the clock.
pub async fn route_single_response(
    transport: Transport,
    response: proto::CommandResponse,
    timestamp_ms: u64,
) {
    let msg = proto::DeviceToApp {
        timestamp_ms,
        platform_id: PLATFORM_ID.load(Ordering::Relaxed),
        payload: Some(proto::device_to_app::Payload::CommandResponse(response)),
    };
    match transport {
        Transport::Ble => BLE_TX_CHANNEL.sender().send(msg).await,
        Transport::Mqtt => MQTT_TX_CHANNEL.sender().send(msg).await,
    }
}

/// Reads command responses from `CMD_RESP_CHANNEL` and routes each as a
/// `DeviceToApp(command_response)` to BLE or MQTT based on the transport tag.
///
/// `timestamp_ms` reflects device uptime. SNTP wall-clock sync is future work.
#[embassy_executor::task]
pub async fn route_responses_task() {
    let receiver = CMD_RESP_CHANNEL.receiver();
    loop {
        let (transport, response) = receiver.receive().await;
        route_single_response(transport, response, Instant::now().as_millis()).await;
    }
}

/// Publishes a single vehicle state update.
/// - BLE receives the full state (basic + advanced).
/// - MQTT receives basic only (`advanced_state_bytes` is always empty).
/// `timestamp_ms` is passed explicitly so callers (and tests) control the clock.
pub async fn publish_single_state(payload: VehicleStatePayload, timestamp_ms: u64) {
    let pid = PLATFORM_ID.load(Ordering::Relaxed);

    // BLE: full state (basic + advanced)
    BLE_TX_CHANNEL
        .sender()
        .send(proto::DeviceToApp {
            timestamp_ms,
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
            timestamp_ms,
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
        publish_single_state(payload, Instant::now().as_millis()).await;
    }
}
