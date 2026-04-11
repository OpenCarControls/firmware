use crate::proto;
use crate::types::{CanFrame, CanRawCapture, InboundCommand, Transport, VehicleStatePayload};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

// ── Static Channels ───────────────────────────────────────────────────────────

/// Outbound state/response messages toward the BLE driver task.
pub static BLE_TX_CHANNEL: Channel<CriticalSectionRawMutex, proto::DeviceToApp, 4> = Channel::new();
/// Inbound commands arriving from the BLE driver task.
pub static BLE_RX_CHANNEL: Channel<CriticalSectionRawMutex, proto::AppToDevice, 4> = Channel::new();
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
pub static BASIC_CMD_CHANNEL: Channel<CriticalSectionRawMutex, InboundCommand, 4> = Channel::new();
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
/// Raw CAN frames captured before vehicle filtering, consumed by `publish_can_debug_task`.
pub static CAN_DEBUG_RX_CHANNEL: Channel<CriticalSectionRawMutex, CanRawCapture, 64> =
    Channel::new();
