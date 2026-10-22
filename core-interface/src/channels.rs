use crate::proto;
use crate::types::{CanFrame, InboundCommand, Transport, VehicleStatePayload};
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
/// Maximum number of physical CAN buses a board can drive.
pub const MAX_CAN_BUSES: usize = 4;
/// Per-bus CAN transmit queues, indexed by `CanFrame::bus_id`. Vehicle tasks enqueue with
/// [`send_can_frame`]; each board CAN driver drains only its own bus's channel, so no
/// driver ever has to look at (or re-queue) another bus's frames.
pub static CAN_TX_CHANNELS: [Channel<CriticalSectionRawMutex, CanFrame, 16>; MAX_CAN_BUSES] =
    [const { Channel::new() }; MAX_CAN_BUSES];

/// Returns the transmit channel for `bus_id`, or `None` if the bus does not exist.
pub fn can_tx_channel(
    bus_id: u8,
) -> Option<&'static Channel<CriticalSectionRawMutex, CanFrame, 16>> {
    CAN_TX_CHANNELS.get(bus_id as usize)
}

/// Queues `frame` for transmission on its `bus_id`. Returns `false` (and drops the frame)
/// if that bus does not exist. Whether the frame reaches the wire is still decided by the
/// board driver's CAN read-only check.
pub async fn send_can_frame(frame: CanFrame) -> bool {
    match can_tx_channel(frame.bus_id) {
        Some(channel) => {
            channel.send(frame).await;
            true
        }
        None => false,
    }
}
