use alloc::vec;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_time::Instant;

use crate::channels::{BLE_TX_CHANNEL, CMD_RESP_CHANNEL, MQTT_TX_CHANNEL, VEHICLE_STATE_CHANNEL};
use crate::proto;
use crate::types::{Transport, VehicleStatePayload};

// ── MQTT Throttle ─────────────────────────────────────────────────────────────

/// Seconds of silence from any MQTT client before entering idle rate.
const MQTT_ACTIVITY_WINDOW_S: u32 = 35;
/// Minimum gap between MQTT state sends while no client is active.
const MQTT_IDLE_RATE_S: u32 = 30;

/// Uptime seconds of the most recent inbound MQTT message (command or heartbeat).
static LAST_MQTT_RX_SECS: AtomicU32 = AtomicU32::new(0);
/// Uptime seconds of the most recent MQTT state message actually sent.
static LAST_MQTT_STATE_SECS: AtomicU32 = AtomicU32::new(0);

/// Called by `handle_mqtt_message` whenever any valid MQTT message arrives
/// (command, heartbeat, or any other payload). Keeps the activity window alive
/// so state updates continue to flow at full rate.
pub fn record_mqtt_activity(secs: u32) {
    LAST_MQTT_RX_SECS.store(secs, Ordering::Relaxed);
}

/// Resets MQTT throttle state to boot defaults. For use in tests only.
pub fn reset_mqtt_throttle_for_tests() {
    LAST_MQTT_RX_SECS.store(0, Ordering::Relaxed);
    LAST_MQTT_STATE_SECS.store(0, Ordering::Relaxed);
}

// ── Response Router ───────────────────────────────────────────────────────────

/// Routes a single command response to BLE or MQTT based on the transport tag.
/// `timestamp_ms` is passed explicitly so callers (and tests) control the clock.
pub async fn route_single_response(
    transport: Transport,
    response: proto::CommandResponse,
    timestamp_ms: u64,
) {
    let msg = proto::DeviceToApp {
        timestamp_ms,
        platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
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

// ── State Publisher ───────────────────────────────────────────────────────────

/// Publishes a single vehicle state update.
/// - BLE receives the full state (basic + advanced).
/// - MQTT receives basic only (`advanced_state_bytes` is always empty).
/// `timestamp_ms` is passed explicitly so callers (and tests) control the clock.
///
/// Both TX channels are written non-blocking. State updates are ephemeral —
/// the latest reading always supersedes older ones. If a channel is full
/// (e.g. BLE is pull-based and nobody has polled yet, or the MQTT broker is
/// temporarily unreachable), the oldest pending message is evicted to make
/// room for the fresher state. This ensures `publish_state_task` never blocks,
/// which would otherwise stall `VEHICLE_STATE_CHANNEL` and deadlock every
/// producer task (`state_update_task`, command handlers).
pub async fn publish_single_state(payload: VehicleStatePayload, timestamp_ms: u64) {
    let pid = crate::PLATFORM_ID.load(Ordering::Relaxed);

    // BLE: full state (basic + advanced)
    let ble_msg = proto::DeviceToApp {
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
    };
    if let Err(embassy_sync::channel::TrySendError::Full(msg)) = BLE_TX_CHANNEL.try_send(ble_msg) {
        // Channel full — evict the oldest stale state and replace with latest.
        let _ = BLE_TX_CHANNEL.try_receive();
        let _ = BLE_TX_CHANNEL.try_send(msg);
    }

    // MQTT: basic only (advanced is BLE-exclusive), throttled when no client is active.
    let now_secs = (timestamp_ms / 1000) as u32;
    let last_rx = LAST_MQTT_RX_SECS.load(Ordering::Relaxed);
    let mqtt_active = now_secs.saturating_sub(last_rx) <= MQTT_ACTIVITY_WINDOW_S;
    let should_send_mqtt = if mqtt_active {
        true
    } else {
        let last_state = LAST_MQTT_STATE_SECS.load(Ordering::Relaxed);
        now_secs.saturating_sub(last_state) >= MQTT_IDLE_RATE_S
    };

    if should_send_mqtt {
        let mqtt_msg = proto::DeviceToApp {
            timestamp_ms,
            platform_id: pid,
            payload: Some(proto::device_to_app::Payload::StateUpdate(
                proto::StateUpdate {
                    system_state: None,
                    vehicle_state: Some(proto::VehicleState {
                        basic_state_bytes: payload.basic,
                        advanced_state_bytes: vec![],
                    }),
                },
            )),
        };
        if let Err(embassy_sync::channel::TrySendError::Full(msg)) =
            MQTT_TX_CHANNEL.try_send(mqtt_msg)
        {
            // Channel full — evict the oldest stale state and replace with latest.
            let _ = MQTT_TX_CHANNEL.try_receive();
            let _ = MQTT_TX_CHANNEL.try_send(msg);
        }
        LAST_MQTT_STATE_SECS.store(now_secs, Ordering::Relaxed);
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
        publish_single_state(payload, Instant::now().as_millis()).await;
    }
}
