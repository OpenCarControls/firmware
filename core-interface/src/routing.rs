use alloc::vec;
use core::sync::atomic::Ordering;

use embassy_time::Instant;

use crate::channels::{BLE_TX_CHANNEL, CMD_RESP_CHANNEL, MQTT_TX_CHANNEL, VEHICLE_STATE_CHANNEL};
use crate::proto;
use crate::types::{Transport, VehicleStatePayload};

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
pub async fn publish_single_state(payload: VehicleStatePayload, timestamp_ms: u64) {
    let pid = crate::PLATFORM_ID.load(Ordering::Relaxed);

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
                        advanced_state_bytes: vec![],
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
