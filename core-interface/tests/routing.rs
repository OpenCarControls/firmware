use core_interface::{
    BLE_TX_CHANNEL, MQTT_TX_CHANNEL, Transport, VehicleStatePayload, init, proto,
    publish_single_state, route_single_response,
};

const PLATFORM_ID: u32 = 0xCAFE_BABE;
const TS: u64 = 12345;

/// Drain both outbound channels so earlier tests don't pollute later ones when
/// running under `cargo test` (single process, shared statics).
fn drain_tx_channels() {
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
}

fn make_response(message_id: u64) -> proto::CommandResponse {
    proto::CommandResponse {
        message_id,
        success: true,
        error_message: String::new(),
        response_data: None,
    }
}

// ── route_single_response ─────────────────────────────────────────────────────

#[test]
fn ble_response_goes_to_ble_tx_only() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let resp = make_response(1);
    embassy_futures::block_on(route_single_response(Transport::Ble, resp, TS));
    let msg = BLE_TX_CHANNEL.try_receive().expect("BLE_TX has no message");
    assert_eq!(msg.timestamp_ms, TS);
    assert!(matches!(
        msg.payload,
        Some(proto::device_to_app::Payload::CommandResponse(_))
    ));
    assert!(MQTT_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_response_goes_to_mqtt_tx_only() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let resp = make_response(2);
    embassy_futures::block_on(route_single_response(Transport::Mqtt, resp, TS));
    let msg = MQTT_TX_CHANNEL
        .try_receive()
        .expect("MQTT_TX has no message");
    assert_eq!(msg.timestamp_ms, TS);
    assert!(matches!(
        msg.payload,
        Some(proto::device_to_app::Payload::CommandResponse(_))
    ));
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn response_carries_correct_platform_id() {
    drain_tx_channels();
    init(PLATFORM_ID);
    embassy_futures::block_on(route_single_response(Transport::Ble, make_response(3), TS));
    let msg = BLE_TX_CHANNEL.try_receive().unwrap();
    assert_eq!(msg.platform_id, PLATFORM_ID);
}

// ── publish_single_state ──────────────────────────────────────────────────────

#[test]
fn ble_receives_full_state_basic_and_advanced() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let payload = VehicleStatePayload {
        basic: vec![0x01, 0x02],
        advanced: vec![0x03, 0x04],
    };
    embassy_futures::block_on(publish_single_state(payload, TS));

    let msg = BLE_TX_CHANNEL
        .try_receive()
        .expect("BLE_TX has no state message");
    match msg.payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => {
            let vs = update
                .vehicle_state
                .expect("no vehicle_state in BLE update");
            assert_eq!(vs.basic_state_bytes, vec![0x01, 0x02]);
            assert_eq!(vs.advanced_state_bytes, vec![0x03, 0x04]);
        }
        other => panic!("unexpected BLE payload: {:?}", other),
    }
}

#[test]
fn mqtt_receives_basic_only_advanced_is_empty() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let payload = VehicleStatePayload {
        basic: vec![0xAA],
        advanced: vec![0xBB, 0xCC],
    };
    embassy_futures::block_on(publish_single_state(payload, TS));

    // drain BLE first
    let _ = BLE_TX_CHANNEL.try_receive();

    let msg = MQTT_TX_CHANNEL
        .try_receive()
        .expect("MQTT_TX has no state message");
    match msg.payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => {
            let vs = update
                .vehicle_state
                .expect("no vehicle_state in MQTT update");
            assert_eq!(vs.basic_state_bytes, vec![0xAA]);
            assert!(
                vs.advanced_state_bytes.is_empty(),
                "MQTT must not include advanced bytes"
            );
        }
        other => panic!("unexpected MQTT payload: {:?}", other),
    }
}

#[test]
fn state_publish_carries_correct_timestamp() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let payload = VehicleStatePayload {
        basic: vec![],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(payload, 9999));
    let ble_msg = BLE_TX_CHANNEL.try_receive().unwrap();
    let mqtt_msg = MQTT_TX_CHANNEL.try_receive().unwrap();
    assert_eq!(ble_msg.timestamp_ms, 9999);
    assert_eq!(mqtt_msg.timestamp_ms, 9999);
}
