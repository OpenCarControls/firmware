use core_interface::{
    BLE_TX_CHANNEL, MQTT_TX_CHANNEL, Transport, VehicleStatePayload, init, proto,
    publish_single_state, record_mqtt_activity, reset_mqtt_throttle_for_tests,
    route_single_response,
};

const PLATFORM_ID: u32 = 0xCAFE_BABE;
const TS: u64 = 12345;

/// Drain both outbound channels and reset throttle state so earlier tests
/// don't pollute later ones when running under `cargo test` (single process,
/// shared statics).
fn drain_tx_channels() {
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
    reset_mqtt_throttle_for_tests();
}

fn make_response(message_id: u64) -> proto::CommandResponse {
    proto::CommandResponse {
        message_id,
        success: true,
        error_message: String::new(),
        response_data: None,
        status_code: 1,
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
    // timestamp = 9999 ms = 9 s; LAST_MQTT_RX_SECS = 0 → elapsed = 9 s < 35 s → active
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

// ── MQTT throttle ─────────────────────────────────────────────────────────────

#[test]
fn mqtt_suppressed_in_idle_when_within_30s() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // LAST_MQTT_RX_SECS = 0. Publish at t = 40 s → elapsed since RX = 40 > 35 → idle.
    // LAST_MQTT_STATE_SECS = 0 → elapsed since last state = 40 s > 30 s → allowed.
    // Send at t = 40 s first to set LAST_MQTT_STATE_SECS = 40.
    let payload = VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(payload, 40_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    MQTT_TX_CHANNEL.try_receive().unwrap(); // first idle send allowed

    // Now publish at t = 60 s → elapsed since last state = 20 s < 30 s → suppressed.
    let payload2 = VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(payload2, 60_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    assert!(
        MQTT_TX_CHANNEL.try_receive().is_err(),
        "MQTT must be suppressed in idle within 30 s window"
    );
}

#[test]
fn mqtt_allowed_in_idle_after_30s() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // LAST_MQTT_RX_SECS = 0. Publish at t = 100 s → idle.
    // LAST_MQTT_STATE_SECS = 0 → elapsed = 100 s >= 30 s → allowed.
    let payload = VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(payload, 100_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    assert!(
        MQTT_TX_CHANNEL.try_receive().is_ok(),
        "MQTT must be allowed in idle after 30 s"
    );
}

#[test]
fn mqtt_idle_rate_resets_after_send() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // First idle send at t = 100 s — sets LAST_MQTT_STATE_SECS = 100.
    let p = || VehicleStatePayload { basic: vec![0x01], advanced: vec![] };
    embassy_futures::block_on(publish_single_state(p(), 100_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    MQTT_TX_CHANNEL.try_receive().unwrap(); // first send allowed

    // Second send at t = 129 s → elapsed since last state = 29 s < 30 s → suppressed.
    embassy_futures::block_on(publish_single_state(p(), 129_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    assert!(
        MQTT_TX_CHANNEL.try_receive().is_err(),
        "MQTT must be suppressed within 30 s of last idle send"
    );

    // Third send at t = 131 s → elapsed = 31 s >= 30 s → allowed.
    embassy_futures::block_on(publish_single_state(p(), 131_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    assert!(
        MQTT_TX_CHANNEL.try_receive().is_ok(),
        "MQTT must be allowed after 30 s idle gap"
    );
}

#[test]
fn mqtt_not_throttled_when_client_active() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // Simulate heartbeat at t = 50 s; publish state at t = 60 s.
    // elapsed since RX = 10 s < 35 s → active → all sends go through.
    record_mqtt_activity(50);
    let p = || VehicleStatePayload { basic: vec![0x01], advanced: vec![] };
    embassy_futures::block_on(publish_single_state(p(), 60_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    assert!(
        MQTT_TX_CHANNEL.try_receive().is_ok(),
        "MQTT must not be throttled while a client is active"
    );

    // Still active — second send at t = 70 s (elapsed = 20 s < 35 s).
    embassy_futures::block_on(publish_single_state(p(), 70_000));
    let _ = BLE_TX_CHANNEL.try_receive().unwrap();
    assert!(
        MQTT_TX_CHANNEL.try_receive().is_ok(),
        "MQTT must continue sending while within activity window"
    );
}

#[test]
fn ble_never_throttled_regardless_of_mqtt_activity() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // No recent MQTT activity; publish many states — BLE always receives all of them.
    let p = || VehicleStatePayload { basic: vec![0x01], advanced: vec![] };
    for i in 0u64..5 {
        embassy_futures::block_on(publish_single_state(p(), 40_000 + i * 1_000));
        assert!(
            BLE_TX_CHANNEL.try_receive().is_ok(),
            "BLE must always receive state updates"
        );
        let _ = MQTT_TX_CHANNEL.try_receive(); // may or may not be present — don't care
    }
}
