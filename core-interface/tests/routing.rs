use core_interface::{
    BLE_STATE_TX, BLE_TX_CHANNEL, MQTT_STATE_TX, MQTT_TX_CHANNEL, Transport, VehicleStatePayload,
    init, latest_ble_state, proto, publish_single_state, record_mqtt_activity,
    reset_latest_ble_state_for_tests, reset_mqtt_throttle_for_tests, route_single_response,
    set_link_up,
};

const PLATFORM_ID: u32 = 0xCAFE_BABE;
const TS: u64 = 12345;

/// Drain both outbound channels and state slots, bring both links up and reset
/// throttle state so earlier tests don't pollute later ones when running under
/// `cargo test` (single process, shared statics).
fn drain_tx_channels() {
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
    BLE_STATE_TX.reset();
    MQTT_STATE_TX.reset();
    set_link_up(Transport::Ble, true);
    set_link_up(Transport::Mqtt, true);
    reset_mqtt_throttle_for_tests();
}

fn response_id(msg: proto::DeviceToApp) -> u64 {
    match msg.payload {
        Some(proto::device_to_app::Payload::CommandResponse(r)) => r.message_id,
        other => panic!("not a command response: {:?}", other),
    }
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

#[test]
fn responses_are_dropped_while_the_link_is_down() {
    drain_tx_channels();
    init(PLATFORM_ID);
    set_link_up(Transport::Ble, false);
    set_link_up(Transport::Mqtt, false);
    embassy_futures::block_on(route_single_response(Transport::Ble, make_response(5), TS));
    embassy_futures::block_on(route_single_response(Transport::Mqtt, make_response(6), TS));
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
    assert!(MQTT_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_responses_are_not_stuck_behind_ble_with_no_phone_connected() {
    drain_tx_channels();
    init(PLATFORM_ID);
    set_link_up(Transport::Ble, false);
    // Many BLE responses with nobody to take them: each returns at once.
    for id in 10..20 {
        embassy_futures::block_on(route_single_response(Transport::Ble, make_response(id), TS));
    }
    embassy_futures::block_on(route_single_response(
        Transport::Mqtt,
        make_response(20),
        TS,
    ));
    assert_eq!(response_id(MQTT_TX_CHANNEL.try_receive().unwrap()), 20);
}

#[test]
fn link_down_clears_what_was_queued_for_the_peer() {
    drain_tx_channels();
    init(PLATFORM_ID);
    embassy_futures::block_on(route_single_response(Transport::Ble, make_response(30), TS));
    set_link_up(Transport::Ble, false);
    set_link_up(Transport::Ble, true);
    assert!(
        BLE_TX_CHANNEL.try_receive().is_err(),
        "the next phone must not get the previous phone's response"
    );
}

#[test]
fn a_response_waiting_for_room_is_dropped_when_the_link_goes_down() {
    drain_tx_channels();
    init(PLATFORM_ID);
    for id in 40..44 {
        embassy_futures::block_on(route_single_response(Transport::Ble, make_response(id), TS));
    }
    // The queue is full: the next response waits until the phone leaves, and a new
    // phone connecting right after must not get it either.
    embassy_futures::block_on(embassy_futures::join::join(
        route_single_response(Transport::Ble, make_response(44), TS),
        async {
            embassy_futures::yield_now().await;
            set_link_up(Transport::Ble, false);
            set_link_up(Transport::Ble, true);
        },
    ));
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn a_response_waiting_for_room_is_sent_once_there_is_room() {
    drain_tx_channels();
    init(PLATFORM_ID);
    for id in 50..54 {
        embassy_futures::block_on(route_single_response(Transport::Ble, make_response(id), TS));
    }
    embassy_futures::block_on(embassy_futures::join::join(
        route_single_response(Transport::Ble, make_response(54), TS),
        async {
            embassy_futures::yield_now().await;
            assert_eq!(response_id(BLE_TX_CHANNEL.try_receive().unwrap()), 50);
        },
    ));
    let ids: Vec<u64> = core::iter::from_fn(|| BLE_TX_CHANNEL.try_receive().ok())
        .map(response_id)
        .collect();
    assert_eq!(ids, vec![51, 52, 53, 54]);
}

#[test]
fn state_updates_never_push_out_responses() {
    drain_tx_channels();
    init(PLATFORM_ID);
    record_mqtt_activity((TS / 1000) as u32);
    for id in 60..64 {
        embassy_futures::block_on(route_single_response(Transport::Ble, make_response(id), TS));
        embassy_futures::block_on(route_single_response(
            Transport::Mqtt,
            make_response(id),
            TS,
        ));
    }
    for basic in 0..10u8 {
        let payload = VehicleStatePayload {
            basic: vec![basic],
            advanced: vec![],
        };
        embassy_futures::block_on(publish_single_state(payload, TS));
    }
    for channel in [&BLE_TX_CHANNEL, &MQTT_TX_CHANNEL] {
        let ids: Vec<u64> = core::iter::from_fn(|| channel.try_receive().ok())
            .map(response_id)
            .collect();
        assert_eq!(ids, vec![60, 61, 62, 63]);
    }
    // Only the newest state waits in each slot.
    for slot in [&BLE_STATE_TX, &MQTT_STATE_TX] {
        match slot.try_take().expect("no state").payload {
            Some(proto::device_to_app::Payload::StateUpdate(update)) => {
                assert_eq!(update.vehicle_state.unwrap().basic_state_bytes, vec![9]);
            }
            other => panic!("unexpected payload: {:?}", other),
        }
        assert!(slot.try_take().is_none());
    }
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

    let msg = BLE_STATE_TX
        .try_take()
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

    let msg = MQTT_STATE_TX
        .try_take()
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
    let ble_msg = BLE_STATE_TX.try_take().unwrap();
    let mqtt_msg = MQTT_STATE_TX.try_take().unwrap();
    assert_eq!(ble_msg.timestamp_ms, 9999);
    assert_eq!(mqtt_msg.timestamp_ms, 9999);
}

// ── latest_ble_state ──────────────────────────────────────────────────────────

#[test]
fn latest_ble_state_is_none_before_any_publish() {
    drain_tx_channels();
    reset_latest_ble_state_for_tests();
    assert!(latest_ble_state().is_none());
}

#[test]
fn latest_ble_state_keeps_the_newest_full_state() {
    drain_tx_channels();
    reset_latest_ble_state_for_tests();
    init(PLATFORM_ID);
    for (basic, ts) in [(0x01, 1000), (0x02, 2000)] {
        let payload = VehicleStatePayload {
            basic: vec![basic],
            advanced: vec![0xBB],
        };
        embassy_futures::block_on(publish_single_state(payload, ts));
    }
    drain_tx_channels();

    // Still available after the slot was taken (as the BLE task does
    // while nobody has subscribed).
    let msg = latest_ble_state().expect("no cached BLE state");
    assert_eq!(msg.timestamp_ms, 2000);
    assert_eq!(msg.platform_id, PLATFORM_ID);
    match msg.payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => {
            let vs = update.vehicle_state.expect("no vehicle_state");
            assert_eq!(vs.basic_state_bytes, vec![0x02]);
            assert_eq!(vs.advanced_state_bytes, vec![0xBB]);
        }
        other => panic!("unexpected payload: {:?}", other),
    }
}

#[test]
fn command_responses_do_not_replace_latest_ble_state() {
    drain_tx_channels();
    reset_latest_ble_state_for_tests();
    init(PLATFORM_ID);
    let payload = VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(payload, TS));
    embassy_futures::block_on(route_single_response(Transport::Ble, make_response(4), TS));
    drain_tx_channels();
    assert!(matches!(
        latest_ble_state().and_then(|m| m.payload),
        Some(proto::device_to_app::Payload::StateUpdate(_))
    ));
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
    let _ = BLE_STATE_TX.try_take().unwrap();
    MQTT_STATE_TX.try_take().unwrap(); // first idle send allowed

    // Now publish at t = 60 s → elapsed since last state = 20 s < 30 s → suppressed.
    let payload2 = VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(payload2, 60_000));
    let _ = BLE_STATE_TX.try_take().unwrap();
    assert!(
        MQTT_STATE_TX.try_take().is_none(),
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
    let _ = BLE_STATE_TX.try_take().unwrap();
    assert!(
        MQTT_STATE_TX.try_take().is_some(),
        "MQTT must be allowed in idle after 30 s"
    );
}

#[test]
fn mqtt_idle_rate_resets_after_send() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // First idle send at t = 100 s — sets LAST_MQTT_STATE_SECS = 100.
    let p = || VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(p(), 100_000));
    let _ = BLE_STATE_TX.try_take().unwrap();
    MQTT_STATE_TX.try_take().unwrap(); // first send allowed

    // Second send at t = 129 s → elapsed since last state = 29 s < 30 s → suppressed.
    embassy_futures::block_on(publish_single_state(p(), 129_000));
    let _ = BLE_STATE_TX.try_take().unwrap();
    assert!(
        MQTT_STATE_TX.try_take().is_none(),
        "MQTT must be suppressed within 30 s of last idle send"
    );

    // Third send at t = 131 s → elapsed = 31 s >= 30 s → allowed.
    embassy_futures::block_on(publish_single_state(p(), 131_000));
    let _ = BLE_STATE_TX.try_take().unwrap();
    assert!(
        MQTT_STATE_TX.try_take().is_some(),
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
    let p = || VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    embassy_futures::block_on(publish_single_state(p(), 60_000));
    let _ = BLE_STATE_TX.try_take().unwrap();
    assert!(
        MQTT_STATE_TX.try_take().is_some(),
        "MQTT must not be throttled while a client is active"
    );

    // Still active — second send at t = 70 s (elapsed = 20 s < 35 s).
    embassy_futures::block_on(publish_single_state(p(), 70_000));
    let _ = BLE_STATE_TX.try_take().unwrap();
    assert!(
        MQTT_STATE_TX.try_take().is_some(),
        "MQTT must continue sending while within activity window"
    );
}

#[test]
fn ble_never_throttled_regardless_of_mqtt_activity() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // No recent MQTT activity; publish many states — BLE always receives all of them.
    let p = || VehicleStatePayload {
        basic: vec![0x01],
        advanced: vec![],
    };
    for i in 0u64..5 {
        embassy_futures::block_on(publish_single_state(p(), 40_000 + i * 1_000));
        assert!(
            BLE_STATE_TX.try_take().is_some(),
            "BLE must always receive state updates"
        );
        let _ = MQTT_STATE_TX.try_take(); // may or may not be present — don't care
    }
}
