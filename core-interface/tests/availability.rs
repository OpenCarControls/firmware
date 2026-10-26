use core_interface::{
    BLE_TX_CHANNEL, COMMAND_AVAILABILITY_REPEAT_MS, CommandKind, MQTT_TX_CHANNEL,
    REPORT_MAX_AGE_MS, approve_can_read_write, command_availability_at, confirm_car_fingerprint,
    confirm_car_identity, init, proto, publish_command_availability,
    publish_command_availability_mqtt, record_mqtt_activity, report_command_availability_at,
    resend_command_availability, reset_can_access_for_tests, reset_command_availability_for_tests,
    reset_mqtt_throttle_for_tests, revoke_car_identity, set_can_dev_write_unlocked,
    set_can_read_only,
};
use proto::CommandAvailabilityState as State;

const PLATFORM_ID: u32 = 0xDEAD_BEEF;

fn drain_tx() {
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
}

/// Boot state: read-only, nothing reported.
fn boot() {
    init(PLATFORM_ID);
    revoke_car_identity();
    reset_can_access_for_tests();
    reset_command_availability_for_tests();
    set_can_read_only(true);
    reset_mqtt_throttle_for_tests();
    drain_tx();
}

fn read_write() {
    boot();
    confirm_car_identity();
    assert!(confirm_car_fingerprint(b"KNAC381AAP5000001"));
    approve_can_read_write().unwrap();
    set_can_dev_write_unlocked(true).unwrap();
    assert!(!set_can_read_only(false));
}

fn sent_availability() -> Option<proto::CommandAvailability> {
    match BLE_TX_CHANNEL.try_receive().ok()?.payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => {
            assert!(update.vehicle_state.is_none());
            update.system_state?.command_availability
        }
        other => panic!("unexpected message {other:?}"),
    }
}

fn state(code: i32) -> State {
    State::try_from(code).unwrap()
}

#[test]
fn nothing_is_published_before_the_vehicle_reports() {
    read_write();
    assert_eq!(command_availability_at(0), None);
    assert!(!publish_command_availability(0));
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn a_report_is_published_as_reported_while_read_write() {
    read_write();
    report_command_availability_at(
        CommandKind::Basic,
        &[(1, State::Available), (2, State::Unavailable)],
        1_000,
    );
    report_command_availability_at(CommandKind::Advanced, &[(1, State::Available)], 1_000);
    assert!(publish_command_availability(1_000));
    let sent = sent_availability().expect("availability");
    assert_eq!(state(sent.basic[&1]), State::Available);
    assert_eq!(state(sent.basic[&2]), State::Unavailable);
    assert_eq!(state(sent.advanced[&1]), State::Available);
}

#[test]
fn everything_is_unavailable_while_read_only() {
    boot();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    let shown = command_availability_at(1_000).unwrap();
    assert_eq!(state(shown.basic[&1]), State::Unavailable);
}

#[test]
fn an_unsupported_command_stays_unsupported_while_read_only() {
    boot();
    report_command_availability_at(
        CommandKind::Basic,
        &[(1, State::Available), (2, State::Unsupported)],
        1_000,
    );
    let shown = command_availability_at(1_000).unwrap();
    assert_eq!(state(shown.basic[&1]), State::Unavailable);
    assert_eq!(state(shown.basic[&2]), State::Unsupported);
}

#[test]
fn a_report_that_is_too_old_is_not_trusted() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    let fresh = command_availability_at(1_000 + REPORT_MAX_AGE_MS).unwrap();
    assert_eq!(state(fresh.basic[&1]), State::Available);
    let old = command_availability_at(1_000 + REPORT_MAX_AGE_MS + 1).unwrap();
    assert_eq!(state(old.basic[&1]), State::Unavailable);
}

#[test]
fn a_change_is_published_at_once_and_an_unchanged_state_only_after_the_repeat() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    assert!(publish_command_availability(1_000));
    assert!(sent_availability().is_some());

    // Unchanged: nothing until the repeat interval has passed.
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 2_000);
    assert!(!publish_command_availability(2_000));
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
    report_command_availability_at(
        CommandKind::Basic,
        &[(1, State::Available)],
        1_000 + COMMAND_AVAILABILITY_REPEAT_MS,
    );
    assert!(publish_command_availability(
        1_000 + COMMAND_AVAILABILITY_REPEAT_MS
    ));
    drain_tx();

    // A change goes out at once.
    let now = 1_000 + COMMAND_AVAILABILITY_REPEAT_MS + 100;
    report_command_availability_at(CommandKind::Basic, &[(1, State::Unavailable)], now);
    assert!(publish_command_availability(now));
    let sent = sent_availability().unwrap();
    assert_eq!(state(sent.basic[&1]), State::Unavailable);
}

#[test]
fn going_read_only_is_published_without_a_new_report() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    assert!(publish_command_availability(1_000));
    drain_tx();

    set_can_read_only(true);
    assert!(publish_command_availability(1_100));
    let sent = sent_availability().unwrap();
    assert_eq!(state(sent.basic[&1]), State::Unavailable);
}

#[test]
fn resend_publishes_again_even_if_unchanged() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    assert!(publish_command_availability(1_000));
    drain_tx();
    assert!(!publish_command_availability(1_100));
    resend_command_availability();
    assert!(publish_command_availability(1_100));
    assert!(sent_availability().is_some());
}

#[test]
fn a_full_queue_is_retried_on_the_next_call() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    for _ in 0..4 {
        BLE_TX_CHANNEL
            .try_send(proto::DeviceToApp::default())
            .unwrap();
    }
    assert!(!publish_command_availability(1_000));
    drain_tx();
    assert!(publish_command_availability(1_100));
}

#[test]
fn a_new_report_replaces_the_previous_one_for_that_kind() {
    read_write();
    report_command_availability_at(
        CommandKind::Basic,
        &[(1, State::Available), (2, State::Available)],
        1_000,
    );
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_100);
    let shown = command_availability_at(1_100).unwrap();
    assert_eq!(shown.basic.len(), 1);
}

// ── MQTT ──────────────────────────────────────────────────────────────────────

fn sent_mqtt_availability() -> Option<proto::CommandAvailability> {
    match MQTT_TX_CHANNEL.try_receive().ok()?.payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => {
            assert!(update.vehicle_state.is_none());
            update.system_state?.command_availability
        }
        other => panic!("unexpected message {other:?}"),
    }
}

/// An MQTT client is active now: state goes out at the full rate.
fn mqtt_client_active(now_secs: u32) {
    record_mqtt_activity(now_secs);
}

#[test]
fn mqtt_is_told_nothing_before_the_vehicle_reports() {
    read_write();
    assert!(!publish_command_availability_mqtt(1_000));
    assert!(MQTT_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_gets_the_basic_commands_only_and_all_unavailable_while_mqtt_commands_are_rejected() {
    read_write();
    report_command_availability_at(
        CommandKind::Basic,
        &[(1, State::Available), (2, State::Unsupported)],
        1_000,
    );
    report_command_availability_at(CommandKind::Advanced, &[(1, State::Available)], 1_000);
    mqtt_client_active(1);
    assert!(publish_command_availability_mqtt(1_000));
    let sent = sent_mqtt_availability().expect("availability");
    // BLE would say available: the device rejects every command over MQTT for now.
    assert_eq!(state(sent.basic[&1]), State::Unavailable);
    assert_eq!(state(sent.basic[&2]), State::Unsupported);
    assert!(sent.advanced.is_empty(), "advanced commands are BLE only");
    // BLE is unaffected.
    assert!(publish_command_availability(1_000));
    let ble = sent_availability().unwrap();
    assert_eq!(state(ble.basic[&1]), State::Available);
    assert_eq!(state(ble.advanced[&1]), State::Available);
}

#[test]
fn mqtt_repeats_while_active_and_sends_a_change_at_once() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    mqtt_client_active(1);
    assert!(publish_command_availability_mqtt(1_000));
    drain_tx();

    // Unchanged: nothing until the repeat interval has passed.
    assert!(!publish_command_availability_mqtt(2_000));
    report_command_availability_at(
        CommandKind::Basic,
        &[(1, State::Available)],
        1_000 + COMMAND_AVAILABILITY_REPEAT_MS,
    );
    mqtt_client_active(6);
    assert!(publish_command_availability_mqtt(
        1_000 + COMMAND_AVAILABILITY_REPEAT_MS
    ));
    assert!(sent_mqtt_availability().is_some());
}

#[test]
fn mqtt_is_sent_at_the_idle_rate_when_no_client_is_active() {
    read_write();
    // No MQTT activity recorded: the first message goes out, then one per idle period.
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 100_000);
    assert!(publish_command_availability_mqtt(100_000));
    drain_tx();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 110_000);
    assert!(!publish_command_availability_mqtt(110_000));
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 130_000);
    assert!(publish_command_availability_mqtt(130_000));
}

#[test]
fn mqtt_resend_publishes_again() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    mqtt_client_active(1);
    assert!(publish_command_availability_mqtt(1_000));
    drain_tx();
    assert!(!publish_command_availability_mqtt(1_100));
    resend_command_availability();
    assert!(publish_command_availability_mqtt(1_100));
}

#[test]
fn a_full_mqtt_queue_never_blocks_and_is_retried() {
    read_write();
    report_command_availability_at(CommandKind::Basic, &[(1, State::Available)], 1_000);
    mqtt_client_active(1);
    for _ in 0..4 {
        MQTT_TX_CHANNEL
            .try_send(proto::DeviceToApp::default())
            .unwrap();
    }
    assert!(!publish_command_availability_mqtt(1_000));
    drain_tx();
    assert!(publish_command_availability_mqtt(1_100));
}
