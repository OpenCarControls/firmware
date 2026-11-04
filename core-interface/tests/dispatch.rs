use core_interface::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_TX_CHANNEL, MQTT_TX_CHANNEL,
    SYSTEM_COMMAND_CHANNEL, Transport, handle_ble_message, handle_mqtt_message, init,
    proto::{
        self, CanDebugFilter as ProtoCanDebugFilter, ClearPairedPhonesCommand,
        ListPairedPhonesCommand, RemovePairedPhoneCommand, RestartCommand, SetCanDebugEnabled,
        SystemCommand, UpdateCanDebugFilters, UpsertPairedPhoneCommand, app_to_device::Payload,
        system_command::Action,
    },
    reset_ble_controller_lease_for_tests, reset_recent_commands_for_tests, set_link_up,
};

const PLATFORM_ID: u32 = 0xDEAD_BEEF;
const WRONG_ID: u32 = 0x0000_0001;
const MSG_ID: u64 = 42;

/// Fresh dispatcher state: no commands seen yet (tests reuse `MSG_ID`), both links up.
fn setup() {
    init(PLATFORM_ID);
    reset_recent_commands_for_tests();
    embassy_futures::block_on(core_interface::clear_paired_phones());
    core_interface::reset_phone_trust_for_tests();
    set_link_up(Transport::Ble, true);
    set_link_up(Transport::Mqtt, true);
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
}

/// The phones the tests send vehicle commands from.
const VALIDATED_PHONES: [&[u8]; 3] = [&[1, 2, 3, 4], &[0xAA], &[0xBB]];

fn ble_msg(platform_id: u32, payload: Option<Payload>) -> proto::AppToDevice {
    proto::AppToDevice {
        platform_id,
        message_id: MSG_ID,
        source_device_id: vec![],
        payload,
    }
}

fn ble_msg_with_source(
    platform_id: u32,
    source_device_id: Vec<u8>,
    payload: Option<Payload>,
) -> proto::AppToDevice {
    proto::AppToDevice {
        platform_id,
        message_id: MSG_ID,
        source_device_id,
        payload,
    }
}

/// Meets every CAN read-write condition, so vehicle commands are routed: they are
/// refused while the device is read-only (tested in `can_access.rs`).
fn allow_read_write() {
    core_interface::confirm_car_identity();
    assert!(core_interface::confirm_car_fingerprint(b"TEST-CAR"));
    core_interface::approve_can_read_write().unwrap();
    core_interface::set_can_dev_write_unlocked(true).unwrap();
    core_interface::set_can_read_only(false);
    assert!(!core_interface::is_can_read_only());
    for phone in VALIDATED_PHONES {
        embassy_futures::block_on(core_interface::add_paired_phone(phone));
        assert!(core_interface::validate_phone_for_tests(phone));
    }
}

fn make_system_cmd() -> SystemCommand {
    SystemCommand {
        action: Some(proto::system_command::Action::RestartCommand(
            RestartCommand {},
        )),
    }
}

// ── Platform-ID filtering ─────────────────────────────────────────────────────

#[test]
fn ble_wrong_platform_id_drops_message() {
    setup();
    let bytes = vec![1u8, 2, 3];
    let msg = ble_msg(WRONG_ID, Some(Payload::BasicCommandBytes(bytes)));
    embassy_futures::block_on(handle_ble_message(msg));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_wrong_platform_id_drops_message() {
    setup();
    let bytes = vec![1u8, 2, 3];
    let msg = ble_msg(WRONG_ID, Some(Payload::BasicCommandBytes(bytes)));
    embassy_futures::block_on(handle_mqtt_message(msg));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

// ── BLE routing ───────────────────────────────────────────────────────────────

#[test]
fn ble_system_command_goes_to_system_channel() {
    setup();
    let cmd = make_system_cmd();
    let msg = ble_msg(PLATFORM_ID, Some(Payload::SystemCommand(cmd)));
    embassy_futures::block_on(handle_ble_message(msg));
    let received = SYSTEM_COMMAND_CHANNEL
        .try_receive()
        .expect("SystemCommand not forwarded");
    assert!(matches!(
        received.action,
        Some(proto::system_command::Action::RestartCommand(_))
    ));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
    assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_basic_command_bytes_goes_to_basic_channel_with_ble_transport() {
    setup();
    allow_read_write();
    embassy_futures::block_on(reset_ble_controller_lease_for_tests());
    let bytes = vec![0xAA, 0xBB];
    let source = vec![1, 2, 3, 4];
    let msg = ble_msg_with_source(
        PLATFORM_ID,
        source.clone(),
        Some(Payload::BasicCommandBytes(bytes.clone())),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    let cmd = BASIC_CMD_CHANNEL
        .try_receive()
        .expect("BasicCommand not forwarded");
    assert_eq!(cmd.message_id, MSG_ID);
    assert_eq!(cmd.bytes, bytes);
    assert_eq!(cmd.source_device_id, source);
    assert!(matches!(cmd.transport, Transport::Ble));
    assert_eq!(
        cmd.authorization.as_ref().map(|a| a.device_id()),
        Some(source.as_slice()),
        "a validated phone's command carries its authorization"
    );
    assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_advanced_command_bytes_goes_to_advanced_channel() {
    setup();
    allow_read_write();
    embassy_futures::block_on(reset_ble_controller_lease_for_tests());
    let bytes = vec![0xCC, 0xDD];
    let source = vec![1, 2, 3, 4];
    let msg = ble_msg_with_source(
        PLATFORM_ID,
        source.clone(),
        Some(Payload::AdvancedCommandBytes(bytes.clone())),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    let cmd = ADVANCED_CMD_CHANNEL
        .try_receive()
        .expect("AdvancedCommand not forwarded");
    assert_eq!(cmd.message_id, MSG_ID);
    assert_eq!(cmd.bytes, bytes);
    assert_eq!(cmd.source_device_id, source);
    assert!(matches!(cmd.transport, Transport::Ble));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_none_payload_drops_silently() {
    setup();
    let msg = ble_msg(PLATFORM_ID, None);
    embassy_futures::block_on(handle_ble_message(msg));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
    assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
    assert!(SYSTEM_COMMAND_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_basic_command_rejected_when_controller_lease_owned_by_another_phone() {
    setup();
    allow_read_write();
    embassy_futures::block_on(reset_ble_controller_lease_for_tests());

    let first = ble_msg_with_source(
        PLATFORM_ID,
        vec![0xAA],
        Some(Payload::BasicCommandBytes(vec![0x01])),
    );
    embassy_futures::block_on(handle_ble_message(first));
    let _ = BASIC_CMD_CHANNEL
        .try_receive()
        .expect("first controller command should be accepted");

    let second = ble_msg_with_source(
        PLATFORM_ID,
        vec![0xBB],
        Some(Payload::BasicCommandBytes(vec![0x02])),
    );
    embassy_futures::block_on(handle_ble_message(second));

    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
    let outbound = BLE_TX_CHANNEL
        .try_receive()
        .expect("expected non-controller rejection response on BLE_TX");
    let resp = match outbound.payload {
        Some(proto::device_to_app::Payload::CommandResponse(r)) => r,
        other => panic!("unexpected payload: {:?}", other),
    };
    assert!(!resp.success);
    assert_eq!(
        resp.status_code,
        proto::CommandStatusCode::RejectedNotController as i32
    );
}

#[test]
fn ble_basic_command_with_empty_source_device_id_is_rejected() {
    setup();
    allow_read_write();
    embassy_futures::block_on(reset_ble_controller_lease_for_tests());

    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::BasicCommandBytes(vec![0x10, 0x20])),
    );
    embassy_futures::block_on(handle_ble_message(msg));

    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
    let outbound = BLE_TX_CHANNEL
        .try_receive()
        .expect("expected a rejection response on BLE_TX");
    let resp = match outbound.payload {
        Some(proto::device_to_app::Payload::CommandResponse(r)) => r,
        other => panic!("unexpected payload: {:?}", other),
    };
    assert!(!resp.success);
    // No phone identity, so never a validated phone.
    assert_eq!(
        resp.status_code,
        proto::CommandStatusCode::RejectedNotValidated as i32
    );
}

// ── MQTT routing (restrictions) ───────────────────────────────────────────────

#[test]
fn mqtt_basic_command_is_rejected_until_mqtt_is_authenticated() {
    setup();
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
    let mut msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::BasicCommandBytes(vec![0x11, 0x22])),
    );
    msg.message_id = 77;
    embassy_futures::block_on(handle_mqtt_message(msg));
    assert!(
        BASIC_CMD_CHANNEL.try_receive().is_err(),
        "MQTT command must not reach the vehicle"
    );
    let reply = MQTT_TX_CHANNEL
        .try_receive()
        .expect("rejected MQTT command not answered");
    match reply.payload {
        Some(proto::device_to_app::Payload::CommandResponse(resp)) => {
            assert_eq!(resp.message_id, 77);
            assert!(!resp.success);
            assert_eq!(resp.status_code, proto::CommandStatusCode::Failed as i32);
            assert!(resp.error_message.contains("MQTT"));
        }
        other => panic!("expected CommandResponse, got {other:?}"),
    }
}

#[test]
fn mqtt_system_command_silently_dropped() {
    setup();
    let msg = ble_msg(PLATFORM_ID, Some(Payload::SystemCommand(make_system_cmd())));
    embassy_futures::block_on(handle_mqtt_message(msg));
    assert!(SYSTEM_COMMAND_CHANNEL.try_receive().is_err());
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_advanced_command_bytes_silently_dropped() {
    setup();
    let bytes = vec![0xFF];
    let msg = ble_msg(PLATFORM_ID, Some(Payload::AdvancedCommandBytes(bytes)));
    embassy_futures::block_on(handle_mqtt_message(msg));
    assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

// ── Deprecated CAN debug commands ─────────────────────────────────────────────

/// The BLE CAN debug stream was removed (contracts still carry the messages, deprecated).
/// A bonded phone sending them must not panic or reach any channel, including with a
/// `bus_id` that used to overflow the old bus bitmask.
#[test]
#[allow(deprecated)]
fn ble_deprecated_can_debug_commands_are_ignored() {
    setup();
    while SYSTEM_COMMAND_CHANNEL.try_receive().is_ok() {}

    for bus_ids in [vec![], vec![0], vec![8], vec![255, 1_000_000]] {
        let msg = ble_msg(
            PLATFORM_ID,
            Some(Payload::SystemCommand(SystemCommand {
                action: Some(Action::SetCanDebugEnabled(SetCanDebugEnabled {
                    enabled: true,
                    bus_ids,
                })),
            })),
        );
        embassy_futures::block_on(handle_ble_message(msg));
    }
    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::UpdateCanDebugFilters(UpdateCanDebugFilters {
                filters: vec![ProtoCanDebugFilter {
                    can_id: 0x100,
                    is_extended_id: false,
                    mask: 0x7FF,
                }],
            })),
        })),
    );
    embassy_futures::block_on(handle_ble_message(msg));

    assert!(SYSTEM_COMMAND_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_restart_command_still_forwarded_to_system_channel_when_debug_available() {
    setup();
    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::RestartCommand(RestartCommand {})),
        })),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    let received = SYSTEM_COMMAND_CHANNEL
        .try_receive()
        .expect("RestartCommand not in SYSTEM_COMMAND_CHANNEL");
    assert!(matches!(received.action, Some(Action::RestartCommand(_))));
}

#[test]
fn ble_list_paired_phones_is_answered_with_the_trust_of_each_phone() {
    setup();
    allow_read_write();
    let bonded_only = vec![0xCC];
    embassy_futures::block_on(core_interface::add_paired_phone(&bonded_only));
    let msg = ble_msg_with_source(
        PLATFORM_ID,
        bonded_only.clone(),
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::ListPairedPhones(ListPairedPhonesCommand {})),
        })),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    assert!(
        SYSTEM_COMMAND_CHANNEL.try_receive().is_err(),
        "answered by core-interface, not forwarded"
    );
    let resp = match BLE_TX_CHANNEL.try_receive().unwrap().payload {
        Some(proto::device_to_app::Payload::CommandResponse(r)) => r,
        other => panic!("unexpected payload: {:?}", other),
    };
    assert!(resp.success);
    let phones = match resp.response_data {
        Some(proto::command_response::ResponseData::SystemResult(proto::SystemCommandResult {
            result: Some(proto::system_command_result::Result::PairedPhones(phones)),
        })) => phones.phones,
        other => panic!("unexpected response data: {:?}", other),
    };
    assert_eq!(phones.len(), 4);
    for phone in &phones {
        let expected = if phone.device_id == bonded_only {
            proto::PhoneTrustLevel::Bonded
        } else {
            proto::PhoneTrustLevel::Validated
        };
        assert_eq!(phone.trust, expected as i32);
        assert_eq!(phone.is_requester, phone.device_id == bonded_only);
    }
}

#[test]
fn ble_remove_paired_phone_forwarded_to_system_channel() {
    setup();
    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::RemovePairedPhone(RemovePairedPhoneCommand {
                device_id: vec![1, 2, 3, 4],
            })),
        })),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    let received = SYSTEM_COMMAND_CHANNEL
        .try_receive()
        .expect("RemovePairedPhone not in SYSTEM_COMMAND_CHANNEL");
    assert!(matches!(
        received.action,
        Some(Action::RemovePairedPhone(_))
    ));
}

#[test]
fn ble_clear_paired_phones_forwarded_to_system_channel() {
    setup();
    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::ClearPairedPhones(ClearPairedPhonesCommand {})),
        })),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    let received = SYSTEM_COMMAND_CHANNEL
        .try_receive()
        .expect("ClearPairedPhones not in SYSTEM_COMMAND_CHANNEL");
    assert!(matches!(
        received.action,
        Some(Action::ClearPairedPhones(_))
    ));
}

#[test]
fn ble_upsert_paired_phone_forwarded_to_system_channel() {
    setup();
    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::UpsertPairedPhone(UpsertPairedPhoneCommand {
                device_id: vec![9, 8, 7, 6],
            })),
        })),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    let received = SYSTEM_COMMAND_CHANNEL
        .try_receive()
        .expect("UpsertPairedPhone not in SYSTEM_COMMAND_CHANNEL");
    assert!(matches!(
        received.action,
        Some(Action::UpsertPairedPhone(_))
    ));
}

// ── Duplicate commands ────────────────────────────────────────────────────────

#[test]
fn ble_repeated_command_runs_once() {
    setup();
    allow_read_write();
    embassy_futures::block_on(reset_ble_controller_lease_for_tests());
    let source = vec![1, 2, 3, 4];
    for _ in 0..2 {
        let msg = ble_msg_with_source(
            PLATFORM_ID,
            source.clone(),
            Some(Payload::BasicCommandBytes(vec![0xAA])),
        );
        embassy_futures::block_on(handle_ble_message(msg));
    }
    assert!(BASIC_CMD_CHANNEL.try_receive().is_ok());
    assert!(
        BASIC_CMD_CHANNEL.try_receive().is_err(),
        "the repeat must be dropped"
    );
}

#[test]
fn ble_repeated_system_command_runs_once() {
    setup();
    for _ in 0..2 {
        let msg = ble_msg(PLATFORM_ID, Some(Payload::SystemCommand(make_system_cmd())));
        embassy_futures::block_on(handle_ble_message(msg));
    }
    assert!(SYSTEM_COMMAND_CHANNEL.try_receive().is_ok());
    assert!(SYSTEM_COMMAND_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_redelivered_command_is_answered_once() {
    setup();
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
    for _ in 0..2 {
        let msg = ble_msg(PLATFORM_ID, Some(Payload::BasicCommandBytes(vec![1])));
        embassy_futures::block_on(handle_mqtt_message(msg));
    }
    assert!(MQTT_TX_CHANNEL.try_receive().is_ok());
    assert!(
        MQTT_TX_CHANNEL.try_receive().is_err(),
        "a QoS 1 redelivery must not be handled again"
    );
}
