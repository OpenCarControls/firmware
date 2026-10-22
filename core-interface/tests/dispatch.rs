use core_interface::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_TX_CHANNEL, MQTT_TX_CHANNEL,
    SYSTEM_COMMAND_CHANNEL, Transport, handle_ble_message, handle_mqtt_message, init,
    proto::{
        self, CanDebugFilter as ProtoCanDebugFilter, ClearPairedPhonesCommand,
        ListPairedPhonesCommand, RemovePairedPhoneCommand, RestartCommand, SetCanDebugEnabled,
        SystemCommand, UpdateCanDebugFilters, UpsertPairedPhoneCommand, app_to_device::Payload,
        system_command::Action,
    },
    reset_ble_controller_lease_for_tests,
};

const PLATFORM_ID: u32 = 0xDEAD_BEEF;
const WRONG_ID: u32 = 0x0000_0001;
const MSG_ID: u64 = 42;

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
    init(PLATFORM_ID);
    let bytes = vec![1u8, 2, 3];
    let msg = ble_msg(WRONG_ID, Some(Payload::BasicCommandBytes(bytes)));
    embassy_futures::block_on(handle_ble_message(msg));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_wrong_platform_id_drops_message() {
    init(PLATFORM_ID);
    let bytes = vec![1u8, 2, 3];
    let msg = ble_msg(WRONG_ID, Some(Payload::BasicCommandBytes(bytes)));
    embassy_futures::block_on(handle_mqtt_message(msg));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

// ── BLE routing ───────────────────────────────────────────────────────────────

#[test]
fn ble_system_command_goes_to_system_channel() {
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
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
    assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_advanced_command_bytes_goes_to_advanced_channel() {
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
    let msg = ble_msg(PLATFORM_ID, None);
    embassy_futures::block_on(handle_ble_message(msg));
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
    assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
    assert!(SYSTEM_COMMAND_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_basic_command_rejected_when_controller_lease_owned_by_another_phone() {
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
    embassy_futures::block_on(reset_ble_controller_lease_for_tests());

    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::BasicCommandBytes(vec![0x10, 0x20])),
    );
    embassy_futures::block_on(handle_ble_message(msg));

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

// ── MQTT routing (restrictions) ───────────────────────────────────────────────

#[test]
fn mqtt_basic_command_is_rejected_until_mqtt_is_authenticated() {
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
    let msg = ble_msg(PLATFORM_ID, Some(Payload::SystemCommand(make_system_cmd())));
    embassy_futures::block_on(handle_mqtt_message(msg));
    assert!(SYSTEM_COMMAND_CHANNEL.try_receive().is_err());
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn mqtt_advanced_command_bytes_silently_dropped() {
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
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
fn ble_list_paired_phones_forwarded_to_system_channel() {
    init(PLATFORM_ID);
    let msg = ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::ListPairedPhones(ListPairedPhonesCommand {})),
        })),
    );
    embassy_futures::block_on(handle_ble_message(msg));
    let received = SYSTEM_COMMAND_CHANNEL
        .try_receive()
        .expect("ListPairedPhones not in SYSTEM_COMMAND_CHANNEL");
    assert!(matches!(received.action, Some(Action::ListPairedPhones(_))));
}

#[test]
fn ble_remove_paired_phone_forwarded_to_system_channel() {
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
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
    init(PLATFORM_ID);
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
