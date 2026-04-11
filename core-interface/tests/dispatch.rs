use core_interface::{
    handle_ble_message, handle_mqtt_message, init,
    proto::{
        self,
        app_to_device::Payload,
        RestartCommand, SystemCommand,
    },
    Transport, ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, SYSTEM_COMMAND_CHANNEL,
};

const PLATFORM_ID: u32 = 0xDEAD_BEEF;
const WRONG_ID: u32 = 0x0000_0001;
const MSG_ID: u64 = 42;

fn ble_msg(platform_id: u32, payload: Option<Payload>) -> proto::AppToDevice {
    proto::AppToDevice { platform_id, message_id: MSG_ID, payload }
}

fn make_system_cmd() -> SystemCommand {
    SystemCommand {
        action: Some(proto::system_command::Action::RestartCommand(RestartCommand {})),
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
    let received = SYSTEM_COMMAND_CHANNEL.try_receive().expect("SystemCommand not forwarded");
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
    let bytes = vec![0xAA, 0xBB];
    let msg = ble_msg(PLATFORM_ID, Some(Payload::BasicCommandBytes(bytes.clone())));
    embassy_futures::block_on(handle_ble_message(msg));
    let cmd = BASIC_CMD_CHANNEL.try_receive().expect("BasicCommand not forwarded");
    assert_eq!(cmd.message_id, MSG_ID);
    assert_eq!(cmd.bytes, bytes);
    assert!(matches!(cmd.transport, Transport::Ble));
    assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
}

#[test]
fn ble_advanced_command_bytes_goes_to_advanced_channel() {
    init(PLATFORM_ID);
    let bytes = vec![0xCC, 0xDD];
    let msg = ble_msg(PLATFORM_ID, Some(Payload::AdvancedCommandBytes(bytes.clone())));
    embassy_futures::block_on(handle_ble_message(msg));
    let cmd = ADVANCED_CMD_CHANNEL.try_receive().expect("AdvancedCommand not forwarded");
    assert_eq!(cmd.message_id, MSG_ID);
    assert_eq!(cmd.bytes, bytes);
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

// ── MQTT routing (restrictions) ───────────────────────────────────────────────

#[test]
fn mqtt_basic_command_bytes_goes_to_basic_channel_with_mqtt_transport() {
    init(PLATFORM_ID);
    let bytes = vec![0x11, 0x22];
    let msg = ble_msg(PLATFORM_ID, Some(Payload::BasicCommandBytes(bytes.clone())));
    embassy_futures::block_on(handle_mqtt_message(msg));
    let cmd = BASIC_CMD_CHANNEL.try_receive().expect("BasicCommand not forwarded from MQTT");
    assert_eq!(cmd.bytes, bytes);
    assert!(matches!(cmd.transport, Transport::Mqtt));
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
