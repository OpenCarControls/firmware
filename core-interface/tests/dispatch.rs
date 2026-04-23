use core_interface::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_TX_CHANNEL, SYSTEM_COMMAND_CHANNEL, Transport,
    can_debug_wants_bus, debug_dropped_count, debug_filter_count, handle_ble_message,
    handle_mqtt_message, increment_can_debug_dropped, init, is_can_debug_active,
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
fn mqtt_basic_command_bytes_goes_to_basic_channel_with_mqtt_transport() {
    init(PLATFORM_ID);
    let bytes = vec![0x11, 0x22];
    let msg = ble_msg(PLATFORM_ID, Some(Payload::BasicCommandBytes(bytes.clone())));
    embassy_futures::block_on(handle_mqtt_message(msg));
    let cmd = BASIC_CMD_CHANNEL
        .try_receive()
        .expect("BasicCommand not forwarded from MQTT");
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

// ── CAN debug command dispatch ────────────────────────────────────────────────

fn set_debug_msg(enabled: bool, bus_ids: Vec<u32>) -> proto::AppToDevice {
    ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::SetCanDebugEnabled(SetCanDebugEnabled {
                enabled,
                bus_ids,
            })),
        })),
    )
}

fn update_filters_msg(filters: Vec<ProtoCanDebugFilter>) -> proto::AppToDevice {
    ble_msg(
        PLATFORM_ID,
        Some(Payload::SystemCommand(SystemCommand {
            action: Some(Action::UpdateCanDebugFilters(UpdateCanDebugFilters {
                filters,
            })),
        })),
    )
}

#[test]
fn ble_set_can_debug_enabled_activates_debug_mode() {
    init(PLATFORM_ID);
    // Ensure disabled first.
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
    assert!(!is_can_debug_active());

    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![0])));
    assert!(is_can_debug_active());
    assert!(can_debug_wants_bus(0));
    assert!(!can_debug_wants_bus(1));
    assert!(!can_debug_wants_bus(2));

    // Cleanup
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
}

#[test]
fn ble_set_can_debug_disabled_deactivates() {
    init(PLATFORM_ID);
    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![0])));
    assert!(is_can_debug_active());

    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
    assert!(!is_can_debug_active());
}

#[test]
fn ble_set_can_debug_empty_bus_ids_watches_all_buses() {
    init(PLATFORM_ID);
    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![])));
    assert!(is_can_debug_active());
    for bus in 0u8..=7 {
        assert!(
            can_debug_wants_bus(bus),
            "expected bus {} to be watched",
            bus
        );
    }

    // Cleanup
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
}

#[test]
fn ble_update_debug_filters_ignored_when_inactive() {
    init(PLATFORM_ID);
    // Ensure debug is off.
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
    assert!(!is_can_debug_active());

    // Should not panic; state should not change.
    let filter = ProtoCanDebugFilter {
        can_id: 0x100,
        is_extended_id: false,
        mask: 0x7FF,
    };
    embassy_futures::block_on(handle_ble_message(update_filters_msg(vec![filter])));
    // Can't inspect filters directly; only assert no panic and debug is still off.
    assert!(!is_can_debug_active());
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

#[test]
fn ble_set_can_debug_multiple_specific_buses() {
    init(PLATFORM_ID);
    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![1, 3])));
    assert!(is_can_debug_active());
    assert!(!can_debug_wants_bus(0), "bus 0 should not be watched");
    assert!(can_debug_wants_bus(1), "bus 1 should be watched");
    assert!(!can_debug_wants_bus(2), "bus 2 should not be watched");
    assert!(can_debug_wants_bus(3), "bus 3 should be watched");
    assert!(!can_debug_wants_bus(4), "bus 4 should not be watched");

    // Cleanup
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
}

#[test]
fn ble_update_debug_filters_stored_when_active() {
    init(PLATFORM_ID);
    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![0])));

    let filters = vec![
        ProtoCanDebugFilter {
            can_id: 0x100,
            is_extended_id: false,
            mask: 0x7FF,
        },
        ProtoCanDebugFilter {
            can_id: 0x200,
            is_extended_id: false,
            mask: 0x7FF,
        },
    ];
    embassy_futures::block_on(handle_ble_message(update_filters_msg(filters)));
    let count = embassy_futures::block_on(debug_filter_count());
    assert_eq!(count, 2, "filter list should contain 2 entries");

    // Cleanup
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
}

#[test]
fn ble_re_enable_resets_filter_list() {
    init(PLATFORM_ID);
    // Enable and install 2 filters.
    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![0])));
    let filters = vec![
        ProtoCanDebugFilter {
            can_id: 0x100,
            is_extended_id: false,
            mask: 0x7FF,
        },
        ProtoCanDebugFilter {
            can_id: 0x200,
            is_extended_id: false,
            mask: 0x7FF,
        },
    ];
    embassy_futures::block_on(handle_ble_message(update_filters_msg(filters)));
    assert_eq!(embassy_futures::block_on(debug_filter_count()), 2);

    // Disable then re-enable — filters must be cleared.
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![0])));
    assert_eq!(
        embassy_futures::block_on(debug_filter_count()),
        0,
        "re-enable must clear filters"
    );

    // Cleanup
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
}

#[test]
fn ble_re_enable_resets_dropped_counter() {
    init(PLATFORM_ID);
    // Accumulate some drops.
    increment_can_debug_dropped();
    increment_can_debug_dropped();
    increment_can_debug_dropped();
    assert_eq!(debug_dropped_count(), 3);

    // Enabling debug must reset the counter to 0.
    embassy_futures::block_on(handle_ble_message(set_debug_msg(true, vec![0])));
    assert_eq!(
        debug_dropped_count(),
        0,
        "enable must reset dropped counter"
    );

    // Cleanup
    embassy_futures::block_on(handle_ble_message(set_debug_msg(false, vec![])));
}
