//! The CAN read-write policy (docs/can.md, "Read-write policy"): the three conditions,
//! the approval's persistence and binding to one car, the relock conditions, and that
//! only a bonded phone over BLE can approve or unlock. Release builds (no dev lock) are
//! in `can_access_production.rs`, a separate process, since that is set once per boot.

use core_interface::{
    BLE_TX_CHANNEL, CanLogSession, MQTT_TX_CHANNEL, add_paired_phone, approve_can_read_write,
    can_access_state, can_read_write_approval, clear_paired_phones, confirm_car_fingerprint,
    confirm_car_identity, handle_ble_message, handle_mqtt_message, has_can_dev_write_lock, init,
    is_can_bus_read_only, is_can_dev_write_unlocked, is_can_read_only,
    is_car_fingerprint_confirmed, proto, publish_can_access_state, remove_paired_phone,
    reset_can_access_for_tests, restore_can_read_write_approval, revoke_can_read_write_approval,
    revoke_car_identity, set_can_dev_write_unlocked, set_can_read_only,
    try_take_can_read_write_approval_to_store,
};
use proto::CanReadWriteApproval as Approval;
use proto::app_to_device::Payload;
use proto::system_command::Action;

const PLATFORM_ID: u32 = 0xDEAD_BEEF;
const CAR_A: &[u8] = b"KNAC381AAP5000001";
const CAR_B: &[u8] = b"KNAC381AAP5000002";
const PHONE: &[u8] = b"ble:\x01\x02\x03\x04\x05\x06";
const STRANGER: &[u8] = b"ble:\x0A\x0B\x0C\x0D\x0E\x0F";

fn drain_tx() {
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
}

/// Boot state: nothing identified, approved or unlocked, no phones, bus locked.
fn boot() {
    init(PLATFORM_ID);
    revoke_car_identity();
    reset_can_access_for_tests();
    embassy_futures::block_on(clear_paired_phones());
    set_can_read_only(true);
    drain_tx();
}

fn identify(car: &[u8]) {
    confirm_car_identity();
    assert!(confirm_car_fingerprint(car));
}

/// Every condition met for `car`, and the vehicle has asked to unlock.
fn read_write_for(car: &[u8]) {
    boot();
    identify(car);
    approve_can_read_write().unwrap();
    set_can_dev_write_unlocked(true).unwrap();
    assert!(!set_can_read_only(false));
    assert!(!is_can_read_only());
}

fn system_command(source: &[u8], action: Action) -> proto::AppToDevice {
    proto::AppToDevice {
        message_id: 7,
        platform_id: PLATFORM_ID,
        source_device_id: source.to_vec(),
        payload: Some(Payload::SystemCommand(proto::SystemCommand {
            action: Some(action),
        })),
    }
}

fn approve_cmd(source: &[u8], approved: bool) -> proto::AppToDevice {
    system_command(
        source,
        Action::SetCanReadWriteApproval(proto::SetCanReadWriteApprovalCommand { approved }),
    )
}

fn unlock_cmd(source: &[u8], unlocked: bool) -> proto::AppToDevice {
    system_command(
        source,
        Action::SetCanDevWriteUnlock(proto::SetCanDevWriteUnlockCommand { unlocked }),
    )
}

/// The `CommandResponse` the BLE dispatcher sent for the last command.
fn ble_response() -> proto::CommandResponse {
    loop {
        let msg = BLE_TX_CHANNEL.try_receive().expect("no BLE response");
        if let Some(proto::device_to_app::Payload::CommandResponse(resp)) = msg.payload {
            return resp;
        }
    }
}

// ── The three conditions ──────────────────────────────────────────────────────

#[test]
fn development_builds_have_the_dev_lock_locked_at_boot() {
    boot();
    assert!(has_can_dev_write_lock());
    assert!(!is_can_dev_write_unlocked());
}

#[test]
fn read_write_needs_the_car_identified() {
    read_write_for(CAR_A);
    revoke_car_identity();
    assert!(is_can_read_only());
    assert!(is_can_bus_read_only(0));
}

#[test]
fn read_write_needs_the_exact_car_not_only_the_platform() {
    boot();
    confirm_car_identity();
    assert!(!is_car_fingerprint_confirmed());
    assert_eq!(
        approve_can_read_write(),
        Err(core_interface::CanAccessError::CarNotIdentified)
    );
    set_can_dev_write_unlocked(true).unwrap();
    assert!(set_can_read_only(false), "unlock must be refused");
    assert!(is_can_read_only());
}

#[test]
fn a_fingerprint_needs_the_platform_first_and_some_bytes() {
    boot();
    assert!(
        !confirm_car_fingerprint(CAR_A),
        "refused before the platform matched"
    );
    assert!(!is_car_fingerprint_confirmed());
    confirm_car_identity();
    assert!(
        !confirm_car_fingerprint(b""),
        "an empty fingerprint identifies nothing"
    );
    assert!(confirm_car_fingerprint(CAR_A));
    assert!(is_car_fingerprint_confirmed());
}

#[test]
fn read_write_needs_the_owners_approval() {
    boot();
    identify(CAR_A);
    set_can_dev_write_unlocked(true).unwrap();
    // The vehicle's unlock is accepted (the car is identified) but waits for approval.
    assert!(set_can_read_only(false));
    assert!(is_can_read_only());
    assert_eq!(can_read_write_approval(), Approval::NotApproved);

    read_write_for(CAR_A);
    revoke_can_read_write_approval();
    assert!(is_can_read_only());
    assert_eq!(can_read_write_approval(), Approval::NotApproved);
}

#[test]
fn read_write_needs_the_dev_lock_unlocked_in_development_builds() {
    boot();
    identify(CAR_A);
    approve_can_read_write().unwrap();
    assert!(set_can_read_only(false));
    assert!(is_can_read_only(), "the dev lock is locked at boot");

    read_write_for(CAR_A);
    set_can_dev_write_unlocked(false).unwrap();
    assert!(is_can_read_only());
}

/// The vehicle's accepted unlock waits for the owner: approving and unlocking open the
/// bus without the vehicle asking again, and withdrawing either closes it at once.
#[test]
fn approval_and_dev_lock_take_effect_without_the_vehicle_asking_again() {
    boot();
    identify(CAR_A);
    assert!(set_can_read_only(false));
    approve_can_read_write().unwrap();
    assert!(is_can_read_only());
    set_can_dev_write_unlocked(true).unwrap();
    assert!(!is_can_read_only());

    set_can_dev_write_unlocked(false).unwrap();
    assert!(is_can_read_only());
    set_can_dev_write_unlocked(true).unwrap();
    assert!(!is_can_read_only());
}

// ── Binding to one car ────────────────────────────────────────────────────────

#[test]
fn approval_does_not_carry_over_to_another_car() {
    read_write_for(CAR_A);
    revoke_car_identity();
    identify(CAR_B);
    assert_eq!(can_read_write_approval(), Approval::OtherCar);
    assert!(set_can_read_only(false));
    assert!(
        is_can_read_only(),
        "a car of the same model is not the approved car"
    );

    // Approving here replaces the approval: back on car A, it no longer applies.
    approve_can_read_write().unwrap();
    assert!(!set_can_read_only(false));
    revoke_car_identity();
    identify(CAR_A);
    assert_eq!(can_read_write_approval(), Approval::OtherCar);
    assert!(set_can_read_only(false));
}

#[test]
fn the_same_fingerprint_on_another_platform_is_another_car() {
    read_write_for(CAR_A);
    let stored = try_take_can_read_write_approval_to_store().unwrap();
    boot();
    init(0x1234_5678);
    restore_can_read_write_approval(stored);
    identify(CAR_A);
    assert_eq!(can_read_write_approval(), Approval::OtherCar);
    init(PLATFORM_ID);
}

// ── Persistence ───────────────────────────────────────────────────────────────

#[test]
fn approval_survives_a_reboot() {
    read_write_for(CAR_A);
    let stored = try_take_can_read_write_approval_to_store().expect("approval not stored");
    assert!(stored.is_some());

    // Reboot: RAM state is gone, the board restores what it stored.
    boot();
    restore_can_read_write_approval(stored);
    assert_eq!(can_read_write_approval(), Approval::AwaitingCar);
    assert!(is_can_read_only());

    identify(CAR_A);
    assert_eq!(can_read_write_approval(), Approval::Approved);
    // The dev lock is locked again after the reboot; the approval is not asked again.
    assert!(set_can_read_only(false));
    assert!(is_can_read_only());
    set_can_dev_write_unlocked(true).unwrap();
    assert!(!is_can_read_only());
}

#[test]
fn pairing_or_removing_phones_does_not_reset_the_approval() {
    read_write_for(CAR_A);
    let _ = try_take_can_read_write_approval_to_store();
    embassy_futures::block_on(async {
        assert!(add_paired_phone(PHONE).await);
        assert!(add_paired_phone(STRANGER).await);
        assert!(remove_paired_phone(STRANGER).await);
        clear_paired_phones().await;
    });
    assert_eq!(can_read_write_approval(), Approval::Approved);
    assert!(!is_can_read_only());
    assert!(
        try_take_can_read_write_approval_to_store().is_none(),
        "nothing to rewrite"
    );
}

#[test]
fn revoking_erases_the_stored_approval() {
    read_write_for(CAR_A);
    revoke_can_read_write_approval();
    assert_eq!(try_take_can_read_write_approval_to_store(), Some(None));
}

// ── Relock conditions ─────────────────────────────────────────────────────────

/// A bus fault (the board calls `set_can_read_only(true)`) relocks, and the vehicle has
/// to unlock again; the approval and the dev lock are kept.
#[test]
fn a_bus_fault_relocks_until_the_vehicle_unlocks_again() {
    read_write_for(CAR_A);
    set_can_read_only(true);
    assert!(is_can_read_only());
    assert_eq!(can_read_write_approval(), Approval::Approved);
    assert!(!set_can_read_only(false));
}

#[test]
fn a_lost_identity_relocks_until_the_car_is_identified_again() {
    read_write_for(CAR_A);
    revoke_car_identity();
    assert!(set_can_read_only(false), "refused without an identity");
    identify(CAR_A);
    assert!(is_can_read_only(), "the earlier unlock does not survive");
    assert!(!set_can_read_only(false));
}

#[test]
fn a_logging_session_forces_read_only_and_relocks() {
    read_write_for(CAR_A);
    let session = CanLogSession::start().unwrap();
    assert!(is_can_read_only());
    assert!(set_can_read_only(false), "no unlock while logging");
    assert!(can_access_state().logging.unwrap());
    drop(session);
    assert!(
        is_can_read_only(),
        "still locked until the vehicle unlocks again"
    );
    assert!(!set_can_read_only(false));
}

// ── Who can approve and unlock ────────────────────────────────────────────────

#[test]
fn a_bonded_phone_approves_and_unlocks_over_ble() {
    boot();
    embassy_futures::block_on(add_paired_phone(PHONE));
    identify(CAR_A);
    assert!(set_can_read_only(false));

    embassy_futures::block_on(handle_ble_message(approve_cmd(PHONE, true)));
    let resp = ble_response();
    assert!(resp.success, "{}", resp.error_message);
    assert_eq!(resp.message_id, 7);
    assert_eq!(can_read_write_approval(), Approval::Approved);

    embassy_futures::block_on(handle_ble_message(unlock_cmd(PHONE, true)));
    assert!(ble_response().success);
    assert!(!is_can_read_only());

    embassy_futures::block_on(handle_ble_message(unlock_cmd(PHONE, false)));
    assert!(ble_response().success);
    assert!(is_can_read_only());

    embassy_futures::block_on(handle_ble_message(approve_cmd(PHONE, false)));
    assert!(ble_response().success);
    assert_eq!(can_read_write_approval(), Approval::NotApproved);
}

#[test]
fn approving_before_the_car_is_identified_is_refused() {
    boot();
    embassy_futures::block_on(add_paired_phone(PHONE));
    confirm_car_identity();
    embassy_futures::block_on(handle_ble_message(approve_cmd(PHONE, true)));
    let resp = ble_response();
    assert!(!resp.success);
    assert_eq!(resp.status_code, proto::CommandStatusCode::Failed as i32);
    assert!(resp.error_message.contains("not identified"));
    assert_eq!(can_read_write_approval(), Approval::NotApproved);
}

/// While the pairing window is open, a phone that isn't bonded can still write: the
/// stamped `source_device_id` must be a bonded phone, and an empty one never is.
#[test]
fn a_phone_that_is_not_bonded_cannot_approve_or_unlock() {
    boot();
    embassy_futures::block_on(add_paired_phone(PHONE));
    identify(CAR_A);
    for source in [STRANGER, &[][..]] {
        embassy_futures::block_on(handle_ble_message(approve_cmd(source, true)));
        let resp = ble_response();
        assert!(!resp.success);
        assert!(resp.error_message.contains("bonded"));
        embassy_futures::block_on(handle_ble_message(unlock_cmd(source, true)));
        assert!(!ble_response().success);
    }
    assert_eq!(can_read_write_approval(), Approval::NotApproved);
    assert!(!is_can_dev_write_unlocked());
}

#[test]
fn mqtt_can_never_approve_unlock_or_revoke() {
    boot();
    embassy_futures::block_on(add_paired_phone(PHONE));
    identify(CAR_A);
    // Even with a bonded phone's id: MQTT drops every system command.
    embassy_futures::block_on(handle_mqtt_message(approve_cmd(PHONE, true)));
    embassy_futures::block_on(handle_mqtt_message(unlock_cmd(PHONE, true)));
    assert_eq!(can_read_write_approval(), Approval::NotApproved);
    assert!(!is_can_dev_write_unlocked());
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
    assert!(MQTT_TX_CHANNEL.try_receive().is_err());

    // Nor take an approval away.
    read_write_for(CAR_A);
    embassy_futures::block_on(add_paired_phone(PHONE));
    embassy_futures::block_on(handle_mqtt_message(approve_cmd(PHONE, false)));
    embassy_futures::block_on(handle_mqtt_message(unlock_cmd(PHONE, false)));
    assert_eq!(can_read_write_approval(), Approval::Approved);
    assert!(!is_can_read_only());
}

// ── What the app sees ─────────────────────────────────────────────────────────

#[test]
fn access_state_reports_each_condition() {
    boot();
    let state = can_access_state();
    assert_eq!(state.platform_confirmed, Some(false));
    assert_eq!(state.car_identity_confirmed, Some(false));
    assert_eq!(state.approval, Some(Approval::NotApproved as i32));
    assert_eq!(state.dev_lock_available, Some(true));
    assert_eq!(state.dev_unlocked, Some(false));
    assert_eq!(state.logging, Some(false));
    assert_eq!(state.read_only, Some(true));

    read_write_for(CAR_A);
    let state = can_access_state();
    assert_eq!(state.platform_confirmed, Some(true));
    assert_eq!(state.car_identity_confirmed, Some(true));
    assert_eq!(state.approval, Some(Approval::Approved as i32));
    assert_eq!(state.dev_unlocked, Some(true));
    assert_eq!(state.read_only, Some(false));
}

#[test]
fn access_state_is_published_on_change_and_only_over_ble() {
    boot();
    assert!(publish_can_access_state(1));
    assert!(!publish_can_access_state(2), "unchanged");
    let msg = BLE_TX_CHANNEL.try_receive().unwrap();
    let update = match msg.payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => update,
        other => panic!("expected a state update, got {other:?}"),
    };
    assert!(update.vehicle_state.is_none());
    let access = update.system_state.unwrap().can_access.unwrap();
    assert_eq!(access.read_only, Some(true));

    identify(CAR_A);
    assert!(publish_can_access_state(3));
    assert!(MQTT_TX_CHANNEL.try_receive().is_err());
    drain_tx();
}

/// A message lost as the app subscribes, or a missed change notification, is replaced
/// within `CAN_ACCESS_REPEAT_MS` even if nothing changes.
#[test]
fn an_unchanged_access_state_is_sent_again_periodically() {
    use core_interface::CAN_ACCESS_REPEAT_MS;
    boot();
    assert!(publish_can_access_state(1_000));
    assert!(!publish_can_access_state(1_000 + CAN_ACCESS_REPEAT_MS - 1));
    assert!(publish_can_access_state(1_000 + CAN_ACCESS_REPEAT_MS));
    assert!(BLE_TX_CHANNEL.try_receive().is_ok());
    assert!(BLE_TX_CHANNEL.try_receive().is_ok());
    assert!(BLE_TX_CHANNEL.try_receive().is_err());

    // A full queue leaves it unsent, and the next call retries.
    drain_tx();
    while BLE_TX_CHANNEL
        .try_send(core_interface::can_access_state_update(0))
        .is_ok()
    {}
    assert!(!publish_can_access_state(1_000 + 3 * CAN_ACCESS_REPEAT_MS));
    drain_tx();
    assert!(publish_can_access_state(1_000 + 3 * CAN_ACCESS_REPEAT_MS));
    drain_tx();
}

// ── Vehicle commands while read-only ──────────────────────────────────────────

fn vehicle_cmd(advanced: bool) -> proto::AppToDevice {
    proto::AppToDevice {
        message_id: 11,
        platform_id: PLATFORM_ID,
        source_device_id: PHONE.to_vec(),
        payload: Some(if advanced {
            Payload::AdvancedCommandBytes(vec![1])
        } else {
            Payload::BasicCommandBytes(vec![1])
        }),
    }
}

/// While read-only, vehicle commands are answered with the reason instead of reaching
/// the vehicle, which would otherwise report success for a command it never sent.
#[test]
fn vehicle_commands_are_refused_with_the_reason_while_read_only() {
    use core_interface::{ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, can_read_only_reason};
    boot();
    embassy_futures::block_on(core_interface::reset_ble_controller_lease_for_tests());
    embassy_futures::block_on(add_paired_phone(PHONE));

    let mut reasons = Vec::new();
    let mut refused = |expected: &str| {
        assert_eq!(can_read_only_reason(), Some(expected));
        for advanced in [false, true] {
            embassy_futures::block_on(handle_ble_message(vehicle_cmd(advanced)));
            let resp = ble_response();
            assert!(!resp.success);
            assert_eq!(resp.message_id, 11);
            assert_eq!(resp.status_code, proto::CommandStatusCode::Failed as i32);
            assert!(
                resp.error_message.contains("read-only"),
                "{}",
                resp.error_message
            );
            assert!(
                resp.error_message.contains(expected),
                "{}",
                resp.error_message
            );
        }
        assert!(BASIC_CMD_CHANNEL.try_receive().is_err());
        assert!(ADVANCED_CMD_CHANNEL.try_receive().is_err());
        reasons.push(expected.to_string());
    };

    refused("the car platform is not recognised yet");
    confirm_car_identity();
    refused("the car is not identified yet");
    assert!(confirm_car_fingerprint(CAR_A));
    refused("control is not approved for this car");
    approve_can_read_write().unwrap();
    refused("the developer write lock is locked");
    set_can_dev_write_unlocked(true).unwrap();
    refused("the vehicle has not enabled control");
    assert!(!set_can_read_only(false));
    assert_eq!(can_read_only_reason(), None);

    // Read-write: commands reach the vehicle again.
    embassy_futures::block_on(handle_ble_message(vehicle_cmd(false)));
    assert_eq!(BASIC_CMD_CHANNEL.try_receive().unwrap().message_id, 11);
    embassy_futures::block_on(handle_ble_message(vehicle_cmd(true)));
    assert_eq!(ADVANCED_CMD_CHANNEL.try_receive().unwrap().message_id, 11);

    let session = CanLogSession::start().unwrap();
    refused("CAN logging is active");
    drop(session);

    revoke_car_identity();
    identify(CAR_B);
    refused("control was approved for a different car");
    assert_eq!(reasons.len(), 7);
}
