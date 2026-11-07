//! Phone trust (docs/ble.md, "Phone trust"): vehicle commands need a validated phone, the
//! validation session (methods, code entry, in-car action, timeouts, cooldown), validations
//! bound to one car, who can change the approval or remove a validated phone, persistence,
//! and the command TX class.

use core_interface::{
    BASIC_CMD_CHANNEL, BLE_TX_CHANNEL, CODE_LENGTH, CanFrame, CanTxFrame, CommandAuthorization,
    MAX_CODE_ATTEMPTS, VALIDATION_SESSION_MS, ValidationKind, ValidationMethod, add_paired_phone,
    allow_can_tx, approve_can_read_write, car_has_validated_phone, clear_paired_phones,
    complete_validation_action, confirm_car_fingerprint, confirm_car_identity,
    current_validation_challenge, declare_validation_methods, encode_phone_validations,
    forget_phone_validation, handle_ble_message, init, is_phone_validated, phone_access_state,
    phone_trust_level, proto, prune_phone_validations, remove_paired_phone,
    reset_can_access_for_tests, reset_phone_trust_for_tests, reset_phone_validation_for_tests,
    restore_phone_validations, revoke_car_identity, set_ble_peer, set_can_dev_write_unlocked,
    set_can_read_only, set_random_source, tick_validation, try_take_phone_validations_to_store,
    validate_phone_for_tests,
};
use embedded_can::{Id, StandardId};
use proto::app_to_device::Payload;
use proto::system_command::Action;
use proto::{PhoneTrustLevel as Trust, PhoneValidationStatus as Status};
use std::sync::atomic::{AtomicBool, Ordering};

const PLATFORM_ID: u32 = 0xDEAD_BEEF;
const CAR_A: &[u8] = b"KNAC381AAP5000001";
const CAR_B: &[u8] = b"KNAC381AAP5000002";
const OWNER: &[u8] = b"ble:\x01\x02\x03\x04\x05\x06";
const STRANGER: &[u8] = b"ble:\x0A\x0B\x0C\x0D\x0E\x0F";

/// What the vehicle says about "test.live" right now.
static LIVE_METHOD_AVAILABLE: AtomicBool = AtomicBool::new(false);

static METHODS: [ValidationMethod; 3] = [
    ValidationMethod {
        id: "test.code",
        kind: ValidationKind::CodeEntry,
        needs_approval: true,
        is_available: || true,
    },
    ValidationMethod {
        id: "test.button",
        kind: ValidationKind::InCarAction,
        needs_approval: false,
        is_available: || true,
    },
    ValidationMethod {
        id: "test.live",
        kind: ValidationKind::InCarAction,
        needs_approval: false,
        is_available: || LIVE_METHOD_AVAILABLE.load(Ordering::Relaxed),
    },
];

/// Every code is 123456, cut to `CODE_LENGTH` digits.
fn fixed_random(buf: &mut [u8]) {
    let bytes = 123_456u32.to_le_bytes();
    for (i, b) in buf.iter_mut().enumerate() {
        *b = bytes[i % 4];
    }
}

fn boot() {
    init(PLATFORM_ID);
    revoke_car_identity();
    reset_can_access_for_tests();
    embassy_futures::block_on(clear_paired_phones());
    reset_phone_trust_for_tests();
    reset_phone_validation_for_tests();
    declare_validation_methods(&METHODS);
    LIVE_METHOD_AVAILABLE.store(false, Ordering::Relaxed);
    set_random_source(fixed_random);
    set_can_read_only(true);
    core_interface::set_link_up(core_interface::Transport::Ble, true);
    core_interface::reset_recent_commands_for_tests();
    embassy_futures::block_on(core_interface::reset_ble_controller_lease_for_tests());
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
    for phone in [OWNER, STRANGER] {
        embassy_futures::block_on(add_paired_phone(phone));
    }
}

fn identify(car: &[u8]) {
    confirm_car_identity();
    assert!(confirm_car_fingerprint(car));
}

/// Car A identified and approved, dev lock unlocked, the vehicle asked to unlock.
fn read_write() {
    boot();
    identify(CAR_A);
    approve_can_read_write().unwrap();
    set_can_dev_write_unlocked(true).unwrap();
    assert!(!set_can_read_only(false));
}

fn next_message_id() -> u64 {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

fn send(source: &[u8], payload: Payload) -> proto::CommandResponse {
    let msg = proto::AppToDevice {
        message_id: next_message_id(),
        platform_id: PLATFORM_ID,
        source_device_id: source.to_vec(),
        payload: Some(payload),
    };
    embassy_futures::block_on(handle_ble_message(msg));
    loop {
        let msg = BLE_TX_CHANNEL.try_receive().expect("no BLE response");
        if let Some(proto::device_to_app::Payload::CommandResponse(resp)) = msg.payload {
            return resp;
        }
    }
}

fn system(source: &[u8], action: Action) -> proto::CommandResponse {
    send(
        source,
        Payload::SystemCommand(proto::SystemCommand {
            action: Some(action),
        }),
    )
}

fn result_of(resp: &proto::CommandResponse) -> Option<proto::system_command_result::Result> {
    match &resp.response_data {
        Some(proto::command_response::ResponseData::SystemResult(r)) => r.result.clone(),
        _ => None,
    }
}

fn session_of(resp: &proto::CommandResponse) -> proto::PhoneValidationSession {
    match result_of(resp) {
        Some(proto::system_command_result::Result::ValidationSession(s)) => s,
        other => panic!("no session in {resp:?}: {other:?}"),
    }
}

fn start(source: &[u8], method_id: &str) -> proto::CommandResponse {
    system(
        source,
        Action::StartPhoneValidation(proto::StartPhoneValidationCommand {
            method_id: method_id.into(),
        }),
    )
}

fn submit(source: &[u8], code: &str) -> proto::CommandResponse {
    system(
        source,
        Action::SubmitPhoneValidationCode(proto::SubmitPhoneValidationCodeCommand {
            code: code.into(),
        }),
    )
}

/// The running session's code, as the car shows it.
fn the_code() -> String {
    let code = current_validation_challenge()
        .and_then(|c| c.code)
        .expect("a code entry session");
    String::from_utf8(code.to_vec()).unwrap()
}

/// The code `fixed_random` draws.
fn expected_code() -> String {
    let space = 10u32.pow(CODE_LENGTH as u32);
    format!("{:0width$}", 123_456 % space, width = CODE_LENGTH)
}

/// A code `fixed_random` never draws.
fn wrong_code() -> String {
    "0".repeat(CODE_LENGTH)
}

fn approve(source: &[u8], approved: bool) -> proto::CommandResponse {
    system(
        source,
        Action::SetCanReadWriteApproval(proto::SetCanReadWriteApprovalCommand { approved }),
    )
}

/// Validates `phone` with the in-car action method.
fn validate_with_button(phone: &[u8]) {
    let resp = start(phone, "test.button");
    assert!(resp.success, "{}", resp.error_message);
    let challenge = current_validation_challenge().expect("a challenge for the vehicle");
    assert!(complete_validation_action(challenge.session));
    assert!(is_phone_validated(phone));
}

fn basic_command(source: &[u8]) -> proto::CommandResponse {
    let msg = proto::AppToDevice {
        message_id: next_message_id(),
        platform_id: PLATFORM_ID,
        source_device_id: source.to_vec(),
        payload: Some(Payload::BasicCommandBytes(vec![0x01])),
    };
    embassy_futures::block_on(handle_ble_message(msg));
    match BLE_TX_CHANNEL.try_receive() {
        Ok(msg) => match msg.payload {
            Some(proto::device_to_app::Payload::CommandResponse(resp)) => resp,
            other => panic!("unexpected {other:?}"),
        },
        // Forwarded to the vehicle, which answers.
        Err(_) => proto::CommandResponse {
            success: true,
            ..Default::default()
        },
    }
}

fn frame() -> CanFrame {
    CanFrame {
        bus_id: 0,
        id: Id::Standard(StandardId::new(0x310).unwrap()),
        data: [1; 8],
        dlc: 8,
    }
}

// ── Vehicle commands need a validated phone ───────────────────────────────────

#[test]
fn a_bonded_phone_cant_send_vehicle_commands_until_validated() {
    read_write();
    let resp = basic_command(OWNER);
    assert!(!resp.success);
    assert_eq!(
        resp.status_code,
        proto::CommandStatusCode::RejectedNotValidated as i32
    );
    assert!(BASIC_CMD_CHANNEL.try_receive().is_err());

    validate_with_button(OWNER);
    assert!(basic_command(OWNER).success);
    let cmd = BASIC_CMD_CHANNEL.try_receive().expect("forwarded");
    assert_eq!(cmd.authorization.unwrap().device_id(), OWNER);

    // Another bonded phone is still refused.
    let resp = basic_command(STRANGER);
    assert_eq!(
        resp.status_code,
        proto::CommandStatusCode::RejectedNotValidated as i32
    );
}

#[test]
fn read_only_is_reported_before_validation() {
    boot();
    let resp = basic_command(OWNER);
    assert_eq!(
        resp.status_code,
        proto::CommandStatusCode::RejectedUnavailable as i32
    );
}

// ── Sessions ──────────────────────────────────────────────────────────────────

#[test]
fn methods_are_listed_with_their_availability() {
    boot();
    identify(CAR_A);
    let resp = system(
        OWNER,
        Action::ListPhoneValidationMethods(proto::ListPhoneValidationMethodsCommand {}),
    );
    let methods = match result_of(&resp) {
        Some(proto::system_command_result::Result::ValidationMethods(m)) => m.methods,
        other => panic!("{other:?}"),
    };
    assert_eq!(methods.len(), 3);
    assert_eq!(methods[0].method_id, "test.code");
    assert_eq!(
        methods[0].kind,
        proto::PhoneValidationMethodKind::CodeEntry as i32
    );
    assert!(methods[0].needs_approval);
    assert!(!methods[0].available, "read-only: the code can't be shown");
    assert!(methods[1].available);

    // The code method can't start while read-only either.
    let resp = start(OWNER, "test.code");
    assert!(!resp.success);
    assert!(
        resp.error_message.contains("read-only"),
        "{}",
        resp.error_message
    );
}

#[test]
fn the_vehicle_decides_live_whether_a_method_can_start() {
    boot();
    identify(CAR_A);
    let live_available = || {
        let resp = system(
            OWNER,
            Action::ListPhoneValidationMethods(proto::ListPhoneValidationMethodsCommand {}),
        );
        match result_of(&resp) {
            Some(proto::system_command_result::Result::ValidationMethods(m)) => {
                m.methods[2].available
            }
            other => panic!("{other:?}"),
        }
    };
    assert!(!live_available());
    let resp = start(OWNER, "test.live");
    assert!(!resp.success);
    assert!(
        resp.error_message.contains("current state"),
        "{}",
        resp.error_message
    );

    LIVE_METHOD_AVAILABLE.store(true, Ordering::Relaxed);
    assert!(live_available());
    assert!(start(OWNER, "test.live").success);

    // A running session doesn't depend on it: the vehicle can still complete it.
    LIVE_METHOD_AVAILABLE.store(false, Ordering::Relaxed);
    let session = current_validation_challenge().unwrap().session;
    assert!(complete_validation_action(session));
    assert!(is_phone_validated(OWNER));
}

#[test]
fn only_a_bonded_phone_can_validate() {
    read_write();
    let unknown = b"ble:\x09\x09\x09\x09\x09\x09";
    let resp = start(unknown, "test.button");
    assert!(!resp.success);
    assert!(current_validation_challenge().is_none());
    assert_eq!(phone_trust_level(unknown), Trust::NotBonded);
}

#[test]
fn a_session_needs_the_exact_car() {
    boot();
    confirm_car_identity();
    let resp = start(OWNER, "test.button");
    assert!(!resp.success);
    assert!(resp.error_message.contains("not identified"));
}

#[test]
fn the_right_code_validates_the_phone() {
    read_write();
    let resp = start(OWNER, "test.code");
    assert!(resp.success, "{}", resp.error_message);
    let session = session_of(&resp);
    assert_eq!(session.status, Status::InProgress as i32);
    assert_eq!(session.code_length, CODE_LENGTH as u32);
    assert_eq!(session.attempts_left, MAX_CODE_ATTEMPTS);
    assert!(session.expires_in_s > 0);
    let challenge = current_validation_challenge().unwrap();
    assert_eq!(challenge.method_id, "test.code");
    assert_eq!(the_code(), expected_code());

    let resp = submit(OWNER, &expected_code());
    assert!(resp.success, "{}", resp.error_message);
    assert_eq!(session_of(&resp).status, Status::Succeeded as i32);
    assert!(is_phone_validated(OWNER));
    assert!(
        current_validation_challenge().is_none(),
        "the vehicle stops showing the code"
    );
}

#[test]
fn wrong_codes_use_the_attempts_then_the_cooldown_refuses_new_sessions() {
    read_write();
    assert!(start(OWNER, "test.code").success);
    for left in (0..MAX_CODE_ATTEMPTS).rev() {
        let resp = submit(OWNER, &wrong_code());
        assert!(!resp.success);
        let session = session_of(&resp);
        assert_eq!(session.attempts_left, left);
        if left == 0 {
            assert_eq!(session.status, Status::Failed as i32);
            assert!(session.cooldown_s > 0);
        }
    }
    assert!(!is_phone_validated(OWNER));
    assert!(current_validation_challenge().is_none());
    // The right code no longer works, and no session starts during the cooldown.
    assert!(!submit(OWNER, &expected_code()).success);
    let resp = start(OWNER, "test.button");
    assert!(!resp.success);
    assert!(
        resp.error_message.contains("too many"),
        "{}",
        resp.error_message
    );
    assert!(
        !start(STRANGER, "test.button").success,
        "the cooldown is for every phone"
    );
}

#[test]
fn one_session_at_a_time_and_it_belongs_to_its_phone() {
    read_write();
    assert!(start(OWNER, "test.code").success);
    let resp = start(STRANGER, "test.button");
    assert!(!resp.success);
    assert!(resp.error_message.contains("another phone"));
    // The stranger can't answer the owner's code, or complete it.
    assert!(!submit(STRANGER, &the_code()).success);
    assert!(!is_phone_validated(STRANGER));
    // The code session can't be completed as an in-car action.
    let challenge = current_validation_challenge().unwrap();
    assert!(!complete_validation_action(challenge.session));
    assert!(!is_phone_validated(OWNER));
}

#[test]
fn starting_again_replaces_the_session_and_a_stale_action_is_ignored() {
    read_write();
    assert!(start(OWNER, "test.button").success);
    let first = current_validation_challenge().unwrap();
    assert!(start(OWNER, "test.button").success);
    let second = current_validation_challenge().unwrap();
    assert_ne!(first.session, second.session);
    assert!(!complete_validation_action(first.session));
    assert!(!is_phone_validated(OWNER));
    assert!(complete_validation_action(second.session));
    assert!(is_phone_validated(OWNER));
}

#[test]
fn a_session_times_out_or_is_cancelled() {
    read_write();
    assert!(start(OWNER, "test.button").success);
    let challenge = current_validation_challenge().unwrap();
    let now = embassy_time::Instant::now().as_millis();
    tick_validation(now + VALIDATION_SESSION_MS + 1);
    assert!(current_validation_challenge().is_none());
    assert!(!complete_validation_action(challenge.session));
    set_ble_peer(Some(OWNER));
    let state = phone_access_state(now + VALIDATION_SESSION_MS + 2).unwrap();
    assert_eq!(state.validation.unwrap().status, Status::TimedOut as i32);

    assert!(start(OWNER, "test.button").success);
    let resp = system(
        OWNER,
        Action::CancelPhoneValidation(proto::CancelPhoneValidationCommand {}),
    );
    assert!(resp.success);
    assert!(current_validation_challenge().is_none());
}

#[test]
fn another_car_cancels_the_session() {
    read_write();
    assert!(start(OWNER, "test.button").success);
    let challenge = current_validation_challenge().unwrap();
    revoke_car_identity();
    identify(CAR_B);
    assert!(!complete_validation_action(challenge.session));
    assert!(!is_phone_validated(OWNER));
}

#[test]
fn code_entry_is_refused_without_a_random_source() {
    read_write();
    reset_phone_validation_for_tests();
    declare_validation_methods(&METHODS);
    let resp = start(OWNER, "test.code");
    assert!(!resp.success);
    assert!(resp.error_message.contains("can't generate"));
}

// ── Bound to the car ──────────────────────────────────────────────────────────

#[test]
fn validations_only_apply_to_their_car_and_end_when_another_is_approved() {
    read_write();
    validate_with_button(OWNER);
    assert!(car_has_validated_phone());

    revoke_car_identity();
    assert!(!is_phone_validated(OWNER), "no car identified");
    identify(CAR_B);
    assert!(!is_phone_validated(OWNER));
    assert_eq!(phone_trust_level(OWNER), Trust::Bonded);
    assert!(!car_has_validated_phone());

    // A bonded phone does the first setup for car B, which erases the validations.
    assert!(approve(STRANGER, true).success);
    revoke_car_identity();
    identify(CAR_A);
    assert!(!is_phone_validated(OWNER));
}

#[test]
fn approving_the_same_car_again_keeps_the_validations() {
    read_write();
    validate_with_button(OWNER);
    assert!(approve(OWNER, false).success);
    assert!(approve(OWNER, true).success);
    assert!(is_phone_validated(OWNER));
}

// ── Who controls what ─────────────────────────────────────────────────────────

#[test]
fn before_any_validation_a_bonded_phone_can_change_the_approval() {
    read_write();
    assert!(approve(STRANGER, false).success);
    assert!(approve(STRANGER, true).success);
}

#[test]
fn once_a_phone_is_validated_only_a_validated_phone_changes_the_approval() {
    read_write();
    validate_with_button(OWNER);
    for approved in [false, true] {
        let resp = approve(STRANGER, approved);
        assert!(!resp.success);
        assert_eq!(
            resp.status_code,
            proto::CommandStatusCode::RejectedNotValidated as i32
        );
    }
    assert_eq!(
        core_interface::can_read_write_approval(),
        proto::CanReadWriteApproval::Approved
    );
    assert!(approve(OWNER, false).success);
}

#[test]
fn a_bonded_phone_cant_remove_a_validated_one() {
    read_write();
    validate_with_button(OWNER);
    for action in [
        Action::RemovePairedPhone(proto::RemovePairedPhoneCommand {
            device_id: OWNER.to_vec(),
        }),
        Action::ClearPairedPhones(proto::ClearPairedPhonesCommand {}),
    ] {
        let resp = system(STRANGER, action);
        assert_eq!(
            resp.status_code,
            proto::CommandStatusCode::RejectedNotValidated as i32
        );
        assert!(
            core_interface::SYSTEM_COMMAND_CHANNEL
                .try_receive()
                .is_err()
        );
    }
    // Removing an unvalidated phone is up to the board (pairing window).
    let msg = proto::AppToDevice {
        message_id: next_message_id(),
        platform_id: PLATFORM_ID,
        source_device_id: STRANGER.to_vec(),
        payload: Some(Payload::SystemCommand(proto::SystemCommand {
            action: Some(Action::RemovePairedPhone(proto::RemovePairedPhoneCommand {
                device_id: STRANGER.to_vec(),
            })),
        })),
    };
    embassy_futures::block_on(handle_ble_message(msg));
    assert!(core_interface::SYSTEM_COMMAND_CHANNEL.try_receive().is_ok());
}

#[test]
fn removing_the_bond_ends_the_validation() {
    read_write();
    validate_with_button(OWNER);
    embassy_futures::block_on(remove_paired_phone(OWNER));
    assert!(!is_phone_validated(OWNER));
    embassy_futures::block_on(add_paired_phone(OWNER));
    assert!(
        !is_phone_validated(OWNER),
        "pairing again doesn't bring it back"
    );

    validate_with_button(OWNER);
    forget_phone_validation(OWNER);
    assert!(!is_phone_validated(OWNER));

    validate_with_button(OWNER);
    embassy_futures::block_on(clear_paired_phones());
    assert!(!is_phone_validated(OWNER));
}

// ── Persistence ───────────────────────────────────────────────────────────────

#[test]
fn validations_survive_a_reboot_and_are_pruned_to_the_bonds() {
    read_write();
    validate_with_button(OWNER);
    assert!(validate_phone_for_tests(STRANGER));
    let stored = try_take_phone_validations_to_store().expect("a write for the board");
    assert_eq!(stored, encode_phone_validations());

    // Reboot: the board restores the validations, then the bonds (only the owner's).
    reset_phone_trust_for_tests();
    embassy_futures::block_on(clear_paired_phones());
    assert!(restore_phone_validations(&stored));
    embassy_futures::block_on(add_paired_phone(OWNER));
    embassy_futures::block_on(prune_phone_validations());
    assert!(is_phone_validated(OWNER));
    assert!(!is_phone_validated(STRANGER));
    assert_ne!(
        try_take_phone_validations_to_store(),
        Some(stored),
        "the pruned list is stored"
    );
}

#[test]
fn malformed_stored_validations_read_as_none() {
    boot();
    identify(CAR_A);
    assert!(!restore_phone_validations(&[]));
    assert!(!restore_phone_validations(&[1, 2, 3]));
    let mut bytes = vec![9];
    bytes.extend_from_slice(&[0; 33]);
    assert!(!restore_phone_validations(&bytes), "unknown version");
    assert!(!car_has_validated_phone());
}

// ── What the app sees ─────────────────────────────────────────────────────────

#[test]
fn the_connected_phone_sees_its_trust_and_session() {
    read_write();
    assert_eq!(phone_access_state(0), None, "no phone connected");
    set_ble_peer(Some(OWNER));
    let state = phone_access_state(0).unwrap();
    assert_eq!(state.trust, Some(Trust::Bonded as i32));
    assert_eq!(state.car_has_validated_phone, Some(false));
    assert_eq!(state.validation, None);

    validate_with_button(OWNER);
    let now = embassy_time::Instant::now().as_millis();
    let state = phone_access_state(now).unwrap();
    assert_eq!(state.trust, Some(Trust::Validated as i32));
    assert_eq!(state.car_has_validated_phone, Some(true));
    assert_eq!(state.validation.unwrap().status, Status::Succeeded as i32);

    // It is sent with the CAN access state.
    let update = core_interface::can_access_state_update(now);
    let Some(proto::device_to_app::Payload::StateUpdate(update)) = update.payload else {
        panic!();
    };
    assert!(update.system_state.unwrap().phone_access.is_some());

    // Disconnecting forgets the peer.
    core_interface::set_link_up(core_interface::Transport::Ble, false);
    assert_eq!(phone_access_state(now), None);
}

// ── TX classes ────────────────────────────────────────────────────────────────

#[test]
fn command_frames_need_a_validated_phone_when_they_are_sent() {
    read_write();
    let auth = CommandAuthorization::for_tests(OWNER);
    assert!(allow_can_tx(0, &CanTxFrame::platform(frame())));
    assert!(!allow_can_tx(0, &CanTxFrame::command(&auth, frame())));

    validate_with_button(OWNER);
    assert!(allow_can_tx(0, &CanTxFrame::command(&auth, frame())));
    assert!(
        !allow_can_tx(1, &CanTxFrame::command(&auth, frame())),
        "only on its own bus"
    );

    // Losing the validation drops frames already queued.
    let queued = CanTxFrame::command(&auth, frame());
    forget_phone_validation(OWNER);
    assert!(!allow_can_tx(0, &queued));
}

#[test]
fn command_frames_never_go_out_while_read_only() {
    read_write();
    validate_with_button(OWNER);
    let auth = CommandAuthorization::for_tests(OWNER);
    set_can_dev_write_unlocked(false).unwrap();
    assert!(!allow_can_tx(0, &CanTxFrame::command(&auth, frame())));
    assert!(!allow_can_tx(0, &CanTxFrame::platform(frame())));
}
