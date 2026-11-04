use core_interface::{CommandAuthorization, CommandError, Rejection, VEHICLE_STATE_CHANNEL};
use prost::Message;
use virtual_car_controller::{
    CAR_STATE, COMMAND_REPEATS, process_basic_command,
    proto::{BasicCommand, DoorLockCommand, basic_command::Action},
};

/// What the dispatcher hands a command from a validated phone.
fn auth() -> CommandAuthorization {
    CommandAuthorization::for_tests(b"ble:\x01\x02\x03\x04\x05\x06")
}

fn encode_basic_cmd(action: Option<Action>) -> Vec<u8> {
    BasicCommand { action }.encode_to_vec()
}

// ── DoorLock ──────────────────────────────────────────────────────────────────

#[test]
fn door_lock_true_succeeds_and_updates_channel() {
    COMMAND_REPEATS.reset();
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: true })));
    let result = embassy_futures::block_on(process_basic_command(&bytes, Some(&auth())));
    assert!(result.is_ok());
    let payload = VEHICLE_STATE_CHANNEL
        .try_receive()
        .expect("no state update after door lock");
    // Verify the state was actually updated by decoding the basic state
    use prost::Message;
    let basic =
        virtual_car_controller::proto::BasicState::decode(payload.basic.as_slice()).unwrap();
    assert_eq!(basic.are_doors_locked, Some(true));
}

#[test]
fn door_lock_false_succeeds_and_updates_channel() {
    COMMAND_REPEATS.reset();
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: false })));
    let result = embassy_futures::block_on(process_basic_command(&bytes, Some(&auth())));
    assert!(result.is_ok());
    let payload = VEHICLE_STATE_CHANNEL
        .try_receive()
        .expect("no state update after door unlock");
    let basic =
        virtual_car_controller::proto::BasicState::decode(payload.basic.as_slice()).unwrap();
    assert_eq!(basic.are_doors_locked, Some(false));
}

// ── Error cases ───────────────────────────────────────────────────────────────

#[test]
fn malformed_bytes_return_err_and_no_channel_update() {
    let result =
        embassy_futures::block_on(process_basic_command(&[0xFF, 0xFF, 0xFF], Some(&auth())));
    assert!(result.is_err());
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

#[test]
fn empty_bytes_return_err() {
    let result = embassy_futures::block_on(process_basic_command(&[], Some(&auth())));
    // Empty bytes decode to an empty BasicCommand (no action)
    assert!(result.is_err());
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

#[test]
fn none_action_returns_no_action_error() {
    let bytes = encode_basic_cmd(None);
    let result = embassy_futures::block_on(process_basic_command(&bytes, Some(&auth())));
    assert_eq!(
        result,
        Err(CommandError::Failed("No action in BasicCommand"))
    );
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

// ── Requirements ──────────────────────────────────────────────────────────────

fn set_driving(driving: Option<bool>) {
    embassy_futures::block_on(async {
        CAR_STATE.lock().await.is_driving = driving;
    });
}

#[test]
fn door_lock_is_refused_while_the_simulated_car_drives() {
    set_driving(Some(true));
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: true })));
    let result = embassy_futures::block_on(process_basic_command(&bytes, Some(&auth())));
    match result {
        Err(CommandError::Rejected(Rejection::Unmet(message))) => {
            assert_eq!(message, "command rejected: the car is driving");
        }
        other => panic!("expected a rejection, got {other:?}"),
    }
    assert!(
        VEHICLE_STATE_CHANNEL.try_receive().is_err(),
        "a refused command must not change the state"
    );
    set_driving(Some(false));
}

#[test]
fn door_lock_with_an_unknown_driving_state_is_refused() {
    set_driving(None);
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: true })));
    let result = embassy_futures::block_on(process_basic_command(&bytes, Some(&auth())));
    assert!(matches!(
        result,
        Err(CommandError::Rejected(Rejection::Unknown(_)))
    ));
    set_driving(Some(false));
}

// ── Repeat limit ──────────────────────────────────────────────────────────────

#[test]
fn a_second_door_lock_inside_the_interval_is_refused() {
    COMMAND_REPEATS.reset();
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: true })));
    assert!(embassy_futures::block_on(process_basic_command(&bytes, Some(&auth()))).is_ok());
    VEHICLE_STATE_CHANNEL.try_receive().expect("a state update");

    let unlock = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: false })));
    let result = embassy_futures::block_on(process_basic_command(&unlock, Some(&auth())));
    assert!(matches!(
        result,
        Err(CommandError::Rejected(Rejection::TooSoon(_)))
    ));
    assert!(
        VEHICLE_STATE_CHANNEL.try_receive().is_err(),
        "a refused command must not change the state"
    );
    COMMAND_REPEATS.reset();
}

#[test]
fn door_lock_sends_a_command_frame_for_the_sending_phone() {
    COMMAND_REPEATS.reset();
    let channel = core_interface::can_tx_channel(0).unwrap();
    while channel.try_receive().is_ok() {}
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: true })));
    assert!(embassy_futures::block_on(process_basic_command(&bytes, Some(&auth()))).is_ok());
    while VEHICLE_STATE_CHANNEL.try_receive().is_ok() {}
    let frame = channel.try_receive().expect("the door lock frame");
    assert!(frame.is_command());
    assert_eq!(frame.frame().data[0], 1);
}

#[test]
fn door_lock_without_an_authorization_fails() {
    COMMAND_REPEATS.reset();
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: true })));
    let result = embassy_futures::block_on(process_basic_command(&bytes, None));
    assert!(matches!(result, Err(CommandError::Failed(_))));
}
