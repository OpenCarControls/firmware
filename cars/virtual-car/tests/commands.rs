use core_interface::VEHICLE_STATE_CHANNEL;
use prost::Message;
use virtual_car_controller::{
    process_basic_command,
    proto::{BasicCommand, DoorLockCommand, basic_command::Action},
};

fn encode_basic_cmd(action: Option<Action>) -> Vec<u8> {
    BasicCommand { action }.encode_to_vec()
}

// ── DoorLock ──────────────────────────────────────────────────────────────────

#[test]
fn door_lock_true_succeeds_and_updates_channel() {
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: true })));
    let result = embassy_futures::block_on(process_basic_command(&bytes));
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
    let bytes = encode_basic_cmd(Some(Action::DoorLock(DoorLockCommand { lock: false })));
    let result = embassy_futures::block_on(process_basic_command(&bytes));
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
    let result = embassy_futures::block_on(process_basic_command(&[0xFF, 0xFF, 0xFF]));
    assert!(result.is_err());
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

#[test]
fn empty_bytes_return_err() {
    let result = embassy_futures::block_on(process_basic_command(&[]));
    // Empty bytes decode to an empty BasicCommand (no action)
    assert!(result.is_err());
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

#[test]
fn none_action_returns_no_action_error() {
    let bytes = encode_basic_cmd(None);
    let result = embassy_futures::block_on(process_basic_command(&bytes));
    assert_eq!(result, Err("No action in BasicCommand"));
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}
