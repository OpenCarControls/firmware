use core_interface::{command_availability, proto::CommandAvailabilityState};
use prost::Message;
use virtual_car_controller::{
    CAR_STATE, DOOR_LOCK_COMMAND_ID, TOGGLE_CUSTOM_STATE1_COMMAND_ID,
    proto::{
        AdvancedCommand, BasicCommand, DoorLockCommand, ToggleCustomState1Command,
        advanced_command, basic_command,
    },
};

#[test]
fn command_ids_are_the_wire_field_numbers() {
    let basic = BasicCommand {
        action: Some(basic_command::Action::DoorLock(DoorLockCommand {
            lock: true,
        })),
    }
    .encode_to_vec();
    assert_eq!(u32::from(basic[0] >> 3), DOOR_LOCK_COMMAND_ID);
    let advanced = AdvancedCommand {
        action: Some(advanced_command::Action::ToggleCustomState1(
            ToggleCustomState1Command {},
        )),
    }
    .encode_to_vec();
    assert_eq!(u32::from(advanced[0] >> 3), TOGGLE_CUSTOM_STATE1_COMMAND_ID);
}

#[test]
fn reported_commands_are_available_unless_the_device_is_read_only() {
    core_interface::reset_command_availability_for_tests();
    core_interface::reset_can_access_for_tests();
    core_interface::revoke_car_identity();
    embassy_futures::block_on(virtual_car_controller::report_availability());

    // Nothing identified, approved or unlocked yet: core holds everything back.
    let shown = command_availability().expect("reported");
    assert_eq!(
        shown.basic[&DOOR_LOCK_COMMAND_ID],
        CommandAvailabilityState::Unavailable as i32
    );

    core_interface::confirm_car_identity();
    assert!(core_interface::confirm_car_fingerprint(b"SIMULATED-CAR"));
    core_interface::approve_can_read_write().unwrap();
    core_interface::set_can_dev_write_unlocked(true).unwrap();
    assert!(!core_interface::set_can_read_only(false));
    let shown = command_availability().expect("reported");
    assert_eq!(
        shown.basic[&DOOR_LOCK_COMMAND_ID],
        CommandAvailabilityState::Available as i32
    );
    assert_eq!(
        shown.advanced[&TOGGLE_CUSTOM_STATE1_COMMAND_ID],
        CommandAvailabilityState::Available as i32
    );
    core_interface::revoke_car_identity();
}

#[test]
fn the_door_lock_is_unavailable_while_the_simulated_car_drives() {
    let door_lock = || {
        embassy_futures::block_on(virtual_car_controller::report_availability());
        command_availability().expect("reported").basic[&DOOR_LOCK_COMMAND_ID]
    };
    core_interface::reset_command_availability_for_tests();
    core_interface::reset_can_access_for_tests();
    core_interface::confirm_car_identity();
    assert!(core_interface::confirm_car_fingerprint(b"SIMULATED-CAR"));
    core_interface::approve_can_read_write().unwrap();
    core_interface::set_can_dev_write_unlocked(true).unwrap();
    assert!(!core_interface::set_can_read_only(false));

    embassy_futures::block_on(async { CAR_STATE.lock().await.is_driving = Some(false) });
    assert_eq!(door_lock(), CommandAvailabilityState::Available as i32);

    embassy_futures::block_on(async { CAR_STATE.lock().await.is_driving = Some(true) });
    assert_eq!(door_lock(), CommandAvailabilityState::Unavailable as i32);
    // The advanced command has no requirement.
    assert_eq!(
        command_availability().unwrap().advanced[&TOGGLE_CUSTOM_STATE1_COMMAND_ID],
        CommandAvailabilityState::Available as i32
    );

    embassy_futures::block_on(async { CAR_STATE.lock().await.is_driving = Some(false) });
    core_interface::revoke_car_identity();
}
