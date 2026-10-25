//! A release build (`cargo xtask build --production`): no dev lock, so the car's
//! identity and the owner's approval are enough. Its own test binary, since
//! `declare_production_build()` holds for the rest of the process.

use core_interface::{
    BLE_TX_CHANNEL, CanAccessError, add_paired_phone, approve_can_read_write, can_access_state,
    confirm_car_fingerprint, confirm_car_identity, declare_production_build, handle_ble_message,
    has_can_dev_write_lock, init, is_can_dev_write_unlocked, is_can_read_only, proto,
    revoke_can_read_write_approval, set_can_dev_write_unlocked, set_can_read_only,
};

#[test]
fn release_builds_have_no_dev_lock_and_need_identity_and_approval() {
    init(0xDEAD_BEEF);
    declare_production_build();
    assert!(!has_can_dev_write_lock());
    assert_eq!(
        set_can_dev_write_unlocked(true),
        Err(CanAccessError::NoDevLock)
    );
    assert!(!is_can_dev_write_unlocked());

    confirm_car_identity();
    assert!(confirm_car_fingerprint(b"KNAC381AAP5000001"));
    assert!(set_can_read_only(false));
    assert!(is_can_read_only(), "still needs the owner's approval");

    approve_can_read_write().unwrap();
    assert!(!is_can_read_only());
    let state = can_access_state();
    assert_eq!(state.dev_lock_available, Some(false));
    assert_eq!(state.dev_unlocked, Some(false));
    assert_eq!(state.read_only, Some(false));

    revoke_can_read_write_approval();
    assert!(is_can_read_only());
}

#[test]
fn release_builds_reject_the_dev_unlock_command() {
    init(0xDEAD_BEEF);
    declare_production_build();
    let phone = b"ble:\x01\x02\x03\x04\x05\x06";
    embassy_futures::block_on(add_paired_phone(phone));
    while BLE_TX_CHANNEL.try_receive().is_ok() {}

    embassy_futures::block_on(handle_ble_message(proto::AppToDevice {
        message_id: 9,
        platform_id: 0xDEAD_BEEF,
        source_device_id: phone.to_vec(),
        payload: Some(proto::app_to_device::Payload::SystemCommand(
            proto::SystemCommand {
                action: Some(proto::system_command::Action::SetCanDevWriteUnlock(
                    proto::SetCanDevWriteUnlockCommand { unlocked: true },
                )),
            },
        )),
    }));
    let resp = loop {
        let msg = BLE_TX_CHANNEL.try_receive().expect("no response");
        if let Some(proto::device_to_app::Payload::CommandResponse(resp)) = msg.payload {
            break resp;
        }
    };
    assert!(!resp.success);
    assert!(resp.error_message.contains("release build"));
    assert!(!is_can_dev_write_unlocked());
}
