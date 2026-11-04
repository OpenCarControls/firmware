//! The simulated validation methods: the button (an in-car action on 0x400) and the code
//! shown on the simulated cluster (0x500).

use core_interface::{
    BLE_TX_CHANNEL, CanFrame, handle_ble_message, proto, proto::app_to_device::Payload,
    proto::system_command::Action,
};
use embedded_can::{Id, StandardId};
use virtual_car_controller::{
    BUTTON_FRAME_ID, BUTTON_METHOD_ID, CLUSTER_CODE_METHOD_ID, VALIDATION_METHODS,
    cluster_code_frame, handle_can_frame, identify_simulated_car,
};

const PLATFORM_ID: u32 = 0xF754_4D7E;
const PHONE: &[u8] = b"ble:\x01\x02\x03\x04\x05\x06";

fn random(buf: &mut [u8]) {
    buf.fill(7);
}

fn setup() {
    core_interface::init(PLATFORM_ID);
    core_interface::revoke_car_identity();
    core_interface::reset_can_access_for_tests();
    core_interface::reset_phone_trust_for_tests();
    core_interface::reset_phone_validation_for_tests();
    core_interface::reset_recent_commands_for_tests();
    core_interface::declare_validation_methods(&VALIDATION_METHODS);
    core_interface::set_random_source(random);
    core_interface::set_link_up(core_interface::Transport::Ble, true);
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
    embassy_futures::block_on(core_interface::clear_paired_phones());
    embassy_futures::block_on(core_interface::add_paired_phone(PHONE));
    identify_simulated_car();
}

fn start(method_id: &str) -> proto::CommandResponse {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    let msg = proto::AppToDevice {
        message_id: NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        platform_id: PLATFORM_ID,
        source_device_id: PHONE.to_vec(),
        payload: Some(Payload::SystemCommand(proto::SystemCommand {
            action: Some(Action::StartPhoneValidation(
                proto::StartPhoneValidationCommand {
                    method_id: method_id.into(),
                },
            )),
        })),
    };
    embassy_futures::block_on(handle_ble_message(msg));
    match BLE_TX_CHANNEL.try_receive().unwrap().payload {
        Some(proto::device_to_app::Payload::CommandResponse(resp)) => resp,
        other => panic!("{other:?}"),
    }
}

fn button(pressed: bool) -> CanFrame {
    CanFrame {
        bus_id: 0,
        id: Id::Standard(StandardId::new(BUTTON_FRAME_ID).unwrap()),
        data: [pressed as u8, 0, 0, 0, 0, 0, 0, 0],
        dlc: 1,
    }
}

#[test]
fn pressing_the_button_validates_the_phone_without_the_approval() {
    setup();
    assert!(start(BUTTON_METHOD_ID).success);
    embassy_futures::block_on(handle_can_frame(button(false)));
    assert!(!core_interface::is_phone_validated(PHONE));
    embassy_futures::block_on(handle_can_frame(button(true)));
    assert!(core_interface::is_phone_validated(PHONE));
}

#[test]
fn the_button_does_nothing_without_a_session() {
    setup();
    embassy_futures::block_on(handle_can_frame(button(true)));
    assert!(!core_interface::is_phone_validated(PHONE));
}

#[test]
fn the_cluster_shows_the_code_once_approved() {
    setup();
    assert!(!start(CLUSTER_CODE_METHOD_ID).success, "read-only");
    core_interface::approve_can_read_write().unwrap();
    core_interface::set_can_dev_write_unlocked(true).unwrap();
    assert!(start(CLUSTER_CODE_METHOD_ID).success);
    let frame = cluster_code_frame().expect("the code frame");
    let code = core_interface::current_validation_challenge()
        .unwrap()
        .code
        .unwrap();
    assert_eq!(&frame.data[..6], &code);
    assert!(cluster_code_frame().is_some());
}
