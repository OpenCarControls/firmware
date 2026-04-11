use core_interface::VEHICLE_STATE_CHANNEL;
use embedded_can::{Id, StandardId};
use prost::Message;
use virtual_car_controller::{
    handle_can_frame,
    proto::{AdvancedState, BasicState},
};

fn std_frame(raw_id: u16, data: &[u8]) -> core_interface::CanFrame {
    let mut buf = [0u8; 8];
    let dlc = data.len().min(8) as u8;
    buf[..dlc as usize].copy_from_slice(&data[..dlc as usize]);
    core_interface::CanFrame {
        bus_id: 0,
        id: Id::Standard(StandardId::new(raw_id).unwrap()),
        data: buf,
        dlc,
    }
}

fn decode_advanced(payload: &core_interface::VehicleStatePayload) -> AdvancedState {
    AdvancedState::decode(payload.advanced.as_slice()).unwrap()
}

fn decode_basic(payload: &core_interface::VehicleStatePayload) -> BasicState {
    BasicState::decode(payload.basic.as_slice()).unwrap()
}

// ── 0x100 speed frames ────────────────────────────────────────────────────────

#[test]
fn speed_frame_updates_state_and_sends_to_channel() {
    // 120 kph big-endian i16 = [0x00, 0x78]
    let frame = std_frame(0x100, &[0x00, 0x78]);
    embassy_futures::block_on(handle_can_frame(frame));
    let payload = VEHICLE_STATE_CHANNEL
        .try_receive()
        .expect("no state update");
    assert_eq!(decode_advanced(&payload).speed, Some(120));
}

#[test]
fn speed_frame_too_short_does_not_update_state() {
    let frame = std_frame(0x100, &[0x00]); // dlc=1, need 2
    embassy_futures::block_on(handle_can_frame(frame));
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

#[test]
fn speed_frame_negative_value_decoded_correctly() {
    // -10 kph big-endian i16 = [0xFF, 0xF6]
    let frame = std_frame(0x100, &[0xFF, 0xF6]);
    embassy_futures::block_on(handle_can_frame(frame));
    let payload = VEHICLE_STATE_CHANNEL
        .try_receive()
        .expect("no state update");
    assert_eq!(decode_advanced(&payload).speed, Some(-10));
}

// ── 0x200 gear frames ─────────────────────────────────────────────────────────

#[test]
fn gear_frame_updates_state_and_sends_to_channel() {
    let frame = std_frame(0x200, &[1]); // GEAR_PARK = 1
    embassy_futures::block_on(handle_can_frame(frame));
    let payload = VEHICLE_STATE_CHANNEL
        .try_receive()
        .expect("no state update");
    assert_eq!(decode_advanced(&payload).gear, Some(1));
}

#[test]
fn gear_frame_too_short_does_not_update_state() {
    let frame = std_frame(0x200, &[]); // dlc=0, need 1
    embassy_futures::block_on(handle_can_frame(frame));
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

// ── 0x300 door frames ─────────────────────────────────────────────────────────

#[test]
fn doors_locked_frame_sets_locked_true() {
    let frame = std_frame(0x300, &[0x01]);
    embassy_futures::block_on(handle_can_frame(frame));
    let payload = VEHICLE_STATE_CHANNEL
        .try_receive()
        .expect("no state update");
    assert_eq!(decode_basic(&payload).are_doors_locked, Some(true));
}

#[test]
fn doors_unlocked_frame_sets_locked_false() {
    let frame = std_frame(0x300, &[0x00]);
    embassy_futures::block_on(handle_can_frame(frame));
    let payload = VEHICLE_STATE_CHANNEL
        .try_receive()
        .expect("no state update");
    assert_eq!(decode_basic(&payload).are_doors_locked, Some(false));
}

#[test]
fn doors_frame_too_short_does_not_update_state() {
    let frame = std_frame(0x300, &[]); // dlc=0, need 1
    embassy_futures::block_on(handle_can_frame(frame));
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}

// ── Unknown IDs ───────────────────────────────────────────────────────────────

#[test]
fn unknown_can_id_does_not_write_to_channel() {
    let frame = std_frame(0x400, &[0xDE, 0xAD]);
    embassy_futures::block_on(handle_can_frame(frame));
    assert!(VEHICLE_STATE_CHANNEL.try_receive().is_err());
}
