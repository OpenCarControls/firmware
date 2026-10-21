use core_interface::{
    CAN_TX_CHANNELS, CanFrame, MAX_CAN_BUSES, can_read_only_receiver, can_tx_channel,
    send_can_frame, set_can_read_only,
};
use embedded_can::{Id, StandardId};

fn frame(bus_id: u8, raw_id: u16) -> CanFrame {
    CanFrame {
        bus_id,
        id: Id::Standard(StandardId::new(raw_id).unwrap()),
        data: [0; 8],
        dlc: 0,
    }
}

fn drain_all() {
    for channel in CAN_TX_CHANNELS.iter() {
        while channel.try_receive().is_ok() {}
    }
}

#[test]
fn frames_land_only_on_their_own_bus_channel() {
    drain_all();
    assert!(embassy_futures::block_on(send_can_frame(frame(1, 0x100))));

    assert!(can_tx_channel(0).unwrap().try_receive().is_err());
    let got = can_tx_channel(1).unwrap().try_receive().unwrap();
    assert_eq!(got.bus_id, 1);
    assert!(can_tx_channel(1).unwrap().try_receive().is_err());
}

#[test]
fn unknown_bus_is_rejected_without_blocking() {
    drain_all();
    assert!(can_tx_channel(MAX_CAN_BUSES as u8).is_none());
    assert!(!embassy_futures::block_on(send_can_frame(frame(
        MAX_CAN_BUSES as u8,
        0x100
    ))));
}

#[test]
fn read_only_receiver_wakes_on_every_change() {
    set_can_read_only(true);
    let mut rx = can_read_only_receiver().expect("receiver slot");
    // A fresh receiver reports the current value once; consumers must tolerate that.
    assert!(embassy_futures::poll_once(rx.changed()).is_ready());
    assert!(embassy_futures::poll_once(rx.changed()).is_pending());

    core_interface::confirm_car_identity();
    set_can_read_only(false);
    assert!(embassy_futures::poll_once(rx.changed()).is_ready());
    assert!(embassy_futures::poll_once(rx.changed()).is_pending());

    set_can_read_only(true);
    assert!(embassy_futures::poll_once(rx.changed()).is_ready());
}
