#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(clippy::all)]
#![allow(unexpected_cfgs)]
#![allow(unused_imports)]

include!(concat!(env!("OUT_DIR"), "/messages.rs"));

use embedded_can::{Id, StandardId};
use core_interface::{CAN_RX_CHANNEL, CAN_TX_CHANNEL, CanFilter, CanFrame};

// ── CAN Configuration ─────────────────────────────────────────────────────────

pub const CAN_FILTERS: &[CanFilter] = &[];

// ── Vehicle Tasks ─────────────────────────────────────────────────────────────

use crate::{encode_state, CAR_STATE};
use core_interface::VEHICLE_STATE_CHANNEL;

pub async fn handle_can_frame(frame: CanFrame) {
    let id = match frame.id {
        Id::Standard(id) => id.as_raw() as u32,
        Id::Extended(id) => id.as_raw(),
    };

    if let Ok(message) = Messages::from_can_message(id, &frame.data[..frame.dlc as usize]) {
        match message {
            Messages::DummyMessage(m) => {
                let speed_kph = m.dummy_speed() as i32;
                let payload = {
                    let mut state = CAR_STATE.lock().await;
                    state.speed = Some(speed_kph);
                    encode_state(&state)
                };
                VEHICLE_STATE_CHANNEL.sender().send(payload).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn can_rx_task() {
    let receiver = CAN_RX_CHANNEL.receiver();
    loop {
        handle_can_frame(receiver.receive().await).await;
    }
}

#[allow(dead_code)]
pub(crate) async fn send_can_request(id: StandardId, data: &[u8]) {
    let mut buf = [0u8; 8];
    let dlc = data.len().min(8) as u8;
    buf[..dlc as usize].copy_from_slice(&data[..dlc as usize]);
    CAN_TX_CHANNEL
        .sender()
        .send(CanFrame {
            bus_id: 0,
            id: Id::Standard(id),
            data: buf,
            dlc,
        })
        .await;
}
