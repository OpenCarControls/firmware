#![no_std]

extern crate alloc;

use alloc::string::ToString;
use alloc::vec;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.core.v1.rs"));
}

use proto::MessageEnvelope;

pub fn create_test_envelope() -> MessageEnvelope {
    MessageEnvelope {
        car_id: "virtual_car".to_string(),
        message_id: 1,
        timestamp_ms: 0,
        r#type: proto::message_envelope::MessageType::StateBroadcast as i32,
        success: true,
        error_message: "".to_string(),
        payload: vec![],
    }
}

pub trait CarController {}
