#![no_std]

extern crate alloc;

use alloc::vec::Vec;

pub mod proto {
    // The package name in virtual_car.proto is "virtual_car"
    include!(concat!(env!("OUT_DIR"), "/opencar.cars.virtual_car.v1.rs"));
}

use proto::{State, Command};
use prost::Message;

pub fn generate_mock_state() -> Vec<u8> {
    let state = State {
        speed: Some(45),
        are_doors_locked: Some(true),
        ..Default::default()
    };
    
    // This serializes the struct into a raw byte array, ready to be 
    // handed to the core-interface and stuffed into the MessageEnvelope!
    state.encode_to_vec()
}

use core_interface::CarController;

pub struct VirtualCarController;

impl CarController for VirtualCarController {}

pub fn init() -> VirtualCarController {
    VirtualCarController
}
