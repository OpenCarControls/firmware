#![no_std]

use core_interface::CarController;

pub struct VirtualCarController;

impl CarController for VirtualCarController {}

pub fn init() -> VirtualCarController {
    VirtualCarController
}
