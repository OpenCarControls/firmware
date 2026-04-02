use core_interface::CarController;

pub struct RpiBoard<'a> {
    can_interface: &'a str,
}

impl<'a> RpiBoard<'a> {
    pub fn init(can_interface: &'a str) -> Self {
        Self { can_interface }
    }

    pub fn run<C: CarController>(&mut self, car: C) {}
}
