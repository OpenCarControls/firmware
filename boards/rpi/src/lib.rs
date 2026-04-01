use core_interface::Car;

pub struct RpiBoard {
    can_interface: String,
}

impl RpiBoard {
    pub fn init(can_interface: String) -> Self {
        Self { can_interface }
    }

    pub fn run<C: Car>(&mut self, car: C) {
    }
}
