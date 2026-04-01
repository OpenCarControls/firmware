use core_interface::CarController;

pub struct EspBoard {
    can_tx_pin: u8,
    can_rx_pin: u8,
    modem_tx_pin: u8,
    modem_rx_pin: u8,
}

impl EspBoard {
    pub fn init(can_tx_pin: u8, can_rx_pin: u8, modem_tx_pin: u8, modem_rx_pin: u8) -> Self {
        Self {
            can_tx_pin,
            can_rx_pin,
            modem_tx_pin,
            modem_rx_pin,
        }
    }

    pub fn run<C: CarController>(&mut self, car: C) {
    }
}
