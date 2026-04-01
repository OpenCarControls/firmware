use core_interface::Car;

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

    pub fn run<C: Car>(&mut self, car: C) {
    }
}
