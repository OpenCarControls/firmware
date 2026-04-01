use board_esp::EspBoard;

pub const CAN_TX_PIN: u8 = {CAN_TX_PIN};
pub const CAN_RX_PIN: u8 = {CAN_RX_PIN};
pub const MODEM_TX_PIN: u8 = {MODEM_TX_PIN};
pub const MODEM_RX_PIN: u8 = {MODEM_RX_PIN};

fn main() {
    // Bind the ESP-IDF patches
    esp_idf_svc::sys::link_patches();

    let car = car_{CAR_MODULE}::init();
    let mut board = EspBoard::init(CAN_TX_PIN, CAN_RX_PIN, MODEM_TX_PIN, MODEM_RX_PIN);
    
    // Pass control to your board library
    board.run(car);
}
