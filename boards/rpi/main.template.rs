use board_rpi::RpiBoard;

pub const CAN_INTERFACE: &str = "{CAN_INTERFACE}";

fn main() {
    println!("⚠️  Running in RPi Development Mode");
    println!("ℹ️  Note: BLE hardware is required for full remote control functionality.");

    // Initialize the specific car platform controller
    let car = {PLATFORM}_controller::init();
    let mut board = RpiBoard::init(CAN_INTERFACE);
    
    // Pass control to your board library
    board.run(car);
}
