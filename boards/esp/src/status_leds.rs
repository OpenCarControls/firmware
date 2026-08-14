use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use core::cell::Cell;

// Bit 0: Power/Booted
// Bit 1: Wireless Connected
// Bit 2: CAN Activity
pub static SYSTEM_STATUS_FLAGS: Mutex<CriticalSectionRawMutex, Cell<u32>> = Mutex::new(Cell::new(0));

pub const FLAG_POWER: u32 = 1 << 0;
pub const FLAG_WIRELESS: u32 = 1 << 1;
pub const FLAG_CAN: u32 = 1 << 2;

pub fn set_flag(flag: u32, active: bool) {
    SYSTEM_STATUS_FLAGS.lock(|cell| {
        let mut flags = cell.get();
        if active {
            flags |= flag;
        } else {
            flags &= !flag;
        }
        cell.set(flags);
    });
}

pub fn get_flags() -> u32 {
    SYSTEM_STATUS_FLAGS.lock(|cell| cell.get())
}

pub fn pulse_can() {
    set_flag(FLAG_CAN, true);
}
