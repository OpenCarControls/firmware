//! Whether the pairing window may open by itself at boot.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Raised by the BLE transport task with the number of bonds it loaded from flash.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) static BONDS_LOADED: Signal<CriticalSectionRawMutex, usize> = Signal::new();

/// Boards without a pairing button open the window at boot only for first setup:
/// once a phone is bonded, a stranger in range of a rebooted device (watchdog
/// reset, brownout) must not be able to bond, because pairing is Just Works.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) fn open_window_at_boot(bond_count: usize) -> bool {
    bond_count == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boot_window_only_opens_for_first_setup() {
        assert!(open_window_at_boot(0));
        assert!(!open_window_at_boot(1));
        assert!(!open_window_at_boot(8));
    }
}
