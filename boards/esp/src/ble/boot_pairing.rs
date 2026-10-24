//! Whether the pairing window may open by itself at boot, and when that
//! first-setup window closes again.

use core::sync::atomic::{AtomicBool, Ordering};

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

/// Set while the window that opened at boot for first setup is the one open.
/// Button windows never set it.
static FIRST_SETUP_WINDOW: AtomicBool = AtomicBool::new(false);

/// Marks the window just opened at boot as the first-setup window.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) fn mark_first_setup_window() {
    FIRST_SETUP_WINDOW.store(true, Ordering::Relaxed);
}

/// Called once a new bond is stored and kept. Returns `true` the first time
/// during a first-setup window, telling the caller to close the window: the
/// owner has paired, and a second phone must not bond in the time left. A
/// stranger who bonds first makes the owner's pairing fail visibly instead of
/// both phones ending up bonded.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) fn close_window_after_bond() -> bool {
    // Load + store: the C3 has no atomic swap, and only the BLE task clears it.
    let first_setup = FIRST_SETUP_WINDOW.load(Ordering::Relaxed);
    FIRST_SETUP_WINDOW.store(false, Ordering::Relaxed);
    first_setup
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_bond_closes_only_the_first_setup_window() {
        // Without a first-setup window (button board), a bond leaves it open.
        assert!(!close_window_after_bond());

        mark_first_setup_window();
        assert!(close_window_after_bond());
        // Only once: a later bond (e.g. in a button window) doesn't close it.
        assert!(!close_window_after_bond());
    }

    #[test]
    fn boot_window_only_opens_for_first_setup() {
        assert!(open_window_at_boot(0));
        assert!(!open_window_at_boot(1));
        assert!(!open_window_at_boot(8));
    }
}
