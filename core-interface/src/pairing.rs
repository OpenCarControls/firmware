use alloc::vec::Vec;
use core::cell::RefCell;
use portable_atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

static BLE_MAX_BONDED_PHONES: AtomicU8 = AtomicU8::new(8);
/// A blocking lock, so the CAN TX check and the state publisher can read it without
/// awaiting. The API stays async for its callers.
static PAIRED_PHONES: Mutex<CriticalSectionRawMutex, RefCell<Vec<Vec<u8>>>> =
    Mutex::new(RefCell::new(Vec::new()));

pub fn set_ble_max_bonded_phones(max: u8) {
    if max > 0 {
        BLE_MAX_BONDED_PHONES.store(max, Ordering::Relaxed);
    }
}

pub fn ble_max_bonded_phones() -> u8 {
    BLE_MAX_BONDED_PHONES.load(Ordering::Relaxed)
}

pub async fn list_paired_phones() -> Vec<Vec<u8>> {
    PAIRED_PHONES.lock(|phones| phones.borrow().clone())
}

pub async fn paired_phone_count() -> usize {
    PAIRED_PHONES.lock(|phones| phones.borrow().len())
}

pub async fn is_phone_paired(device_id: &[u8]) -> bool {
    is_paired(device_id)
}

/// [`is_phone_paired`] without awaiting.
pub(crate) fn is_paired(device_id: &[u8]) -> bool {
    PAIRED_PHONES.lock(|phones| phones.borrow().iter().any(|id| id.as_slice() == device_id))
}

pub async fn add_paired_phone(device_id: &[u8]) -> bool {
    if device_id.is_empty() {
        return false;
    }
    let added = PAIRED_PHONES.lock(|phones| {
        let mut phones = phones.borrow_mut();
        if phones.iter().any(|id| id.as_slice() == device_id) {
            return true;
        }
        if phones.len() >= ble_max_bonded_phones() as usize {
            return false;
        }
        phones.push(device_id.to_vec());
        true
    });
    crate::phone_trust::notify_phone_access_changed();
    added
}

/// Removes the phone and its validation: a validation never outlives its bond.
pub async fn remove_paired_phone(device_id: &[u8]) -> bool {
    let removed = PAIRED_PHONES.lock(|phones| {
        let mut phones = phones.borrow_mut();
        match phones.iter().position(|id| id.as_slice() == device_id) {
            Some(idx) => {
                phones.remove(idx);
                true
            }
            None => false,
        }
    });
    crate::phone_trust::forget_phone_validation(device_id);
    removed
}

/// Removes every phone and every validation.
pub async fn clear_paired_phones() -> usize {
    let count = PAIRED_PHONES.lock(|phones| {
        let mut phones = phones.borrow_mut();
        let count = phones.len();
        phones.clear();
        count
    });
    crate::phone_trust::clear_phone_validations();
    count
}
