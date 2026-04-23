use alloc::vec::Vec;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

static BLE_MAX_BONDED_PHONES: AtomicU8 = AtomicU8::new(8);
static PAIRED_PHONES: Mutex<CriticalSectionRawMutex, Vec<Vec<u8>>> = Mutex::new(Vec::new());

pub fn set_ble_max_bonded_phones(max: u8) {
    if max > 0 {
        BLE_MAX_BONDED_PHONES.store(max, Ordering::Relaxed);
    }
}

pub fn ble_max_bonded_phones() -> u8 {
    BLE_MAX_BONDED_PHONES.load(Ordering::Relaxed)
}

pub async fn list_paired_phones() -> Vec<Vec<u8>> {
    PAIRED_PHONES.lock().await.clone()
}

pub async fn paired_phone_count() -> usize {
    PAIRED_PHONES.lock().await.len()
}

pub async fn is_phone_paired(device_id: &[u8]) -> bool {
    PAIRED_PHONES
        .lock()
        .await
        .iter()
        .any(|id| id.as_slice() == device_id)
}

pub async fn add_paired_phone(device_id: &[u8]) -> bool {
    if device_id.is_empty() {
        return false;
    }
    let mut phones = PAIRED_PHONES.lock().await;
    if phones.iter().any(|id| id.as_slice() == device_id) {
        return true;
    }
    if phones.len() >= ble_max_bonded_phones() as usize {
        return false;
    }
    phones.push(device_id.to_vec());
    true
}

pub async fn remove_paired_phone(device_id: &[u8]) -> bool {
    let mut phones = PAIRED_PHONES.lock().await;
    if let Some(idx) = phones.iter().position(|id| id.as_slice() == device_id) {
        phones.remove(idx);
        return true;
    }
    false
}

pub async fn clear_paired_phones() -> usize {
    let mut phones = PAIRED_PHONES.lock().await;
    let count = phones.len();
    phones.clear();
    count
}
