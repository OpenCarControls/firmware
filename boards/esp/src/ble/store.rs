#[cfg(feature = "hardware")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "hardware")]
use embassy_sync::mutex::Mutex;
#[cfg(feature = "hardware")]
use embedded_storage::{ReadStorage, Storage};
#[cfg(feature = "hardware")]
use esp_hal::peripherals;
#[cfg(feature = "hardware")]
use esp_storage::FlashStorage;

#[cfg(feature = "hardware")]
const BLE_STORE_MAGIC: u32 = 0x4F43_424C; // "OCBL"
#[cfg(feature = "hardware")]
const BLE_STORE_VERSION: u8 = 1;
#[cfg(feature = "hardware")]
const BLE_STORE_MAX_PHONES: usize = 8;
#[cfg(feature = "hardware")]
const BLE_STORE_MAX_ID_LEN: usize = 32;
#[cfg(feature = "hardware")]
const BLE_STORE_FLASH_SECTORS: usize = 1;

#[cfg(feature = "hardware")]
#[repr(C)]
#[derive(Clone, Copy)]
struct BleBondStore {
    magic: u32,
    version: u8,
    count: u8,
    reserved: [u8; 2],
    lens: [u8; BLE_STORE_MAX_PHONES],
    ids: [[u8; BLE_STORE_MAX_ID_LEN]; BLE_STORE_MAX_PHONES],
    checksum: u32,
}

#[cfg(feature = "hardware")]
impl BleBondStore {
    const fn empty() -> Self {
        Self {
            magic: BLE_STORE_MAGIC,
            version: BLE_STORE_VERSION,
            count: 0,
            reserved: [0; 2],
            lens: [0; BLE_STORE_MAX_PHONES],
            ids: [[0; BLE_STORE_MAX_ID_LEN]; BLE_STORE_MAX_PHONES],
            checksum: 0,
        }
    }
}

#[cfg(feature = "hardware")]
static BLE_BOND_STORE_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());
#[cfg(feature = "hardware")]
static BLE_FLASH_STORE: Mutex<CriticalSectionRawMutex, Option<FlashStorage<'static>>> =
    Mutex::new(None);

#[cfg(feature = "hardware")]
fn ble_store_flash_offset(capacity: usize) -> u32 {
    let reserve = (FlashStorage::SECTOR_SIZE as usize) * BLE_STORE_FLASH_SECTORS;
    capacity.saturating_sub(reserve) as u32
}

#[cfg(feature = "hardware")]
fn ble_store_write_bytes(store: &BleBondStore, out: &mut [u8]) {
    let n = core::mem::size_of::<BleBondStore>();
    if out.len() < n {
        return;
    }
    let src = store as *const BleBondStore as *const u8;
    // Safety: `store` is a valid repr(C) POD-like value and `out` has
    // at least `size_of::<BleBondStore>()` bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(src, out.as_mut_ptr(), n);
    }
}

#[cfg(feature = "hardware")]
fn ble_store_read_bytes(src: &[u8]) -> Option<BleBondStore> {
    let n = core::mem::size_of::<BleBondStore>();
    if src.len() < n {
        return None;
    }
    if src[..n].iter().all(|b| *b == 0xFF) {
        return None;
    }
    let mut out = core::mem::MaybeUninit::<BleBondStore>::uninit();
    // Safety: destination is properly sized/aligned and source has at least n bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(src.as_ptr(), out.as_mut_ptr() as *mut u8, n);
        Some(out.assume_init())
    }
}

#[cfg(feature = "hardware")]
pub(super) async fn init_paired_phones_flash_store(flash_peri: peripherals::FLASH<'static>) {
    let mut guard = BLE_FLASH_STORE.lock().await;
    if guard.is_some() {
        return;
    }

    let flash = FlashStorage::new(flash_peri);
    #[cfg(target_arch = "xtensa")]
    let flash = flash.multicore_auto_park();

    *guard = Some(flash);
}

#[cfg(feature = "hardware")]
fn ble_store_checksum(store: &BleBondStore) -> u32 {
    let mut sum = store.magic
        ^ ((store.version as u32) << 24)
        ^ ((store.count as u32) << 16)
        ^ ((store.reserved[0] as u32) << 8)
        ^ (store.reserved[1] as u32);
    for &len in &store.lens {
        sum = sum.rotate_left(5) ^ (len as u32);
    }
    for row in &store.ids {
        for &b in row {
            sum = sum.rotate_left(5) ^ (b as u32);
        }
    }
    sum
}

#[cfg(feature = "hardware")]
fn ble_store_is_valid(store: &BleBondStore) -> bool {
    if store.magic != BLE_STORE_MAGIC || store.version != BLE_STORE_VERSION {
        return false;
    }
    if store.count as usize > BLE_STORE_MAX_PHONES {
        return false;
    }
    for &len in store.lens.iter().take(store.count as usize) {
        if len as usize > BLE_STORE_MAX_ID_LEN {
            return false;
        }
    }
    ble_store_checksum(store) == store.checksum
}

#[cfg(feature = "hardware")]
pub(super) async fn restore_paired_phones_from_store() -> usize {
    let _guard = BLE_BOND_STORE_LOCK.lock().await;
    let mut flash_guard = BLE_FLASH_STORE.lock().await;
    let Some(flash) = flash_guard.as_mut() else {
        log::warn!("BLE store: flash store not initialized; skipping restore");
        return 0;
    };

    let mut sector = [0xFFu8; FlashStorage::SECTOR_SIZE as usize];
    let offset = ble_store_flash_offset(flash.capacity());
    if let Err(e) = ReadStorage::read(flash, offset, &mut sector) {
        log::warn!("BLE store: read failed at 0x{:x}: {:?}", offset, e);
        return 0;
    }

    let Some(store) = ble_store_read_bytes(&sector) else {
        return 0;
    };
    if !ble_store_is_valid(&store) {
        log::warn!("BLE store: invalid on-flash payload, ignoring");
        return 0;
    }

    let mut restored = 0usize;
    for i in 0..(store.count as usize) {
        let len = store.lens[i] as usize;
        if len == 0 {
            continue;
        }
        let id = &store.ids[i][..len];
        if core_interface::add_paired_phone(id).await {
            restored += 1;
        }
    }
    restored
}

#[cfg(feature = "hardware")]
pub(crate) async fn persist_paired_phones_to_store() {
    let phones = core_interface::list_paired_phones().await;

    let mut store = BleBondStore::empty();
    let max = core::cmp::min(BLE_STORE_MAX_PHONES, phones.len());
    store.count = max as u8;
    for (i, id) in phones.iter().take(max).enumerate() {
        let n = core::cmp::min(BLE_STORE_MAX_ID_LEN, id.len());
        store.lens[i] = n as u8;
        store.ids[i][..n].copy_from_slice(&id[..n]);
    }
    store.checksum = ble_store_checksum(&store);

    let _guard = BLE_BOND_STORE_LOCK.lock().await;
    let mut flash_guard = BLE_FLASH_STORE.lock().await;
    let Some(flash) = flash_guard.as_mut() else {
        log::warn!("BLE store: flash store not initialized; skipping persist");
        return;
    };

    let offset = ble_store_flash_offset(flash.capacity());
    let mut sector = [0xFFu8; FlashStorage::SECTOR_SIZE as usize];
    ble_store_write_bytes(&store, &mut sector);
    if let Err(e) = Storage::write(flash, offset, &sector) {
        log::warn!("BLE store: write failed at 0x{:x}: {:?}", offset, e);
    }
}
