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

// ── BLE Security Store (LTK / BondInformation persistence) ───────────────────

#[cfg(feature = "hardware")]
use trouble_host::{BondInformation, IdentityResolvingKey, LongTermKey};
#[cfg(feature = "hardware")]
use trouble_host::connection::SecurityLevel;

/// Magic bytes for the security store sector: "OCSL"
#[cfg(feature = "hardware")]
const BLE_SECURITY_STORE_MAGIC: u32 = 0x4F43_534C;
#[cfg(feature = "hardware")]
const BLE_SECURITY_STORE_VERSION: u8 = 1;
/// Up to 10 bonds (matches `trouble-host`'s BI_COUNT).
#[cfg(feature = "hardware")]
const BLE_SECURITY_STORE_MAX: usize = 10;
/// Serialised size of one BondInformation entry (bytes):
///   bd_addr[6] + has_irk[1] + irk[16] + ltk[16] + is_bonded[1] + security_level[1] = 41
#[cfg(feature = "hardware")]
const BLE_SECURITY_ENTRY_SIZE: usize = 41;
#[cfg(feature = "hardware")]
const BLE_SECURITY_STORE_SECTORS: usize = 1;

#[cfg(feature = "hardware")]
#[repr(C)]
#[derive(Clone, Copy)]
struct BleSecurityStore {
    magic: u32,
    version: u8,
    count: u8,
    reserved: [u8; 2],
    entries: [[u8; BLE_SECURITY_ENTRY_SIZE]; BLE_SECURITY_STORE_MAX],
    checksum: u32,
}

#[cfg(feature = "hardware")]
impl BleSecurityStore {
    const fn empty() -> Self {
        Self {
            magic: BLE_SECURITY_STORE_MAGIC,
            version: BLE_SECURITY_STORE_VERSION,
            count: 0,
            reserved: [0; 2],
            entries: [[0u8; BLE_SECURITY_ENTRY_SIZE]; BLE_SECURITY_STORE_MAX],
            checksum: 0,
        }
    }
}

/// The security store lives one sector before the phone-ID store at flash end.
#[cfg(feature = "hardware")]
fn ble_security_store_flash_offset(capacity: usize) -> u32 {
    let sector_size = FlashStorage::SECTOR_SIZE as usize;
    let reserve = sector_size * (BLE_STORE_FLASH_SECTORS + BLE_SECURITY_STORE_SECTORS);
    capacity.saturating_sub(reserve) as u32
}

#[cfg(feature = "hardware")]
fn ble_security_store_checksum(store: &BleSecurityStore) -> u32 {
    let mut sum = store.magic
        ^ ((store.version as u32) << 24)
        ^ ((store.count as u32) << 16)
        ^ ((store.reserved[0] as u32) << 8)
        ^ (store.reserved[1] as u32);
    for entry in &store.entries {
        for &b in entry {
            sum = sum.rotate_left(5) ^ (b as u32);
        }
    }
    sum
}

#[cfg(feature = "hardware")]
fn ble_security_store_is_valid(store: &BleSecurityStore) -> bool {
    if store.magic != BLE_SECURITY_STORE_MAGIC || store.version != BLE_SECURITY_STORE_VERSION {
        return false;
    }
    if store.count as usize > BLE_SECURITY_STORE_MAX {
        return false;
    }
    ble_security_store_checksum(store) == store.checksum
}

#[cfg(feature = "hardware")]
fn ble_security_store_write_bytes(store: &BleSecurityStore, out: &mut [u8]) {
    let n = core::mem::size_of::<BleSecurityStore>();
    if out.len() < n {
        return;
    }
    let src = store as *const BleSecurityStore as *const u8;
    // Safety: `store` is a valid repr(C) value; `out` is at least `n` bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(src, out.as_mut_ptr(), n);
    }
}

#[cfg(feature = "hardware")]
fn ble_security_store_read_bytes(src: &[u8]) -> Option<BleSecurityStore> {
    let n = core::mem::size_of::<BleSecurityStore>();
    if src.len() < n {
        return None;
    }
    if src[..n].iter().all(|b| *b == 0xFF) {
        return None;
    }
    let mut out = core::mem::MaybeUninit::<BleSecurityStore>::uninit();
    // Safety: destination is properly sized/aligned; source has at least n bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(src.as_ptr(), out.as_mut_ptr() as *mut u8, n);
        Some(out.assume_init())
    }
}

/// Encode `SecurityLevel` as a single byte.
#[cfg(feature = "hardware")]
fn security_level_to_u8(sl: SecurityLevel) -> u8 {
    match sl {
        SecurityLevel::NoEncryption => 0,
        SecurityLevel::Encrypted => 1,
        SecurityLevel::EncryptedAuthenticated => 2,
    }
}

/// Decode a single byte back into `SecurityLevel`.
#[cfg(feature = "hardware")]
fn u8_to_security_level(v: u8) -> SecurityLevel {
    match v {
        2 => SecurityLevel::EncryptedAuthenticated,
        _ => SecurityLevel::Encrypted,
    }
}

/// Serialise one `BondInformation` into a 41-byte array.
///
/// Layout:
///   [0..6]   bd_addr (little-endian raw bytes)
///   [6]      has_irk flag (0 or 1)
///   [7..23]  IRK as little-endian u128 (zeros if no IRK)
///   [23..39] LTK as little-endian u128
///   [39]     is_bonded flag (0 or 1)
///   [40]     SecurityLevel byte
#[cfg(feature = "hardware")]
fn bond_to_bytes(bond: &BondInformation) -> [u8; BLE_SECURITY_ENTRY_SIZE] {
    let mut entry = [0u8; BLE_SECURITY_ENTRY_SIZE];
    entry[0..6].copy_from_slice(bond.identity.bd_addr.raw());
    if let Some(irk) = bond.identity.irk {
        entry[6] = 1;
        entry[7..23].copy_from_slice(&irk.0.to_le_bytes());
    }
    entry[23..39].copy_from_slice(&bond.ltk.to_le_bytes());
    entry[39] = bond.is_bonded as u8;
    entry[40] = security_level_to_u8(bond.security_level);
    entry
}

/// Deserialise a 41-byte entry back into a `BondInformation`.
/// Returns `None` if the entry is all-zero (empty slot).
#[cfg(feature = "hardware")]
fn bytes_to_bond(entry: &[u8; BLE_SECURITY_ENTRY_SIZE]) -> Option<BondInformation> {
    if entry.iter().all(|&b| b == 0) {
        return None;
    }
    use trouble_host::prelude::BdAddr;
    let mut addr_bytes = [0u8; 6];
    addr_bytes.copy_from_slice(&entry[0..6]);
    let bd_addr = BdAddr::new(addr_bytes);

    let irk = if entry[6] != 0 {
        let mut irk_bytes = [0u8; 16];
        irk_bytes.copy_from_slice(&entry[7..23]);
        Some(IdentityResolvingKey(u128::from_le_bytes(irk_bytes)))
    } else {
        None
    };

    let mut ltk_bytes = [0u8; 16];
    ltk_bytes.copy_from_slice(&entry[23..39]);
    let ltk = LongTermKey::from_le_bytes(ltk_bytes);

    let is_bonded = entry[39] != 0;
    let security_level = u8_to_security_level(entry[40]);

    Some(BondInformation {
        identity: trouble_host::Identity { bd_addr, irk },
        ltk,
        is_bonded,
        security_level,
    })
}

/// Persist all current bonds to the security store sector.
#[cfg(feature = "hardware")]
pub(super) async fn persist_security_store(bonds: &[BondInformation]) {
    let mut store = BleSecurityStore::empty();
    let count = core::cmp::min(BLE_SECURITY_STORE_MAX, bonds.len());
    store.count = count as u8;
    for (i, bond) in bonds.iter().take(count).enumerate() {
        store.entries[i] = bond_to_bytes(bond);
    }
    store.checksum = ble_security_store_checksum(&store);

    let _guard = BLE_BOND_STORE_LOCK.lock().await;
    let mut flash_guard = BLE_FLASH_STORE.lock().await;
    let Some(flash) = flash_guard.as_mut() else {
        log::warn!("BLE security store: flash not initialized; skipping persist");
        return;
    };

    let offset = ble_security_store_flash_offset(flash.capacity());
    let mut sector = [0xFFu8; FlashStorage::SECTOR_SIZE as usize];
    ble_security_store_write_bytes(&store, &mut sector);
    if let Err(e) = Storage::write(flash, offset, &sector) {
        log::warn!("BLE security store: write failed at 0x{:x}: {:?}", offset, e);
    }
}

/// Restore bonds from the security store sector.
#[cfg(feature = "hardware")]
pub(super) async fn restore_security_store() -> heapless::Vec<BondInformation, BLE_SECURITY_STORE_MAX> {
    let _guard = BLE_BOND_STORE_LOCK.lock().await;
    let mut flash_guard = BLE_FLASH_STORE.lock().await;
    let Some(flash) = flash_guard.as_mut() else {
        log::warn!("BLE security store: flash not initialized; skipping restore");
        return heapless::Vec::new();
    };

    let mut sector = [0xFFu8; FlashStorage::SECTOR_SIZE as usize];
    let offset = ble_security_store_flash_offset(flash.capacity());
    if let Err(e) = ReadStorage::read(flash, offset, &mut sector) {
        log::warn!("BLE security store: read failed at 0x{:x}: {:?}", offset, e);
        return heapless::Vec::new();
    }

    let Some(store) = ble_security_store_read_bytes(&sector) else {
        return heapless::Vec::new();
    };
    if !ble_security_store_is_valid(&store) {
        log::warn!("BLE security store: invalid payload, ignoring");
        return heapless::Vec::new();
    }

    let mut out = heapless::Vec::new();
    for i in 0..(store.count as usize) {
        if let Some(bond) = bytes_to_bond(&store.entries[i]) {
            let _ = out.push(bond);
        }
    }
    out
}
