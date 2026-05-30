//! Flash-backed BLE bond store.
//!
//! Persists `BondInformation` records (LTK, identity address, IRK) to a dedicated
//! flash range using `sequential-storage`. On startup, bonds are restored and fed
//! back to the `trouble-host` stack so paired phones can re-encrypt without
//! triggering a full re-pairing procedure.
//!
//! Storage layout: the store occupies the last two NOR flash sectors (fixed end-of-
//! flash regions — see AGENTS.md for the planned migration to named partitions).

#[cfg(feature = "hardware")]
use core::ops::Range;
#[cfg(feature = "hardware")]
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
#[cfg(feature = "hardware")]
use sequential_storage::{
    cache::NoCache,
    map::{SerializationError, Value, fetch_all_items, remove_item, store_item},
};
#[cfg(feature = "hardware")]
use trouble_host::connection::SecurityLevel;
#[cfg(feature = "hardware")]
use trouble_host::prelude::{BdAddr, BondInformation, Identity, IdentityResolvingKey, LongTermKey};

/// Wraps a flash NorFlash storage backend and a flash range, and provides
/// async load / store / remove operations for `BondInformation` records.
/// Each bond is keyed by the peer's 6-byte Bluetooth device address.
#[cfg(feature = "hardware")]
pub(super) struct BondStore<'a, S: NorFlash + MultiwriteNorFlash> {
    storage: &'a mut S,
    range: Range<u32>,
}

/// `BondInformation` wrapper that implements `sequential_storage::map::Value`
/// so it can be written/read directly to/from flash.
///
/// Byte layout (41 bytes total):
/// - [ 0..16] Long Term Key (LTK), little-endian u128
/// - [16..22] Bluetooth Device Address (6 bytes)
/// - [22]     IRK present flag (1 = yes, 0 = no)
/// - [23..39] Identity Resolving Key (IRK), little-endian u128
/// - [39]     is_bonded flag (1 = yes)
/// - [40]     SecurityLevel variant (0=NoEncryption, 1=Encrypted, 2=EncryptedAuthenticated)
#[cfg(feature = "hardware")]
pub(super) struct StoredBond(pub BondInformation);

#[cfg(feature = "hardware")]
impl<'a> Value<'a> for StoredBond {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 41 {
            return Err(SerializationError::BufferTooSmall);
        }
        let bond = &self.0;
        buffer[0..16].copy_from_slice(&bond.ltk.0.to_le_bytes());
        buffer[16..22].copy_from_slice(&bond.identity.bd_addr.0);
        if let Some(irk) = bond.identity.irk {
            buffer[22] = 1;
            buffer[23..39].copy_from_slice(&irk.0.to_le_bytes());
        } else {
            buffer[22] = 0;
            buffer[23..39].fill(0);
        }
        buffer[39] = if bond.is_bonded { 1 } else { 0 };
        buffer[40] = match bond.security_level {
            SecurityLevel::NoEncryption => 0,
            SecurityLevel::Encrypted => 1,
            SecurityLevel::EncryptedAuthenticated => 2,
        };
        Ok(41)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError> {
        if buffer.len() < 41 {
            return Err(SerializationError::BufferTooSmall);
        }
        let ltk = LongTermKey::new(u128::from_le_bytes(buffer[0..16].try_into().unwrap()));
        let mut bd_addr_bytes = [0u8; 6];
        bd_addr_bytes.copy_from_slice(&buffer[16..22]);
        let bd_addr = BdAddr::new(bd_addr_bytes);
        let irk = if buffer[22] == 1 {
            Some(IdentityResolvingKey::new(u128::from_le_bytes(
                buffer[23..39].try_into().unwrap(),
            )))
        } else {
            None
        };
        Ok(StoredBond(BondInformation {
            ltk,
            identity: Identity { bd_addr, irk },
            is_bonded: buffer[39] != 0,
            security_level: match buffer[40] {
                0 => SecurityLevel::NoEncryption,
                1 => SecurityLevel::Encrypted,
                _ => SecurityLevel::EncryptedAuthenticated,
            },
        }))
    }
}

#[cfg(feature = "hardware")]
impl<'a, S: NorFlash + MultiwriteNorFlash> BondStore<'a, S> {
    pub(super) fn new(storage: &'a mut S, range: Range<u32>) -> Self {
        Self { storage, range }
    }

    /// Load all stored bonds from flash. Returns an empty vec on any flash error.
    pub(super) async fn load_bonds(&mut self) -> heapless::Vec<BondInformation, 8> {
        let mut bonds = heapless::Vec::new();
        let mut data_buffer = [0u8; 128];
        let mut cache = NoCache::new();

        match fetch_all_items::<[u8; 6], _, _>(
            self.storage,
            self.range.clone(),
            &mut cache,
            &mut data_buffer,
        )
        .await
        {
            Ok(mut iter) => {
                while let Ok(Some((_key, bond))) =
                    iter.next::<[u8; 6], StoredBond>(&mut data_buffer).await
                {
                    if bonds.push(bond.0).is_err() {
                        log::warn!("BLE store: too many bonds; ignoring the rest");
                        break;
                    }
                }
            }
            Err(_) => {
                log::warn!("BLE store: load_bonds failed (erased or corrupt) — starting fresh");
            }
        }

        log::info!("BLE store: loaded {} bond(s)", bonds.len());
        bonds
    }

    /// Persist a bond to flash (overwrites any existing entry with the same address).
    pub(super) async fn store_bond(&mut self, bond: &BondInformation) -> Result<(), ()> {
        let mut key = [0u8; 6];
        key.copy_from_slice(&bond.identity.bd_addr.0);
        let mut data_buffer = [0u8; 128];

        store_item(
            self.storage,
            self.range.clone(),
            &mut NoCache::new(),
            &mut data_buffer,
            &key,
            &StoredBond(bond.clone()),
        )
        .await
        .map_err(|e| {
            log::error!("BLE store: store_bond failed: {:?}", e);
        })
    }

    /// Remove the bond for `peer_addr` from flash (no-op if not found).
    pub(super) async fn remove_bond(&mut self, peer_addr: &BdAddr) -> Result<(), ()> {
        let mut key = [0u8; 6];
        key.copy_from_slice(&peer_addr.0);
        let mut data_buffer = [0u8; 128];

        remove_item(
            self.storage,
            self.range.clone(),
            &mut NoCache::new(),
            &mut data_buffer,
            &key,
        )
        .await
        .map_err(|e| {
            log::error!("BLE store: remove_bond failed: {:?}", e);
        })
    }
}
