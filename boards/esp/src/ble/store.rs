//! Flash-backed BLE bond store.
//!
//! Persists `BondInformation` records (LTK, identity address, IRK) to a dedicated
//! flash range using `sequential-storage`. On startup, bonds are restored and fed
//! back to the `trouble-host` stack so paired phones can re-encrypt without
//! triggering a full re-pairing procedure.
//!
//! Storage layout: the store occupies the last two NOR flash sectors (fixed end-of-
//! flash regions — see docs/ble.md for the planned migration to named partitions).

use sequential_storage::map::{SerializationError, Value};
use trouble_host::connection::SecurityLevel;
use trouble_host::prelude::{BdAddr, BondInformation, Identity, IdentityResolvingKey, LongTermKey};

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
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) struct StoredBond(pub BondInformation);

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
mod hardware {
    use core::ops::Range;
    use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
    use sequential_storage::{
        cache::NoCache,
        map::{fetch_all_items, remove_item, store_item},
    };
    use trouble_host::prelude::{BdAddr, BondInformation};

    use super::StoredBond;

    /// Wraps a flash NorFlash storage backend and a flash range, and provides
    /// async load / store / remove operations for `BondInformation` records.
    /// Each bond is keyed by the peer's 6-byte Bluetooth device address.
    pub(in crate::ble) struct BondStore<'a, S: NorFlash + MultiwriteNorFlash> {
        storage: &'a mut S,
        range: Range<u32>,
    }

    impl<'a, S: NorFlash + MultiwriteNorFlash> BondStore<'a, S> {
        pub(in crate::ble) fn new(storage: &'a mut S, range: Range<u32>) -> Self {
            Self { storage, range }
        }

        /// Load all stored bonds from flash. Returns an empty vec on any flash error.
        pub(in crate::ble) async fn load_bonds(&mut self) -> heapless::Vec<BondInformation, 8> {
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
        pub(in crate::ble) async fn store_bond(
            &mut self,
            bond: &BondInformation,
        ) -> Result<(), ()> {
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
        pub(in crate::ble) async fn remove_bond(&mut self, peer_addr: &BdAddr) -> Result<(), ()> {
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
}

#[cfg(feature = "hardware")]
pub(in crate::ble) use hardware::*;

#[cfg(test)]
mod tests {
    use super::*;

    fn bond(irk: Option<u128>, security_level: SecurityLevel) -> BondInformation {
        BondInformation {
            ltk: LongTermKey::new(0x0011_2233_4455_6677_8899_AABB_CCDD_EEFF),
            identity: Identity {
                bd_addr: BdAddr::new([0x01, 0x02, 0x03, 0x04, 0x05, 0xC6]),
                irk: irk.map(IdentityResolvingKey::new),
            },
            is_bonded: true,
            security_level,
        }
    }

    fn serialize(bond: BondInformation) -> [u8; 41] {
        let mut buf = [0u8; 41];
        assert_eq!(StoredBond(bond).serialize_into(&mut buf), Ok(41));
        buf
    }

    /// Pins the on-flash layout: changing it would make devices forget their
    /// bonded phones after a firmware update.
    #[test]
    fn serialized_layout_matches_documented_format() {
        let buf = serialize(bond(Some(0xFF), SecurityLevel::EncryptedAuthenticated));
        assert_eq!(
            buf[0..16],
            0x0011_2233_4455_6677_8899_AABB_CCDD_EEFFu128.to_le_bytes()
        );
        assert_eq!(buf[16..22], [0x01, 0x02, 0x03, 0x04, 0x05, 0xC6]);
        assert_eq!(buf[22], 1);
        assert_eq!(buf[23..39], 0xFFu128.to_le_bytes());
        assert_eq!(buf[39], 1);
        assert_eq!(buf[40], 2);
    }

    #[test]
    fn round_trips_with_and_without_irk() {
        for (irk, level) in [
            (Some(0x1234_5678u128), SecurityLevel::Encrypted),
            (None, SecurityLevel::NoEncryption),
            (None, SecurityLevel::EncryptedAuthenticated),
        ] {
            let bytes = serialize(bond(irk, level));
            let StoredBond(restored) = StoredBond::deserialize_from(&bytes).unwrap();
            assert_eq!(restored.ltk.0, 0x0011_2233_4455_6677_8899_AABB_CCDD_EEFF);
            assert_eq!(
                restored.identity.bd_addr.raw(),
                [0x01, 0x02, 0x03, 0x04, 0x05, 0xC6]
            );
            assert_eq!(restored.identity.irk.map(|k| k.0), irk);
            assert!(restored.is_bonded);
            assert_eq!(serialize(restored), bytes);
        }
    }

    #[test]
    fn short_buffers_are_rejected() {
        let mut small = [0u8; 40];
        assert_eq!(
            StoredBond(bond(None, SecurityLevel::Encrypted)).serialize_into(&mut small),
            Err(SerializationError::BufferTooSmall)
        );
        assert!(matches!(
            StoredBond::deserialize_from(&small),
            Err(SerializationError::BufferTooSmall)
        ));
    }
}
