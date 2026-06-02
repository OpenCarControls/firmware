// BLE transport constants generated from contracts/opencar/core/v1/transport.toml
// and pure utility functions shared by all boards that implement BLE.
include!(concat!(env!("OUT_DIR"), "/ble_transport.rs"));

/// Convert a 6-byte MAC address into a BLE static random address.
///
/// The byte order is reversed (MAC is big-endian; BLE address is little-endian)
/// and the two MSBs of the most-significant octet are forced to `1` as required
/// by the Bluetooth specification for static random addresses.
pub const fn mac_to_ble_address(mac: [u8; 6]) -> [u8; 6] {
    [mac[5], mac[4], mac[3], mac[2], mac[1], mac[0] | 0xC0]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mac_to_ble_address_sets_static_random_bits() {
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let addr = mac_to_ble_address(mac);
        assert_eq!(addr[0], 0xFF);
        assert_eq!(addr[1], 0xEE);
        assert_eq!(addr[2], 0xDD);
        assert_eq!(addr[3], 0xCC);
        assert_eq!(addr[4], 0xBB);
        assert_eq!(addr[5], 0xAA | 0xC0);
        assert_eq!(addr[5] & 0xC0, 0xC0, "static-random bits not set");
    }

    #[test]
    fn mac_to_ble_address_all_zeros_still_sets_random_bits() {
        let addr = mac_to_ble_address([0x00; 6]);
        assert_eq!(addr[5] & 0xC0, 0xC0);
        assert_eq!(&addr[0..5], &[0x00; 5]);
    }

    #[test]
    fn gatt_uuids_are_16_bytes_and_distinct() {
        let svc: [u8; 16] = GATT_SERVICE_UUID.to_le_bytes();
        let rx: [u8; 16] = GATT_RX_UUID.to_le_bytes();
        let tx: [u8; 16] = GATT_TX_UUID.to_le_bytes();
        assert_eq!(svc.len(), 16);
        assert_ne!(svc, rx);
        assert_ne!(svc, tx);
        assert_ne!(rx, tx);
    }

    #[test]
    fn gatt_uuids_match_transport_contract() {
        // Values must stay in sync with contracts/opencar/core/v1/transport.toml.
        // If this fails, the transport.toml UUID was changed without a rebuild — or the
        // build.rs codegen is broken.
        assert_eq!(
            GATT_SERVICE_UUID,
            0x1acff001_229b_4d38_a5d2_9af1d9b11f00_u128
        );
        assert_eq!(GATT_RX_UUID, 0x1acff002_229b_4d38_a5d2_9af1d9b11f00_u128);
        assert_eq!(GATT_TX_UUID, 0x1acff003_229b_4d38_a5d2_9af1d9b11f00_u128);
    }
}
