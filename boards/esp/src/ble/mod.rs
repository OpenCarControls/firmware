mod lifecycle;
mod store;
mod transport;

#[cfg(feature = "hardware")]
pub use lifecycle::ble_lifecycle_task;
#[cfg(feature = "hardware")]
pub(crate) use store::persist_paired_phones_to_store;
#[cfg(feature = "hardware")]
pub use transport::ble_transport_task;

#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) const GATT_SERVICE_UUID: u128 = 0x1acff001_229b_4d38_a5d2_9af1d9b11f00;
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) const GATT_RX_UUID: u128 = 0x1acff002_229b_4d38_a5d2_9af1d9b11f00;
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) const GATT_TX_UUID: u128 = 0x1acff003_229b_4d38_a5d2_9af1d9b11f00;

#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) const fn mac_to_ble_address(mac: [u8; 6]) -> [u8; 6] {
    [mac[5], mac[4], mac[3], mac[2], mac[1], mac[0] | 0xC0]
}

#[cfg(test)]
mod tests {
    use super::*;
    use core_interface::proto;
    use prost::Message as _;

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
    fn ble_rx_empty_payload_fits_characteristic() {
        let msg = proto::AppToDevice::default();
        assert!(msg.encoded_len() <= 244);
    }

    #[test]
    fn ble_tx_state_payload_fits_characteristic() {
        let msg = proto::DeviceToApp {
            timestamp_ms: u64::MAX,
            platform_id: u32::MAX,
            payload: Some(proto::device_to_app::Payload::StateUpdate(
                proto::StateUpdate {
                    system_state: None,
                    vehicle_state: Some(proto::VehicleState {
                        basic_state_bytes: vec![0xAB; 100],
                        advanced_state_bytes: vec![0xCD; 100],
                    }),
                },
            )),
        };
        assert!(
            msg.encoded_len() <= 244,
            "encoded_len {} > 244",
            msg.encoded_len()
        );
    }

    #[test]
    fn ble_rx_proto_roundtrip() {
        let original = proto::AppToDevice {
            message_id: 42,
            platform_id: 0xDEAD_BEEF,
            source_device_id: vec![7u8],
            payload: Some(proto::app_to_device::Payload::BasicCommandBytes(vec![
                0x01, 0x02, 0x03,
            ])),
        };
        let mut buf = Vec::new();
        original.encode(&mut buf).unwrap();
        let decoded = proto::AppToDevice::decode(buf.as_slice()).unwrap();
        assert_eq!(decoded.message_id, 42);
        assert_eq!(decoded.platform_id, 0xDEAD_BEEF);
        assert_eq!(decoded.source_device_id, vec![7u8]);
        match decoded.payload {
            Some(proto::app_to_device::Payload::BasicCommandBytes(b)) => {
                assert_eq!(b, vec![0x01, 0x02, 0x03])
            }
            other => panic!("unexpected payload: {other:?}"),
        }
    }

    #[test]
    fn ble_tx_proto_roundtrip() {
        let original = proto::DeviceToApp {
            timestamp_ms: 9_999_999,
            platform_id: 0xF7544D7E,
            payload: Some(proto::device_to_app::Payload::StateUpdate(
                proto::StateUpdate {
                    system_state: None,
                    vehicle_state: Some(proto::VehicleState {
                        basic_state_bytes: vec![1, 2, 3],
                        advanced_state_bytes: vec![4, 5, 6],
                    }),
                },
            )),
        };
        let mut buf = Vec::new();
        original.encode(&mut buf).unwrap();
        let decoded = proto::DeviceToApp::decode(buf.as_slice()).unwrap();
        assert_eq!(decoded.timestamp_ms, 9_999_999);
        assert_eq!(decoded.platform_id, 0xF7544D7E);
        match decoded.payload {
            Some(proto::device_to_app::Payload::StateUpdate(u)) => {
                let vs = u.vehicle_state.unwrap();
                assert_eq!(vs.basic_state_bytes, vec![1, 2, 3]);
                assert_eq!(vs.advanced_state_bytes, vec![4, 5, 6]);
            }
            other => panic!("unexpected payload: {other:?}"),
        }
    }
}
