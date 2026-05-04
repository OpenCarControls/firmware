mod lifecycle;
mod store;
mod transport;

#[cfg(feature = "hardware")]
pub use lifecycle::ble_lifecycle_task;
#[cfg(feature = "hardware")]
pub(crate) use store::persist_paired_phones_to_store;
#[cfg(feature = "hardware")]
pub use transport::ble_transport_task;

#[cfg(test)]
mod tests {
    use core_interface::proto;
    use prost::Message as _;

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
