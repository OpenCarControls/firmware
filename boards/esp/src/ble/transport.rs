#[cfg(feature = "hardware")]
use alloc::vec::Vec;

#[cfg(feature = "hardware")]
use core_interface::{BLE_RX_CHANNEL, BLE_TX_CHANNEL, proto};
#[cfg(feature = "hardware")]
use embassy_futures::{join::join, select::select};
#[cfg(feature = "hardware")]
use esp_hal::peripherals;
#[cfg(feature = "hardware")]
use esp_radio::ble;
#[cfg(feature = "hardware")]
use prost::Message as _;
#[cfg(feature = "hardware")]
use trouble_host::att::AttErrorCode;
#[cfg(feature = "hardware")]
use trouble_host::prelude::*;

#[cfg(feature = "hardware")]
use crate::network::SharedRadioController;

#[cfg(feature = "hardware")]
use super::store::{
    init_paired_phones_flash_store, persist_paired_phones_to_store, restore_paired_phones_from_store,
};
#[cfg(feature = "hardware")]
use super::{GATT_RX_UUID, GATT_SERVICE_UUID, GATT_TX_UUID, mac_to_ble_address};

#[cfg(feature = "hardware")]
const BLE_CONNECTIONS_MAX: usize = 1;
#[cfg(feature = "hardware")]
const BLE_L2CAP_CHANNELS_MAX: usize = 3;
#[cfg(feature = "hardware")]
const BLE_REPAIR_WINDOW_S: u32 = 30;

#[cfg(feature = "hardware")]
#[gatt_server]
struct OpenCarGattServer {
    link: OpenCarGattService,
}

#[cfg(feature = "hardware")]
#[gatt_service(uuid = GATT_SERVICE_UUID)]
struct OpenCarGattService {
    #[characteristic(uuid = GATT_RX_UUID, write, write_without_response)]
    rx: heapless::Vec<u8, 244>,
    #[characteristic(uuid = GATT_TX_UUID, notify)]
    tx: heapless::Vec<u8, 244>,
}

#[cfg(feature = "hardware")]
fn build_ble_device_name(base: &str, mac: [u8; 6]) -> heapless::String<33> {
    use core::fmt::Write as _;

    let mut out = heapless::String::<33>::new();
    let _ = write!(&mut out, "{}-{:02X}{:02X}", base, mac[4], mac[5]);
    if out.is_empty() {
        let _ = write!(&mut out, "OpenCar-{:02X}{:02X}", mac[4], mac[5]);
    }
    out
}

#[cfg(all(test, not(feature = "hardware")))]
fn build_ble_device_name(base: &str, mac: [u8; 6]) -> std::string::String {
    use core::fmt::Write as _;

    let mut out = std::string::String::new();
    let _ = write!(&mut out, "{}-{:02X}{:02X}", base, mac[4], mac[5]);
    out
}

#[cfg(feature = "hardware")]
#[embassy_executor::task]
pub async fn ble_transport_task(
    radio: &'static SharedRadioController,
    bt_peri: peripherals::BT<'static>,
    flash_peri: peripherals::FLASH<'static>,
    name_base: &'static str,
) {
    use static_cell::StaticCell;

    static BLE_DEVICE_NAME: StaticCell<heapless::String<33>> = StaticCell::new();

    init_paired_phones_flash_store(flash_peri).await;
    let restored = restore_paired_phones_from_store().await;
    if restored > 0 {
        log::info!("BLE store: restored {} paired phone(s) from flash", restored);
    }

    let connector = match ble::controller::BleConnector::new(radio, bt_peri, ble::Config::default())
    {
        Ok(c) => c,
        Err(_) => {
            log::error!("BLE transport: failed to initialize BLE connector");
            return;
        }
    };

    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let mut resources: HostResources<
        DefaultPacketPool,
        BLE_CONNECTIONS_MAX,
        BLE_L2CAP_CHANNELS_MAX,
    > = HostResources::new();
    let mac = esp_hal::efuse::Efuse::read_base_mac_address();
    let name = BLE_DEVICE_NAME.init(build_ble_device_name(name_base, mac));
    let name = name.as_str();
    let address: Address = Address::random(mac_to_ble_address(mac));

    let host = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = host.build();

    let server = OpenCarGattServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name,
        appearance: &appearance::computer::GENERIC_COMPUTER,
    }))
    .unwrap();

    log::info!("BLE transport: GATT host initialized, starting advertise loop");

    let _ = join(
        async {
            loop {
                if let Err(e) = runner.run().await {
                    log::warn!("BLE runner stopped: {:?}", e);
                }
            }
        },
        async {
            loop {
                match advertise_and_accept(name, &mut peripheral, &server).await {
                    Ok(conn) => {
                        let peer_id = peer_device_id(&conn);
                        if core_interface::is_phone_paired(&peer_id).await {
                            core_interface::open_pairing_window_for(BLE_REPAIR_WINDOW_S);
                            log::info!(
                                "BLE transport: known peer reconnected, opened {}s pairing window",
                                BLE_REPAIR_WINDOW_S
                            );
                        }
                        let pairing_open = core_interface::is_pairing_window_open();
                        if let Err(e) = conn.raw().set_bondable(pairing_open) {
                            log::warn!("BLE set_bondable({}) failed: {:?}", pairing_open, e);
                        }
                        log::info!("BLE transport: central connected");
                        let _ = select(
                            gatt_event_task(&server, &conn),
                            ble_tx_notify_task(&server, &conn),
                        )
                        .await;
                        log::info!("BLE transport: connection closed, returning to advertise");
                    }
                    Err(e) => {
                        log::warn!("BLE advertise/accept failed: {:?}", e);
                        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
                    }
                }
            }
        },
    )
    .await;
}

#[cfg(feature = "hardware")]
async fn advertise_and_accept<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server OpenCarGattServer<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut adv_data,
    )?;

    let svc_uuid_bytes: [u8; 16] = GATT_SERVICE_UUID.to_le_bytes();
    let mut scan_data = [0u8; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::ServiceUuids128(&[svc_uuid_bytes])],
        &mut scan_data,
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..adv_len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;

    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

#[cfg(feature = "hardware")]
fn link_is_encrypted<P: PacketPool>(conn: &GattConnection<'_, '_, P>) -> bool {
    match conn.raw().security_level() {
        Ok(level) => level.encrypted(),
        Err(e) => {
            log::warn!("BLE security level query failed: {:?}", e);
            false
        }
    }
}

#[cfg(feature = "hardware")]
fn request_link_security<P: PacketPool>(conn: &GattConnection<'_, '_, P>) {
    if let Err(e) = conn.raw().request_security() {
        log::warn!("BLE request_security failed: {:?}", e);
    }
}

#[cfg(feature = "hardware")]
fn sync_bondable_with_window<P: PacketPool>(conn: &GattConnection<'_, '_, P>) {
    let desired = core_interface::is_pairing_window_open();
    match conn.raw().bondable() {
        Ok(current) if current == desired => {}
        Ok(_) => {
            if let Err(e) = conn.raw().set_bondable(desired) {
                log::warn!("BLE set_bondable({}) failed: {:?}", desired, e);
            }
        }
        Err(e) => {
            log::warn!("BLE bondable() query failed: {:?}", e);
        }
    }
}

#[cfg(feature = "hardware")]
fn peer_device_id<P: PacketPool>(conn: &GattConnection<'_, '_, P>) -> Vec<u8> {
    let identity = conn.raw().peer_identity();
    let mut id = Vec::with_capacity(10);
    id.extend_from_slice(b"ble:");
    id.extend_from_slice(identity.bd_addr.raw());
    id
}

#[cfg(feature = "hardware")]
async fn gatt_event_task<P: PacketPool>(
    server: &OpenCarGattServer<'_>,
    conn: &GattConnection<'_, '_, P>,
) {
    let rx_handle = server.link.rx.handle;

    loop {
        sync_bondable_with_window(conn);
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                log::info!("BLE GATT disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::PassKeyDisplay(key) => {
                log::info!("BLE pairing passkey: {:06}", key.value());
            }
            GattConnectionEvent::PassKeyConfirm(key) => {
                if core_interface::is_pairing_window_open() {
                    if let Err(e) = conn.pass_key_confirm() {
                        log::warn!("BLE passkey confirm failed for {:06}: {:?}", key.value(), e);
                    }
                } else if let Err(e) = conn.pass_key_cancel() {
                    log::warn!("BLE passkey cancel failed for {:06}: {:?}", key.value(), e);
                }
            }
            GattConnectionEvent::PassKeyInput => {
                if let Err(e) = conn.pass_key_cancel() {
                    log::warn!("BLE passkey input not supported, cancel failed: {:?}", e);
                }
            }
            GattConnectionEvent::PairingComplete {
                security_level,
                bond,
            } => {
                let device_id = peer_device_id(conn);
                let added_or_existing = core_interface::add_paired_phone(&device_id).await;
                if added_or_existing {
                    persist_paired_phones_to_store().await;
                }
                log::info!(
                    "BLE pairing complete: encrypted={}, authenticated={}, bonded={}, paired_state_ok={}",
                    security_level.encrypted(),
                    security_level.authenticated(),
                    bond.is_some(),
                    added_or_existing,
                );
            }
            GattConnectionEvent::PairingFailed(e) => {
                log::warn!("BLE pairing failed: {:?}", e);
            }
            GattConnectionEvent::Gatt { event } => match event {
                GattEvent::Write(write_evt) if write_evt.handle() == rx_handle => {
                    if !link_is_encrypted(conn) {
                        request_link_security(conn);
                        if let Ok(reply) = write_evt.reject(AttErrorCode::INSUFFICIENT_ENCRYPTION) {
                            let _ = reply.send().await;
                        }
                        continue;
                    }

                    let device_id = peer_device_id(conn);
                    let pairing_open = core_interface::is_pairing_window_open();
                    let is_paired = core_interface::is_phone_paired(&device_id).await;
                    if !pairing_open && !is_paired {
                        if let Ok(reply) =
                            write_evt.reject(AttErrorCode::INSUFFICIENT_AUTHORISATION)
                        {
                            let _ = reply.send().await;
                        }
                        continue;
                    }

                    let payload = write_evt.data().to_vec();
                    if let Ok(reply) = write_evt.accept() {
                        let _ = reply.send().await;
                    }

                    match proto::AppToDevice::decode(payload.as_slice()) {
                        Ok(mut msg) => {
                            msg.source_device_id = device_id;
                            BLE_RX_CHANNEL.send(msg).await;
                        }
                        Err(e) => {
                            log::warn!("BLE GATT RX decode failed: {:?}", e);
                        }
                    }
                }
                other => {
                    if let Ok(reply) = other.accept() {
                        let _ = reply.send().await;
                    }
                }
            },
            _ => {}
        }
    }
}

#[cfg(feature = "hardware")]
async fn ble_tx_notify_task<P: PacketPool>(
    server: &OpenCarGattServer<'_>,
    conn: &GattConnection<'_, '_, P>,
) {
    let mut pending_security_request = false;
    let mut pending_msg: Option<proto::DeviceToApp> = None;

    loop {
        let msg = if let Some(msg) = pending_msg.take() {
            msg
        } else {
            BLE_TX_CHANNEL.receive().await
        };
        sync_bondable_with_window(conn);

        if !link_is_encrypted(conn) {
            if !pending_security_request {
                request_link_security(conn);
                pending_security_request = true;
            }
            pending_msg = Some(msg);
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
            continue;
        }
        pending_security_request = false;

        let mut encoded = Vec::<u8>::new();
        if msg.encode(&mut encoded).is_err() {
            log::warn!("BLE GATT TX encode failed");
            continue;
        }

        let payload = match heapless::Vec::<u8, 244>::from_slice(encoded.as_slice()) {
            Ok(v) => v,
            Err(_) => {
                log::warn!("BLE GATT TX message too large: {} bytes", encoded.len());
                continue;
            }
        };

        if server.link.tx.notify(conn, &payload).await.is_err() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ble_device_name_appends_mac_suffix() {
        let mac = [0x10, 0x20, 0x30, 0x40, 0xAB, 0xCD];
        let name = build_ble_device_name("OpenCar", mac);
        assert_eq!(name.as_str(), "OpenCar-ABCD");
    }
}
