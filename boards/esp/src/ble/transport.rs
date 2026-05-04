#[cfg(feature = "hardware")]
use alloc::vec::Vec;

#[cfg(feature = "hardware")]
use core_interface::{BLE_RX_CHANNEL, BLE_TX_CHANNEL, proto};
#[cfg(feature = "hardware")]
use embassy_futures::{join::join, select::{select, Either}};
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
    init_paired_phones_flash_store, persist_paired_phones_to_store, persist_security_store,
    restore_paired_phones_from_store, restore_security_store,
};
#[cfg(feature = "hardware")]
use core_interface::ble::{GATT_RX_UUID, GATT_SERVICE_UUID, GATT_TX_UUID, mac_to_ble_address};

#[cfg(feature = "hardware")]
const BLE_CONNECTIONS_MAX: usize = 1;
#[cfg(feature = "hardware")]
const BLE_L2CAP_CHANNELS_MAX: usize = 3;

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
    rng_peri: peripherals::RNG<'static>,
    adc1_peri: peripherals::ADC1<'static>,
    name_base: &'static str,
) {
    use static_cell::StaticCell;

    static BLE_DEVICE_NAME: StaticCell<heapless::String<33>> = StaticCell::new();

    init_paired_phones_flash_store(flash_peri).await;
    let restored = restore_paired_phones_from_store().await;
    if restored > 0 {
        log::info!("BLE store: restored {} paired phone(s) from flash", restored);
    }

    let saved_bonds = restore_security_store().await;
    log::info!("BLE security store: restored {} LTK(s) from flash", saved_bonds.len());

    let connector = match ble::controller::BleConnector::new(radio, bt_peri, ble::Config::default())
    {
        Ok(c) => c,
        Err(_) => {
            log::error!("BLE transport: failed to initialize BLE connector");
            return;
        }
    };

    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    static HOST_RESOURCES: StaticCell<
        HostResources<DefaultPacketPool, BLE_CONNECTIONS_MAX, BLE_L2CAP_CHANNELS_MAX>,
    > = StaticCell::new();
    let resources = HOST_RESOURCES.init(HostResources::new());
    let mac = esp_hal::efuse::Efuse::read_base_mac_address();
    let name = BLE_DEVICE_NAME.init(build_ble_device_name(name_base, mac));
    let name = name.as_str();
    let address: Address = Address::random(mac_to_ble_address(mac));

    let _trng_source = esp_hal::rng::TrngSource::new(rng_peri, adc1_peri);
    let mut trng = esp_hal::rng::Trng::try_new().expect("TRNG entropy source should be active");
    let host = trouble_host::new(controller, resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut trng);

    for bond in &saved_bonds {
        if let Err(e) = host.add_bond_information(bond.clone()) {
            log::warn!("BLE security store: failed to restore LTK: {:?}", e);
        }
    }

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
            let mut last_pairing_open = core_interface::is_pairing_window_open();
            loop {
                let pairing_open = core_interface::is_pairing_window_open();
                if pairing_open != last_pairing_open {
                    if pairing_open {
                        log::info!("BLE: pairing window opened — switching to discoverable advertising");
                    } else {
                        log::info!("BLE: pairing window closed — switching to non-discoverable advertising");
                    }
                    last_pairing_open = pairing_open;
                }
                match advertise_and_accept(name, pairing_open, &mut peripheral, &server).await {
                    Ok(Some(conn)) => {
                        if let Err(e) = conn.raw().set_bondable(pairing_open) {
                            log::warn!("BLE set_bondable({}) failed: {:?}", pairing_open, e);
                        }
                        let peer_addr = conn.raw().peer_identity().bd_addr;
                        log::info!(
                            "BLE transport: central connected peer={:02X?} (pairing_window={})",
                            peer_addr.raw(),
                            pairing_open
                        );
                        // tx_auth starts false; gatt_event_task sets it to true once the
                        // peer is confirmed as paired or bonded (synchronously at task
                        // startup for preexisting bonds, or on PairingComplete for new ones).
                        let tx_auth = core::sync::atomic::AtomicBool::new(false);
                        let _ = select(
                            gatt_event_task(&server, &conn, &host, &tx_auth),
                            ble_tx_notify_task(&server, &conn, &tx_auth),
                        )
                        .await;
                        log::info!("BLE transport: connection closed, returning to advertise");
                    }
                    Ok(None) => {
                        // Advertise timeout — re-check pairing window mode on next iteration.
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
    pairing_window_open: bool,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server OpenCarGattServer<'values>,
) -> Result<Option<GattConnection<'values, 'server, DefaultPacketPool>>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let mut scan_data = [0u8; 31];

    let (adv_len, scan_len) = if pairing_window_open {
        // Mode A: discoverable — new phones can find and pair with the device.
        let svc_uuid_bytes: [u8; 16] = GATT_SERVICE_UUID.to_le_bytes();
        let adv_len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::CompleteLocalName(name.as_bytes()),
            ],
            &mut adv_data,
        )?;
        let scan_len = AdStructure::encode_slice(
            &[AdStructure::ServiceUuids128(&[svc_uuid_bytes])],
            &mut scan_data,
        )?;
        (adv_len, scan_len)
    } else {
        // Mode B: non-discoverable — device does not appear in general BLE scans.
        // Bonded phones that know the device address can still reconnect.
        let adv_len = AdStructure::encode_slice(
            &[AdStructure::Flags(BR_EDR_NOT_SUPPORTED)],
            &mut adv_data,
        )?;
        (adv_len, 0)
    };

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..adv_len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;

    // Apply a timeout so the outer loop can re-check the pairing window state
    // and switch advertising modes if needed (within ~10 seconds of any change).
    match select(
        advertiser.accept(),
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)),
    )
    .await
    {
        Either::First(result) => Ok(Some(result?.with_attribute_server(server)?)),
        Either::Second(_) => Ok(None),
    }
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
    // Keep bonding enabled until the link is encrypted so both peers can
    // exchange/store LTKs on first connect.
    let desired = if link_is_encrypted(conn) {
        core_interface::is_pairing_window_open()
    } else {
        true
    };
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
async fn gatt_event_task<C: Controller, P: PacketPool>(
    server: &OpenCarGattServer<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
    tx_auth: &core::sync::atomic::AtomicBool,
) {
    let rx_handle = server.link.rx.handle;
    let preexisting_bond_addrs: Vec<[u8; 6]> = stack
        .get_bond_information()
        .iter()
        .map(|bond| {
            let mut addr = [0u8; 6];
            addr.copy_from_slice(bond.identity.bd_addr.raw());
            addr
        })
        .collect();

    // If the peer already has a preexisting LTK bond, authorize TX immediately
    // so state updates flow without waiting for PairingComplete. Re-encryption
    // fires PairingComplete asynchronously; we cannot block TX until then or
    // the phone reconnect after BLE toggle would receive no state updates.
    {
        let mut peer_addr_at_connect = [0u8; 6];
        peer_addr_at_connect.copy_from_slice(conn.raw().peer_identity().bd_addr.raw());
        if preexisting_bond_addrs
            .iter()
            .any(|addr| *addr == peer_addr_at_connect)
        {
            log::info!(
                "BLE gatt: peer {:02X?} matched preexisting bond — tx_auth=true immediately ({} bond(s) in store)",
                peer_addr_at_connect,
                preexisting_bond_addrs.len()
            );
            tx_auth.store(true, core::sync::atomic::Ordering::Relaxed);
        } else {
            log::info!(
                "BLE gatt: peer {:02X?} NOT in preexisting bonds (store has {} entry/entries: {:02X?}) — awaiting PairingComplete",
                peer_addr_at_connect,
                preexisting_bond_addrs.len(),
                preexisting_bond_addrs
            );
        }
    }

    let mut verified_peer_id: Option<Vec<u8>> = None;

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
                // Phase 2: prefer the stable identity address from the key-distribution
                // PDU (bond.identity.bd_addr) over peer_identity().bd_addr, which may
                // still be the current RPA at the moment this event fires.
                let device_id = if let Some(ref b) = bond {
                    let mut id = Vec::with_capacity(10);
                    id.extend_from_slice(b"ble:");
                    id.extend_from_slice(b.identity.bd_addr.raw());
                    id
                } else {
                    // Re-encryption via existing LTK: no new keys exchanged, IRK
                    // resolution has already resolved peer_identity() to the stable
                    // identity address before this event fires.
                    peer_device_id(conn)
                };
                verified_peer_id = Some(device_id.clone());

                // Phase 1: also allow re-registration for preexisting-bonded phones
                // that reconnect outside the pairing window.
                let mut peer_addr = [0u8; 6];
                peer_addr.copy_from_slice(conn.raw().peer_identity().bd_addr.raw());
                let is_preexisting_bond =
                    preexisting_bond_addrs.iter().any(|addr| *addr == peer_addr);

                let pairing_open = core_interface::is_pairing_window_open();
                let already_known = core_interface::is_phone_paired(&device_id).await;
                let added_or_existing = if pairing_open || already_known || is_preexisting_bond {
                    core_interface::add_paired_phone(&device_id).await
                } else {
                    log::warn!(
                        "BLE pairing complete for unknown phone outside pairing window; not adding to registry"
                    );
                    false
                };
                if added_or_existing {
                    // Phase 3: authorize TX now that the phone is confirmed.
                    tx_auth.store(true, core::sync::atomic::Ordering::Relaxed);
                    persist_paired_phones_to_store().await;
                }
                if bond.is_some() {
                    let all_bonds = stack.get_bond_information();
                    persist_security_store(all_bonds.as_slice()).await;
                    log::info!("BLE security store: persisted {} LTK(s)", all_bonds.len());
                }
                log::info!(
                    "BLE pairing complete: encrypted={}, authenticated={}, bonded={}, pairing_open={}, already_known={}, preexisting_bond={}, paired_state_ok={}, device_id={:02X?}",
                    security_level.encrypted(),
                    security_level.authenticated(),
                    bond.is_some(),
                    pairing_open,
                    already_known,
                    is_preexisting_bond,
                    added_or_existing,
                    device_id,
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

                    let device_id = verified_peer_id
                        .clone()
                        .unwrap_or_else(|| peer_device_id(conn));
                    let pairing_open = core_interface::is_pairing_window_open();
                    let is_paired = core_interface::is_phone_paired(&device_id).await;
                    let mut peer_addr = [0u8; 6];
                    peer_addr.copy_from_slice(conn.raw().peer_identity().bd_addr.raw());
                    let is_preexisting_bond =
                        preexisting_bond_addrs.iter().any(|addr| *addr == peer_addr);
                    if !pairing_open && !is_paired && !is_preexisting_bond {
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
    tx_auth: &core::sync::atomic::AtomicBool,
) {
    let mut pending_security_request = false;
    let mut pending_msg: Option<proto::DeviceToApp> = None;
    let mut tx_held_logged = false;

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
            if !tx_held_logged {
                log::info!("BLE TX: holding message — link not yet encrypted, security requested");
                tx_held_logged = true;
            }
            pending_msg = Some(msg);
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
            continue;
        }
        pending_security_request = false;

        // Only send to phones that have been confirmed as paired or bonded.
        if !tx_auth.load(core::sync::atomic::Ordering::Relaxed) {
            if !tx_held_logged {
                log::info!("BLE TX: holding message — awaiting tx_auth (PairingComplete not yet received)");
                tx_held_logged = true;
            }
            pending_msg = Some(msg);
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
            continue;
        }
        tx_held_logged = false;

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
