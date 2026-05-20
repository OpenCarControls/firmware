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
use bt_hci::cmd::le::{LeAddDeviceToFilterAcceptList, LeClearFilterAcceptList};

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
        appearance: &appearance::motorized_vehicle::CAR,
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
            // Start true so the first cycle always loads bonds restored from flash.
            let mut needs_list_update = true;
            // FilterConn is only safe when bonds with IRKs exist (so the controller
            // can resolve RPAs). Updated whenever bonds change.
            let mut can_hardware_filter = false;
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
                // Synchronise the controller's Filter Accept List and Resolving List
                // with the current bond store. Must be called while advertising is
                // inactive (BLE spec). FilterConn is only used when the Resolving
                // List is active (has_irk=true); otherwise all bonded phones would
                // be hardware-blocked because their RPAs cannot be resolved to the
                // identity addresses in the FAL without IRKs.
                // Only runs when bonds actually changed to avoid redundant HCI traffic.
                if needs_list_update {
                    let bonds = host.get_bond_information();
                    let (bond_count, has_irk) = update_controller_filter_lists(&host, bonds.as_slice()).await;
                    can_hardware_filter = bond_count > 0 && has_irk;
                    needs_list_update = false;
                }
                match advertise_and_accept(name, pairing_open, can_hardware_filter, &mut peripheral, &server).await {
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
                        // tx_auth is the single gate: gatt_event_task sets it to true
                        // only on PairingComplete (covers both new pairing and re-encryption
                        // of existing bonds). ble_tx_notify_task is a pure TX drainer that
                        // just waits on tx_auth and never touches security itself.
                        let tx_auth = core::sync::atomic::AtomicBool::new(false);
                        let phones_need_persist = core::cell::Cell::new(false);
                        let bonds_need_persist = core::cell::Cell::new(false);
                        let _ = select(
                            gatt_event_task(&server, &conn, &host, pairing_open, &tx_auth, &phones_need_persist, &bonds_need_persist),
                            ble_tx_notify_task(&server, &conn, &tx_auth),
                        )
                        .await;
                        log::info!("BLE transport: connection closed, returning to advertise");
                        // Persist to flash only after the BLE connection is closed.
                        // esp-storage disables ALL maskable interrupts during sector
                        // erase (~20-100 ms on ESP32-S3 QSPI flash). If called while
                        // a connection is active, the BLE controller's HCI events go
                        // unprocessed for the full erase duration, which drops the link.
                        if phones_need_persist.get() {
                            persist_paired_phones_to_store().await;
                        }
                        if bonds_need_persist.get() {
                            let all_bonds = host.get_bond_information();
                            persist_security_store(all_bonds.as_slice()).await;
                            log::info!("BLE security store: persisted {} LTK(s)", all_bonds.len());
                            // Bonds changed — update FAL and Resolving List on next cycle.
                            needs_list_update = true;
                        }
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
    // True only when every bonded phone's IRK is in the Resolving List so the
    // controller can resolve RPAs before the FAL check. When false, FilterConn
    // must NOT be used — bonded iOS/Android phones connect with an RPA that the
    // controller cannot match against the identity address in the FAL, causing a
    // silent hardware-level connection reject that appears as "Not bonded" on
    // the phone.
    can_hardware_filter: bool,
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
                AdStructure::Flags(AD_FLAG_LE_LIMITED_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
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

    // Mode B (window closed): FilterConn makes the controller reject connection
    // requests from any device not in the Filter Accept List, enforcing hardware-
    // level access control. This is safe ONLY when the Resolving List is active
    // (can_hardware_filter = true): without IRKs the controller cannot resolve the
    // phone's RPA to the identity address in the FAL and silently rejects the
    // connection at LL level — the phone sees "Not bonded" immediately.
    // When can_hardware_filter is false we fall back to Unfiltered so bonded phones
    // can still connect; non-discoverability (no name, no UUID) keeps Mode B quiet.
    let filter_policy = if pairing_window_open || !can_hardware_filter {
        AdvFilterPolicy::Unfiltered
    } else {
        AdvFilterPolicy::FilterConn
    };
    log::debug!(
        "BLE advertise: mode={} filter_policy={:?} (pairing_open={} can_hw_filter={})",
        if pairing_window_open { "A" } else { "B" },
        filter_policy,
        pairing_window_open,
        can_hardware_filter,
    );
    let adv_params = AdvertisementParameters {
        filter_policy,
        ..Default::default()
    };
    let advertiser = peripheral
        .advertise(
            &adv_params,
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
fn sync_bondable_with_window<P: PacketPool>(
    conn: &GattConnection<'_, '_, P>,
    pairing_open_at_connect: bool,
) {
    // Bondable=true only when the pairing window was open at connect time.
    // Re-encryption via an existing LTK is handled at the LL level and does not
    // depend on this flag. Once encrypted, follow the current window state so
    // a session that started in the window can't be used for bonding after it
    // closes (and vice-versa, for any edge case).
    let desired = if link_is_encrypted(conn) {
        core_interface::is_pairing_window_open()
    } else {
        pairing_open_at_connect
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
fn infer_addr_kind(bd_addr: &BdAddr) -> AddrKind {
    // BdAddr raw bytes are little-endian; raw()[5] is the most significant byte.
    // Random static addresses have the top two bits of the MSB set to 0b11 (mask 0xC0).
    if (bd_addr.raw()[5] & 0xC0) == 0xC0 {
        AddrKind::RANDOM
    } else {
        AddrKind::PUBLIC
    }
}

/// Synchronise the controller's Filter Accept List (FAL) and Resolving List with
/// the current bond store. Must be called while advertising is inactive — the BLE
/// spec requires advertising, scanning, and initiating to all be stopped before
/// modifying these lists. This is guaranteed because we call it between advertise
/// cycles in the outer loop.
///
/// Updates the controller's Filter Accept List with the current bond store.
/// The hardware Resolving List is intentionally NOT populated — keeping it empty
/// means the controller always reports the RPA in `LeEnhancedConnectionComplete`,
/// which is what the phone uses in LESC DHKey (f6) computation. Populating the
/// Resolving List would cause the controller to resolve the RPA to an identity
/// address, creating a mismatch with the phone's f6 and a silent DHKey Check
/// failure. LTK lookups still succeed because `trouble-host` performs software
/// IRK resolution in `Identity::match_address`.
///
/// Returns `(bond_count, has_irk)` — `has_irk` is always `false` since the RL
/// is disabled, so `can_hardware_filter` remains false and Mode B always uses
/// `Unfiltered` policy (non-discoverability provides equivalent protection).
#[cfg(feature = "hardware")]
async fn update_controller_filter_lists<C, P>(stack: &Stack<'_, C, P>, bonds: &[BondInformation]) -> (usize, bool)
where
    C: Controller,
    P: PacketPool,
{
    // --- Filter Accept List: always rebuilt from scratch ---
    if let Err(e) = stack.command(LeClearFilterAcceptList::new()).await {
        log::warn!("BLE: clear FAL failed: {:?}", e);
    }
    for bond in bonds {
        let addr = bond.identity.bd_addr;
        let addr_kind = infer_addr_kind(&addr);
        log::debug!(
            "BLE: FAL += {:?} {:02X?} (irk={})",
            addr_kind,
            addr.raw(),
            bond.identity.irk.is_some(),
        );
        if let Err(e) = stack.command(LeAddDeviceToFilterAcceptList::new(addr_kind, addr)).await {
            log::warn!("BLE: add to FAL failed: {:?}", e);
        }
    }

    // Resolving List is intentionally NOT populated (hardware address resolution
    // stays off). With RL active the controller resolves RPAs in
    // LeEnhancedConnectionComplete to identity addresses; trouble-host then uses
    // that identity address as peer_identity and passes it to LESC f6 DHKey check
    // — but the phone computes f6 with its RPA, causing a DHKey mismatch and a
    // silent pairing failure ("Remote User terminated Connection"). With RL
    // disabled, peer_identity.bd_addr = RPA → f6 correct. LTK lookups still work:
    // trouble-host's Identity::match_address() calls irk.resolve_address() in
    // software to match a stored bond's IRK against the incoming RPA.
    log::debug!(
        "BLE: controller lists updated — {} bond(s) in FAL (RL disabled for LESC compat)",
        bonds.len(),
    );
    (bonds.len(), false)
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
    pairing_open_at_connect: bool,
    tx_auth: &core::sync::atomic::AtomicBool,
    phones_need_persist: &core::cell::Cell<bool>,
    bonds_need_persist: &core::cell::Cell<bool>,
) {
    let rx_handle = server.link.rx.handle;
    // Snapshot bonds at connection start. IRK-based resolution via
    // Identity::match_address() means bonded phones using RPAs are recognised
    // even when the hardware Resolving List is inactive.
    let bonds_at_connect = stack.get_bond_information();
    let peer_addr_at_connect = {
        let mut addr = [0u8; 6];
        addr.copy_from_slice(conn.raw().peer_identity().bd_addr.raw());
        addr
    };
    // BdAddr is Copy; created once and reused throughout the function.
    let peer_bd_addr = BdAddr::new(peer_addr_at_connect);
    // is_preexisting_bond: true only when this phone is in our phone registry AND
    // has a stored LTK. Checking the registry (not just the LTK store) prevents a
    // phone that bonded but was never registered from bypassing the pairing window
    // on reconnect and acquiring tx_auth or forcing a fresh key exchange.
    let is_preexisting_bond = {
        let registry_id = bonds_at_connect
            .iter()
            .find(|b| b.identity.match_address(&peer_bd_addr))
            .map(|b| {
                let mut id = Vec::with_capacity(10);
                id.extend_from_slice(b"ble:");
                id.extend_from_slice(b.identity.bd_addr.raw());
                id
            });
        match registry_id {
            Some(id) => core_interface::is_phone_paired(&id).await,
            None => false,
        }
    };
    let peer_addr_kind = infer_addr_kind(&conn.raw().peer_identity().bd_addr);
    log::info!(
        "BLE gatt: peer {:?} {:02X?} connected — is_preexisting_bond={} ({} known bond(s), pairing_open={})",
        peer_addr_kind,
        peer_addr_at_connect,
        is_preexisting_bond,
        bonds_at_connect.len(),
        pairing_open_at_connect,
    );

    // Reject unknown phones immediately when the pairing window is closed.
    // A registered phone (is_preexisting_bond=true) may reconnect to re-encrypt
    // with its existing LTK. Everyone else must wait for the window to open.
    if !pairing_open_at_connect && !is_preexisting_bond {
        log::info!("BLE: disconnecting unknown peer — pairing window closed");
        return;
    }

    // security_requested prevents sending more than one Security Request PDU
    // per connection. It is reset on PairingComplete so a hypothetical future
    // re-keying event could trigger a fresh request if needed.
    let mut security_requested = false;
    let mut verified_peer_id: Option<Vec<u8>> = None;

    let mut disconnect_after_new_bond = false;

    // For known phones (is_preexisting_bond=true), wait up to 200 ms for the
    // phone to re-encrypt with its stored LTK (LL_ENC_REQ). A phone that still
    // has its LTK will send LL_ENC_REQ within the first few connection intervals
    // (typically < 100 ms). A phone that "forgot" the device on the OS side
    // (deleted its LTK) cannot perform LL_ENC_REQ and will instead skip straight
    // to a fresh Pairing_Request.
    //
    // Detecting LL_ENC_REQ here serves two purposes:
    //   a) Grant tx_auth immediately for the LL re-encryption path. iOS does not
    //      send a Pairing_Request after a successful LL_ENC_REQ, so PairingComplete
    //      never fires on that path — without this, tx_auth would stay false and
    //      outbound notifications would be blocked for the entire session.
    //   b) Record whether the phone proved it has its LTK so that PairingComplete
    //      (which always fires when a Pairing_Request is sent) can tell the
    //      difference between a legitimate key-refresh and a "forgot device" attempt.
    if is_preexisting_bond {
        'ltk_wait: for _ in 0..20u8 {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
            if link_is_encrypted(conn) {
                let device_id = bonds_at_connect
                    .iter()
                    .find(|b| b.identity.match_address(&peer_bd_addr))
                    .map(|b| {
                        let mut id = Vec::with_capacity(10);
                        id.extend_from_slice(b"ble:");
                        id.extend_from_slice(b.identity.bd_addr.raw());
                        id
                    })
                    .unwrap_or_else(|| peer_device_id(conn));
                verified_peer_id = Some(device_id);
                tx_auth.store(true, core::sync::atomic::Ordering::Relaxed);
                log::info!(
                    "BLE: preexisting bond re-encrypted via LTK, granting tx_auth (pairing_open={})",
                    pairing_open_at_connect,
                );
                break 'ltk_wait;
            }
        }
    }

    loop {
        sync_bondable_with_window(conn, pairing_open_at_connect);
        // When a new bond was just created we want to persist the LTK to flash as
        // soon as possible. We can't write flash during an active connection (the
        // erase disables interrupts for ~20-100 ms, which drops the BLE link), so
        // the only safe window is after disconnect. We give the phone 500 ms to
        // finish its own key-storage bookkeeping and drain any queued TX frames,
        // then force a disconnect. The phone will auto-reconnect in Mode B.
        let next_event = if disconnect_after_new_bond {
            match select(
                conn.next(),
                embassy_time::Timer::after(embassy_time::Duration::from_millis(500)),
            )
            .await
            {
                Either::First(e) => e,
                Either::Second(_) => {
                    log::info!("BLE: disconnecting after new bond — will persist LTK immediately");
                    return;
                }
            }
        } else {
            conn.next().await
        };
        match next_event {
            GattConnectionEvent::Disconnected { reason } => {
                log::info!("BLE GATT disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::PassKeyDisplay(key) => {
                log::info!("BLE pairing passkey: {:06}", key.value());
            }
            GattConnectionEvent::PassKeyConfirm(key) => {
                // Allow numeric-comparison confirmation if the pairing window is open
                // OR if this is a preexisting bond re-pairing after the phone removed
                // its bond. Without the is_preexisting_bond check, window-closed
                // reconnects always fail with PairingFailed, which disconnects the phone.
                if core_interface::is_pairing_window_open() || is_preexisting_bond {
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
                // Encryption is now established — reset so a future re-keying event
                // could trigger a fresh request if needed.
                security_requested = false;
                // Phase 2: prefer the stable identity address from the key-distribution
                // PDU (bond.identity.bd_addr) over peer_identity().bd_addr, which may
                // still be the current RPA at the moment this event fires.
                let device_id = if let Some(ref b) = bond {
                    let mut id = Vec::with_capacity(10);
                    id.extend_from_slice(b"ble:");
                    id.extend_from_slice(b.identity.bd_addr.raw());
                    id
                } else {
                    // Re-encryption via existing LTK: no new keys exchanged.
                    // The hardware Resolving List is disabled so peer_identity().bd_addr
                    // is the RPA (not the identity address). Look up the matching bond
                    // via software IRK resolution to recover the same stable
                    // identity-based device_id that was registered at first pairing.
                    bonds_at_connect
                        .iter()
                        .find(|b| b.identity.match_address(&peer_bd_addr))
                        .map(|b| {
                            let mut id = Vec::with_capacity(10);
                            id.extend_from_slice(b"ble:");
                            id.extend_from_slice(b.identity.bd_addr.raw());
                            id
                        })
                        .unwrap_or_else(|| peer_device_id(conn))
                };
                verified_peer_id = Some(device_id.clone());

                // A phone that had its LTK proves it via LL_ENC_REQ before sending
                // Pairing_Request. The pre-loop wait above sets tx_auth=true when
                // LL_ENC_REQ is detected. If tx_auth is still false here, the phone
                // sent a Pairing_Request without prior re-encryption — meaning it has
                // no LTK (the user "forgot" the device in their OS BT settings).
                // Outside the pairing window that is not permitted: the phone must
                // wait for the window to open and go through a full re-pair.
                if !pairing_open_at_connect
                    && is_preexisting_bond
                    && !tx_auth.load(core::sync::atomic::Ordering::Relaxed)
                {
                    log::warn!(
                        "BLE: phone has no LTK (no LL_ENC_REQ before Pairing_Request) — disconnecting outside window"
                    );
                    return;
                }

                // Phase 1: also allow re-registration for preexisting-bonded phones
                // that reconnect outside the pairing window.
                // is_preexisting_bond is computed once at connection start (above).
                let pairing_open = core_interface::is_pairing_window_open();
                let already_known = core_interface::is_phone_paired(&device_id).await;
                let added_or_existing = if pairing_open || already_known || is_preexisting_bond {
                    core_interface::add_paired_phone(&device_id).await
                } else {
                    false
                };
                log::info!(
                    "BLE pairing complete: encrypted={}, authenticated={}, bonded={}, pairing_open={}, already_known={}, preexisting_bond={}, authorized={}, device_id={:02X?}",
                    security_level.encrypted(),
                    security_level.authenticated(),
                    bond.is_some(),
                    pairing_open,
                    already_known,
                    is_preexisting_bond,
                    added_or_existing,
                    device_id,
                );
                if !added_or_existing {
                    // Phone is not in the registry and the pairing window is closed.
                    // Disconnect so it cannot hold an encrypted session without
                    // authorization. bondable=false (set above) means no LTK was
                    // stored on either side, so the phone has nothing persistent.
                    log::warn!("BLE: disconnecting — unauthorized phone outside pairing window");
                    return;
                }
                // Phase 3: authorize TX now that the phone is confirmed.
                tx_auth.store(true, core::sync::atomic::Ordering::Relaxed);
                // Flash write is deferred to after the connection closes — see
                // the outer loop in ble_transport_task for why.
                phones_need_persist.set(true);
                if bond.is_some() {
                    bonds_need_persist.set(true);
                    // Signal the event loop to close the connection after a short
                    // grace period so the LTK can be written to flash promptly.
                    disconnect_after_new_bond = true;
                }
            }
            GattConnectionEvent::PairingFailed(e) => {
                log::warn!("BLE pairing failed: {:?}", e);
                // Reset so the next GATT write can trigger a fresh Security_Request.
                // Without this, a PairingFailed (e.g. from a phone that removed its
                // bond) permanently blocks re-pairing attempts in the same connection.
                security_requested = false;
            }
            GattConnectionEvent::Gatt { event } => match event {
                GattEvent::Write(write_evt) if write_evt.handle() == rx_handle => {
                    if !link_is_encrypted(conn) {
                        if !security_requested {
                            request_link_security(conn);
                            security_requested = true;
                        }
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
                    // is_preexisting_bond is computed once at connection start (above).
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
    let mut pending_msg: Option<proto::DeviceToApp> = None;
    let mut tx_held_logged = false;

    loop {
        let msg = if let Some(msg) = pending_msg.take() {
            msg
        } else {
            BLE_TX_CHANNEL.receive().await
        };

        // tx_auth is set by gatt_event_task on PairingComplete — the single gate
        // that confirms the link is encrypted and the peer is authorised. All
        // security and pairing logic lives in gatt_event_task; this task just
        // waits until that gate opens.
        if !tx_auth.load(core::sync::atomic::Ordering::Relaxed) {
            if !tx_held_logged {
                log::debug!("BLE TX: holding message — awaiting PairingComplete (tx_auth not yet set)");
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
