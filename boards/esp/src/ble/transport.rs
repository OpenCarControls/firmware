//! BLE GATT transport task.
//!
//! Implements the full advertise → connect → service loop on top of `trouble-host`.
//!
//! **Advertising modes**
//! - Mode A (pairing window open): general-discoverable, full name and service UUID
//!   in scan response. New phones can find and pair with the device.
//! - Mode B (pairing window closed): non-discoverable flags only. Bonded phones that
//!   already know the device address can still reconnect; strangers cannot see it.
//!
//! **Security model**
//! - RX writes require link encryption (enforced by `trouble-host` via `permissions(encrypted)`).
//! - TX notifications are withheld until `ble_tx_notify_task` confirms encryption via
//!   `conn.raw().security_level()` and sets per-connection `tx_auth`.
//! - `source_device_id` in every accepted message is overwritten with the firmware-verified
//!   BLE address so the vehicle layer always has a trustworthy caller identity.

#[cfg(feature = "hardware")]
mod hardware {
    use alloc::vec::Vec;
    use core::sync::atomic::{AtomicBool, Ordering};

    use core_interface::{BLE_RX_CHANNEL, BLE_TX_CHANNEL, proto};
    use embassy_embedded_hal::adapter::BlockingAsync;
    use embassy_futures::{
        join::join,
        select::{Either, select},
    };
    use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash, ReadNorFlash};
    use esp_hal::peripherals;
    use esp_radio::ble;
    use esp_storage::FlashStorage;
    use prost::Message as _;
    use trouble_host::prelude::*;

    use bt_hci::cmd::le::{
        LeAddDeviceToFilterAcceptList, LeAddDeviceToResolvingList, LeClearFilterAcceptList,
        LeClearResolvingList, LeRemoveDeviceFromFilterAcceptList, LeRemoveDeviceFromResolvingList,
        LeSetAddrResolutionEnable,
    };
    use bt_hci::controller::ControllerCmdSync;

    use crate::ble::store::BondStore;
    use core_interface::ble::{GATT_RX_UUID, GATT_SERVICE_UUID, GATT_TX_UUID, mac_to_ble_address};

    const BLE_CONNECTIONS_MAX: usize = 1;
    const BLE_L2CAP_CHANNELS_MAX: usize = 3;

    #[gatt_server]
    struct OpenCarGattServer {
        link: OpenCarGattService,
    }

    #[gatt_service(uuid = GATT_SERVICE_UUID)]
    struct OpenCarGattService {
        #[characteristic(uuid = GATT_RX_UUID, write, write_without_response, permissions(encrypted))]
        rx: heapless::Vec<u8, 244>,
        #[characteristic(uuid = GATT_TX_UUID, notify, permissions(encrypted))]
        tx: heapless::Vec<u8, 244>,
    }

    // ─── RNG wrapper ──────────────────────────────────────────────────────────────

    struct MyRng(esp_hal::rng::Rng);

    impl rand_core::RngCore for MyRng {
        fn next_u32(&mut self) -> u32 {
            self.0.random()
        }
        fn next_u64(&mut self) -> u64 {
            ((self.0.random() as u64) << 32) | self.0.random() as u64
        }
        fn fill_bytes(&mut self, dest: &mut [u8]) {
            for chunk in dest.chunks_mut(4) {
                let r = self.0.random().to_le_bytes();
                chunk.copy_from_slice(&r[..chunk.len()]);
            }
        }
        fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
            self.fill_bytes(dest);
            Ok(())
        }
    }

    impl rand_core::CryptoRng for MyRng {}

    // ─── Device name helper ───────────────────────────────────────────────────────

    pub(super) fn build_ble_device_name(base: &str, mac: [u8; 6]) -> heapless::String<33> {
        use core::fmt::Write as _;
        let mut out = heapless::String::<33>::new();
        let _ = write!(&mut out, "{}-{:02X}{:02X}", base, mac[4], mac[5]);
        if out.is_empty() {
            let _ = write!(&mut out, "OpenCar-{:02X}{:02X}", mac[4], mac[5]);
        }
        out
    }

    // ─── Main BLE task ────────────────────────────────────────────────────────────

    #[embassy_executor::task]
    pub async fn ble_transport_task(
        bt_peri: peripherals::BT<'static>,
        flash_peri: peripherals::FLASH<'static>,
        name_base: &'static str,
    ) {
        use static_cell::StaticCell;

        static BLE_DEVICE_NAME: StaticCell<heapless::String<33>> = StaticCell::new();

        // ── Flash / bond store ──────────────────────────────────────────────────
        let flash_raw = FlashStorage::new(flash_peri);
        #[cfg(target_arch = "xtensa")]
        let flash_raw = flash_raw.multicore_auto_park();
        let mut flash_async = BlockingAsync::new(flash_raw);
        let capacity = flash_async.capacity();
        let bond_store_range =
            (capacity - 2 * FlashStorage::SECTOR_SIZE as usize) as u32..capacity as u32;
        let mut bond_store = BondStore::new(&mut flash_async, bond_store_range);

        let loaded_bonds = bond_store.load_bonds().await;

        // ── BLE host setup ──────────────────────────────────────────────────────
        let connector = match ble::controller::BleConnector::new(bt_peri, ble::Config::default()) {
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

        let mac_efuse = esp_hal::efuse::base_mac_address();
        let mut mac = [0u8; 6];
        mac.copy_from_slice(mac_efuse.as_bytes());
        let name = BLE_DEVICE_NAME.init(build_ble_device_name(name_base, mac));
        let name = name.as_str();
        let address: Address = Address::random(mac_to_ble_address(mac));

        let mut my_rng = MyRng(esp_hal::rng::Rng::new());
        let stack = trouble_host::new(controller, resources)
            .set_random_address(address)
            .set_random_generator_seed(&mut my_rng);
        stack.set_io_capabilities(IoCapabilities::NoInputNoOutput);

        for bond in &loaded_bonds {
            if let Err(e) = stack.add_bond_information(bond.clone()) {
                log::warn!("BLE: failed to restore bond: {:?}", e);
            }
        }

        let Host {
            mut peripheral,
            mut runner,
            ..
        } = stack.build();

        let server = OpenCarGattServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name,
            appearance: &appearance::motorized_vehicle::CAR,
        }))
        .unwrap();

        log::info!("BLE transport: GATT host initialised, starting advertise loop");

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
                    let pairing_open = core_interface::is_pairing_window_open();

                    // Sync FAL + RL with in-memory bonds before every advertise cycle.
                    // Must happen while advertising is inactive (BLE spec requirement).
                    let bonds = stack.get_bond_information();
                    let (bond_count, has_irk) =
                        update_controller_filter_lists(&stack, bonds.as_slice()).await;
                    let can_hardware_filter = bond_count > 0 && has_irk;

                    match advertise_and_accept(
                        name,
                        pairing_open,
                        can_hardware_filter,
                        &mut peripheral,
                        &server,
                    )
                    .await
                    {
                        Ok(Some(conn)) => {
                            let pairing_open_at_connect = pairing_open;
                            log::info!(
                                "BLE transport: central connected peer={:02X?} (pairing_window={})",
                                conn.raw().peer_identity().bd_addr.raw(),
                                pairing_open_at_connect
                            );
                            let tx_auth = AtomicBool::new(false);
                            let _ = select(
                                gatt_event_task(
                                    &server,
                                    &conn,
                                    &stack,
                                    &mut bond_store,
                                    pairing_open_at_connect,
                                ),
                                ble_tx_notify_task(&server, &conn, &tx_auth),
                            )
                            .await;
                            log::info!("BLE transport: connection closed, returning to advertise");
                        }
                        Ok(None) => {
                            // Advertise timeout — re-check pairing window on next iteration.
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

    // ─── Advertising ─────────────────────────────────────────────────────────────

    async fn advertise_and_accept<'values, 'server, C: Controller>(
        name: &'values str,
        pairing_window_open: bool,
        can_hardware_filter: bool,
        peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
        server: &'server OpenCarGattServer<'values>,
    ) -> Result<Option<GattConnection<'values, 'server, DefaultPacketPool>>, BleHostError<C::Error>>
    {
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
            // Mode B: non-discoverable — bonded phones can still reconnect.
            let adv_len = AdStructure::encode_slice(
                &[AdStructure::Flags(BR_EDR_NOT_SUPPORTED)],
                &mut adv_data,
            )?;
            (adv_len, 0)
        };

        // Mode B + RL active: use FilterConn so the hardware rejects connections from
        // unknown devices. Safe only when can_hardware_filter=true (RL has IRK entries
        // to resolve RPAs before the FAL check). In Mode A always use Unfiltered.
        let filter_policy = if pairing_window_open || !can_hardware_filter {
            AdvFilterPolicy::Unfiltered
        } else {
            AdvFilterPolicy::FilterConn
        };
        log::debug!(
            "BLE advertise: mode={} filter={:?} (pairing_open={} can_hw_filter={})",
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

        // Timeout so the outer loop can re-check the pairing window state within ~10 s.
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

    // ─── Address kind helper ──────────────────────────────────────────────────────

    fn infer_addr_kind(bd_addr: &BdAddr) -> bt_hci::param::AddrKind {
        // BdAddr raw bytes: index 5 is the most-significant byte.
        // Random static addresses have bits 7:6 of MSB set to 0b11.
        if (bd_addr.raw()[5] & 0xC0) == 0xC0 {
            bt_hci::param::AddrKind::RANDOM
        } else {
            bt_hci::param::AddrKind::PUBLIC
        }
    }

    // ─── Controller list management ───────────────────────────────────────────────

    /// Add one bond entry to the controller's Filter Accept List and (if IRK present)
    /// Resolving List. Call this only while advertising is inactive.
    async fn add_bond_to_controller<C, P>(stack: &Stack<'_, C, P>, bond: &BondInformation)
    where
        C: Controller
            + ControllerCmdSync<LeAddDeviceToFilterAcceptList>
            + ControllerCmdSync<LeAddDeviceToResolvingList>,
        P: PacketPool,
    {
        let addr_bytes = bond.identity.bd_addr.0;
        let addr_kind = infer_addr_kind(&bond.identity.bd_addr);
        let hci_addr = bt_hci::param::BdAddr::new(addr_bytes);

        let _ = stack
            .command(LeAddDeviceToFilterAcceptList::new(addr_kind, hci_addr))
            .await;

        if let Some(irk) = &bond.identity.irk {
            let _ = stack
                .command(LeAddDeviceToResolvingList::new(
                    addr_kind,
                    hci_addr,
                    irk.0.to_le_bytes(),
                    [0u8; 16], // local IRK (all zeros — we use a static random address)
                ))
                .await;
        }
    }

    /// Remove one bond entry from the controller's Resolving List and Filter Accept
    /// List. Disables RL briefly around the RL modification as required by the spec.
    async fn remove_bond_from_controller<C, P>(stack: &Stack<'_, C, P>, bond: &BondInformation)
    where
        C: Controller
            + ControllerCmdSync<LeRemoveDeviceFromFilterAcceptList>
            + ControllerCmdSync<LeRemoveDeviceFromResolvingList>
            + ControllerCmdSync<LeSetAddrResolutionEnable>,
        P: PacketPool,
    {
        let addr_bytes = bond.identity.bd_addr.0;
        let addr_kind = infer_addr_kind(&bond.identity.bd_addr);
        let hci_addr = bt_hci::param::BdAddr::new(addr_bytes);

        let _ = stack.command(LeSetAddrResolutionEnable::new(false)).await;
        if bond.identity.irk.is_some() {
            let _ = stack
                .command(LeRemoveDeviceFromResolvingList::new(addr_kind, hci_addr))
                .await;
        }
        let _ = stack
            .command(LeRemoveDeviceFromFilterAcceptList::new(addr_kind, hci_addr))
            .await;
        let _ = stack.command(LeSetAddrResolutionEnable::new(true)).await;
    }

    /// Rebuild the controller's Filter Accept List and Resolving List from the
    /// current in-memory bond store. Must be called while advertising is inactive.
    ///
    /// Returns `(bond_count, has_irk_any)`. `can_hardware_filter` in the outer loop
    /// is set to `bond_count > 0 && has_irk_any` — FilterConn is only safe when at
    /// least one bond has an IRK so the controller can resolve RPAs before checking
    /// the FAL.
    async fn update_controller_filter_lists<C, P>(
        stack: &Stack<'_, C, P>,
        bonds: &[BondInformation],
    ) -> (usize, bool)
    where
        C: Controller
            + ControllerCmdSync<LeClearFilterAcceptList>
            + ControllerCmdSync<LeClearResolvingList>
            + ControllerCmdSync<LeSetAddrResolutionEnable>
            + ControllerCmdSync<LeAddDeviceToFilterAcceptList>
            + ControllerCmdSync<LeAddDeviceToResolvingList>,
        P: PacketPool,
    {
        // Disable address resolution while modifying RL (spec requirement).
        let _ = stack.command(LeSetAddrResolutionEnable::new(false)).await;
        let _ = stack.command(LeClearFilterAcceptList::new()).await;
        let _ = stack.command(LeClearResolvingList::new()).await;

        let mut has_irk = false;
        for bond in bonds {
            let addr_bytes = bond.identity.bd_addr.0;
            let addr_kind = infer_addr_kind(&bond.identity.bd_addr);
            let hci_addr = bt_hci::param::BdAddr::new(addr_bytes);

            let _ = stack
                .command(LeAddDeviceToFilterAcceptList::new(addr_kind, hci_addr))
                .await;

            if let Some(irk) = &bond.identity.irk {
                let _ = stack
                    .command(LeAddDeviceToResolvingList::new(
                        addr_kind,
                        hci_addr,
                        irk.0.to_le_bytes(),
                        [0u8; 16],
                    ))
                    .await;
                has_irk = true;
            }
        }

        // Re-enable address resolution if any bond has an IRK.
        if has_irk {
            let _ = stack.command(LeSetAddrResolutionEnable::new(true)).await;
        }

        log::debug!(
            "BLE: controller lists updated — {} bond(s) in FAL, RL {}",
            bonds.len(),
            if has_irk {
                "enabled"
            } else {
                "disabled (no IRKs)"
            },
        );
        (bonds.len(), has_irk)
    }

    // ─── GATT event loop ──────────────────────────────────────────────────────────

    async fn gatt_event_task<C, P, S>(
        server: &OpenCarGattServer<'_>,
        conn: &GattConnection<'_, '_, P>,
        stack: &Stack<'_, C, P>,
        bond_store: &mut BondStore<'_, S>,
        pairing_open_at_connect: bool,
    ) where
        C: Controller
            + ControllerCmdSync<LeAddDeviceToFilterAcceptList>
            + ControllerCmdSync<LeAddDeviceToResolvingList>
            + ControllerCmdSync<LeRemoveDeviceFromFilterAcceptList>
            + ControllerCmdSync<LeRemoveDeviceFromResolvingList>
            + ControllerCmdSync<LeSetAddrResolutionEnable>,
        P: PacketPool,
        S: NorFlash + MultiwriteNorFlash,
    {
        let rx_handle = server.link.rx.handle;
        let peer_bd_addr = conn.raw().peer_identity().bd_addr;

        // Determine if this phone has a stored bond (RL active → bd_addr is identity).
        let is_known = stack
            .get_bond_information()
            .iter()
            .any(|b| b.identity.bd_addr == peer_bd_addr || b.identity.match_address(&peer_bd_addr));

        log::info!(
            "BLE gatt: peer {:02X?} connected — is_known={} pairing_open={}",
            peer_bd_addr.raw(),
            is_known,
            pairing_open_at_connect,
        );

        // Software fallback: reject unknown phones when the pairing window is closed.
        // (Hardware FilterConn should prevent this from happening in Mode B, but we
        // apply the check here as a defence-in-depth measure.)
        if !pairing_open_at_connect && !is_known {
            log::info!("BLE: disconnecting unknown peer — pairing window closed");
            return;
        }

        if let Err(e) = conn.raw().set_bondable(pairing_open_at_connect) {
            log::warn!(
                "BLE set_bondable({}) failed: {:?}",
                pairing_open_at_connect,
                e
            );
        }

        loop {
            match conn.next().await {
                GattConnectionEvent::Disconnected { reason } => {
                    log::info!("BLE GATT disconnected: {:?}", reason);
                    // Broken-bond auto-heal: if the phone disconnected with a security error
                    // while the pairing window is open, wipe the stored bond so it can
                    // re-pair fresh on the next connection.
                    let is_security_disconnect = matches!(
                        reason,
                        bt_hci::param::Status::PIN_OR_KEY_MISSING
                            | bt_hci::param::Status::REMOTE_USER_TERMINATED_CONN
                            | bt_hci::param::Status::AUTHENTICATION_FAILURE
                            | bt_hci::param::Status::CONN_TERMINATED_DUE_TO_MIC_FAILURE,
                    );
                    if is_security_disconnect && core_interface::is_pairing_window_open() {
                        let bonds = stack.get_bond_information();
                        if let Some(known_bond) =
                            bonds.iter().find(|b| b.identity.bd_addr == peer_bd_addr)
                        {
                            let _ = stack.remove_bond_information(known_bond.identity);
                            let _ = bond_store.remove_bond(&peer_bd_addr).await;
                            remove_bond_from_controller(stack, known_bond).await;
                            log::info!(
                                "BLE: wiped broken bond for {:02X?} (auto-heal)",
                                peer_bd_addr.raw()
                            );
                        }
                    }
                    break;
                }
                GattConnectionEvent::PairingComplete {
                    security_level,
                    bond,
                } => {
                    if let Some(bond_info) = bond {
                        // New bond: persist inline (async, non-blocking; BlockingAsync
                        // wraps synchronous writes so BLE events continue processing).
                        let _ = bond_store.remove_bond(&bond_info.identity.bd_addr).await;
                        let _ = bond_store.store_bond(&bond_info).await;
                        add_bond_to_controller(stack, &bond_info).await;
                        log::info!(
                            "BLE: new bond stored inline for {:02X?}",
                            bond_info.identity.bd_addr.raw()
                        );
                    }
                    if !security_level.encrypted() {
                        log::warn!("BLE: PairingComplete with encrypted=false");
                    }
                }
                GattConnectionEvent::PairingFailed(e) => {
                    log::warn!("BLE pairing failed: {:?}", e);
                }
                GattConnectionEvent::Gatt { event } => match event {
                    GattEvent::Write(write_evt) if write_evt.handle() == rx_handle => {
                        // With `permissions(encrypted)` on the rx characteristic, trouble-host
                        // automatically rejects writes on unencrypted links before this event
                        // fires. The payload here is always from an encrypted connection.
                        let payload = write_evt.data().to_vec();
                        if let Ok(reply) = write_evt.accept() {
                            let _ = reply.send().await;
                        }

                        // An empty write is the standard BLE pattern for triggering OS-level
                        // pairing/encryption — the app writes zero bytes to provoke the
                        // encrypted-characteristic handshake. Nothing to decode; skip silently.
                        if payload.is_empty() {
                            continue;
                        }

                        match proto::AppToDevice::decode(payload.as_slice()) {
                            Ok(mut msg) => {
                                // Overwrite source_device_id with the firmware-verified peer
                                // address so the app cannot spoof its identity.
                                let mut device_id = Vec::with_capacity(10);
                                device_id.extend_from_slice(b"ble:");
                                device_id.extend_from_slice(peer_bd_addr.raw());
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

    // ─── TX notify loop ───────────────────────────────────────────────────────────

    async fn ble_tx_notify_task<P: PacketPool>(
        server: &OpenCarGattServer<'_>,
        conn: &GattConnection<'_, '_, P>,
        tx_auth: &AtomicBool,
    ) {
        let mut pending_msg: Option<proto::DeviceToApp> = None;
        let mut tx_held_logged = false;

        loop {
            let msg = if let Some(msg) = pending_msg.take() {
                msg
            } else {
                BLE_TX_CHANNEL.receive().await
            };

            // Hold notifications until the link is confirmed encrypted.
            // tx_auth is a per-connection cache: once set it avoids calling security_level() on
            // every message. Checking security_level() here covers both the first-pair path
            // (encryption established during PairingComplete) and the reconnect path (re-encryption
            // via stored LTK, which never fires PairingComplete on iOS).
            if !tx_auth.load(Ordering::Relaxed) {
                let encrypted = conn
                    .raw()
                    .security_level()
                    .map(|l| l.encrypted())
                    .unwrap_or(false);
                if encrypted {
                    tx_auth.store(true, Ordering::Relaxed);
                    log::info!("BLE TX: link encrypted — tx_auth granted");
                } else {
                    if !tx_held_logged {
                        log::debug!("BLE TX: holding message — link not yet encrypted");
                        tx_held_logged = true;
                    }
                    pending_msg = Some(msg);
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
                    continue;
                }
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
}

#[cfg(feature = "hardware")]
pub use hardware::*;

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(all(test, not(feature = "hardware")))]
fn build_ble_device_name(base: &str, mac: [u8; 6]) -> std::string::String {
    use core::fmt::Write as _;
    let mut out = std::string::String::new();
    let _ = write!(&mut out, "{}-{:02X}{:02X}", base, mac[4], mac[5]);
    out
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
