# BLE (ESP)

Implementation notes for the BLE transport in `boards/esp/src/ble/`. The PC board only mocks BLE. For how BLE fits in the overall design, see [Architecture](architecture.md).

## Stack and versions
BLE uses `esp-radio 0.18` + `trouble-host 0.6.0` (with `bt-hci 0.8.1`), built with features `derive`, `security`, `default-packet-pool-mtu-255`.

`trouble_host::new(controller, resources)` returns `Stack<'resources, C, P>` (outer variable); `stack.build()` takes `&'stack self` and returns non-exhaustive `Host { peripheral, runner, .. }` — the `Stack` is still usable after `build()` for `get_bond_information()`, `add_bond_information()`, and `command()`.

## GATT transport
`ble_transport_task` runs a full advertise/connect loop and bridges GATT RX/TX characteristics to `BLE_RX_CHANNEL` / `BLE_TX_CHANNEL`. It accepts a `name_base: &'static str` 4th parameter (injected by the generated `main.rs` as `BLE_DEVICE_NAME_BASE`).

## Message size
BLE messages are never fragmented: every encoded `AppToDevice` / `DeviceToApp` must fit in one ATT write or notification. The limit comes from the contract (`ble.payload` in `transport.toml`): the app requests an ATT MTU of `BLE_ATT_MTU` (247), so a message carries at most `BLE_MAX_MESSAGE_BYTES` (244). Both constants are generated into `core_interface::ble`; the GATT characteristics are sized with `BLE_MAX_MESSAGE_BYTES`, and a compile-time check ensures the stack's packet pool supports `BLE_ATT_MTU`.

What keeps messages within the limit:
*   **State updates:** each vehicle crate has a `worst_case_state_update_fits_one_ble_message` test that builds the state update through `core_interface::ble_state_update` with every field at its longest encoding. Adding fields to a vehicle's state fails that test before it can fail on a car.
*   **CAN debug batches:** `publish_single_debug_batch` splits each batch (up to 20 frames) into as many `CanDebugUpdate` messages as needed. A single 8-byte frame takes 20–30 bytes, so a full batch needs two or three messages.
*   Anything larger is dropped on the device with a `BLE GATT TX message too large` warning.

## Device name
Configurable base name via `[hardware.esp.ble] device_name` in `config.toml` (default `"OpenCar"`). At runtime `ble_transport_task` appends a 2-byte MAC suffix using `build_ble_device_name(base, mac)` → `"{base}-{mac[4]:02X}{mac[5]:02X}"` (e.g. `"OpenCar-ABCD"`). The result is stored in a `StaticCell<heapless::String<33>>` for the `'static` lifetime required by `PeripheralConfig`.

## UUIDs / advertise data
The OpenCar 128-bit service UUID is included in scan response (`AdStructure::ServiceUuids128`) so phone apps can filter devices by service. The UUID is only advertised in Mode A (pairing window open) — see below.

## Stable BLE identity
Device address is derived from the efuse base MAC (`esp_hal::efuse::base_mac_address()` free function returning a `MacAddress` newtype; use `.as_bytes()` to get `&[u8; 6]`) and converted to a static-random BLE address (`mac_to_ble_address`), so identity is stable across reboots.

## Advertising mode policy
`advertise_and_accept()` selects one of two modes on each loop iteration based on `is_pairing_window_open()`:

*   **Mode A (window open):** `AD_FLAG_LE_LIMITED_DISCOVERABLE | BR_EDR_NOT_SUPPORTED`, full device name, and service UUID in scan response — visible to all BLE scanners. `set_bondable(true)` is applied to accepted connections.
*   **Mode B (window closed):** `BR_EDR_NOT_SUPPORTED` only, no name, no UUIDs — invisible in general BLE discovery. Bonded phones that already know the device address can still initiate a connection. `set_bondable(false)` is applied to accepted connections.

The advertiser has a **10-second accept timeout** so the loop restarts and can switch modes within ~10 seconds of any pairing-window state change. The transition is logged.

**Controller FAL / Resolving List:** Before each advertise cycle, `update_controller_filter_lists` programs the controller FAL and Resolving List via `stack.command()` with bonded device addresses and IRKs. In Mode B when at least one bond has an IRK (`can_hardware_filter = true`), `AdvFilterPolicy::FilterConn` is set so the hardware rejects connections from unknown devices at the controller level. `FilterConn` is only enabled when the RL has IRK entries so the controller can resolve RPAs before the FAL check — using it without a populated RL would break reconnects from phones using Resolvable Private Addresses.

## Security enforcement model
Both RX and TX GATT characteristics declare `permissions(encrypted)` in the `#[characteristic(...)]` macro (supported in `trouble-host 0.6`). Additional runtime layers:

*   RX writes are rejected with `AttErrorCode::INSUFFICIENT_AUTHORISATION` (British spelling) if the peer is not in the paired-phone registry AND the pairing window is closed.
*   `source_device_id` in every accepted inbound proto is **overwritten** with the firmware-verified BLE peer address; any app-supplied value is discarded. This makes `source_device_id` a server-side stamp, not a client claim.
*   TX notifications are withheld until `ble_tx_notify_task` confirms link encryption via `conn.raw().security_level()` (see below).
*   `request_security()` is used to trigger pairing/encryption when needed.

## TX buffering during encryption
`ble_tx_notify_task` holds a `pending_msg: Option<DeviceToApp>` buffer. If a message arrives from `BLE_TX_CHANNEL` before the link is encrypted, it is stored in `pending_msg` (not dropped) and retried every 50 ms.

Encryption is checked via a per-connection `tx_auth: AtomicBool` (initialized `false`). On each retry while `tx_auth` is `false`, `ble_tx_notify_task` calls `conn.raw().security_level()` — if the result is `Encrypted`, it sets `tx_auth = true` (cached for all subsequent messages) and sends immediately. This single check covers both the first-pair path (encryption established during `PairingComplete`) and the reconnect path (re-encryption via stored LTK, which never fires `PairingComplete` on iOS). `gatt_event_task` does not touch `tx_auth`.

## Pairing events
Passkey display/confirm/input and pairing-complete/failure events are handled in the BLE event loop. On `PairingComplete`, new bond data is persisted to flash and added to the controller filter lists. A warning is logged if `security_level` is not encrypted. `tx_auth` is not touched here — TX unblocking is handled entirely by `ble_tx_notify_task`.

## Known-peer reconnect
Bonded phones reconnect in Mode B and re-encrypt via LL_ENC_REQ using the stored LTK. `trouble-host` has no dedicated "encryption changed" event, so `ble_tx_notify_task` detects this by polling `conn.raw().security_level()` on each 50 ms retry until it returns `Encrypted`, then sets `tx_auth = true` and begins sending. iOS never sends `Pairing_Request` after LL_ENC_REQ so `PairingComplete` never fires on that path — this is the exact case the `security_level()` check was introduced to handle.

## Pairing lifecycle controls
`ble_lifecycle_task` implements active-low pairing-button hold detection (`pairing_button_hold_s`) and opens pairing windows using the transport contract value `contracts/opencar/core/v1/transport.toml` → `[ble.pairing].pairing_window_seconds` via `core_interface::open_pairing_window_for(...)`.

## Bondable mode policy
Bondability is set to match the pairing-window state at connect time (via `set_bondable(pairing_open)` in the advertise loop) and kept in sync during active connections via `sync_bondable_with_window()` — which keeps bondable `true` until the link is encrypted on first connect so LTKs can be exchanged, then follows the window state.

## Persistence (power-cycle)
Paired-phone data is persisted in flash using `esp-storage` (single sector, checksum + versioned payload) and restored at BLE startup.

LTKs (`BondInformation`) are persisted via `BleSecurityStore` in the second-to-last flash sector using `stack.get_bond_information()`; the sector before that holds the phone-identity store. On startup, `restore_security_store()` loads the bonds and feeds them back via `stack.add_bond_information()` before `stack.build()`, so the phone can re-encrypt without re-pairing after a power cycle.

**Current caveat:** persistence uses fixed end-of-flash regions (last two sectors), not dedicated partitions. Move to partition-table-backed regions when the OTA partition layout is introduced.
