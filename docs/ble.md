# BLE (ESP)

Implementation notes for the BLE transport in `boards/esp/src/ble/`. The PC board only mocks BLE. For how BLE fits in the overall design, see [Architecture](architecture.md).

## Stack and versions
BLE uses `esp-radio 0.18` + `trouble-host 0.6.0` (with `bt-hci 0.8.1`), built with features `derive`, `security`, `default-packet-pool-mtu-255`.

`trouble_host::new(controller, resources)` returns `Stack<'resources, C, P>` (outer variable); `stack.build()` takes `&'stack self` and returns non-exhaustive `Host { peripheral, runner, .. }` — the `Stack` is still usable after `build()` for `get_bond_information()`, `add_bond_information()`, and `command()`.

## GATT transport
`ble_transport_task` runs a full advertise/connect loop and bridges GATT RX/TX characteristics to `BLE_RX_CHANNEL` / `BLE_TX_CHANNEL`. It accepts a `name_base: &'static str` 4th parameter (injected by the generated `main.rs` as `BLE_DEVICE_NAME_BASE`).

## Message size and fragmentation
Messages larger than one ATT packet are split into chunks, as defined by the contract (`ble.payload` and `ble.fragmentation` in `transport.toml`, explained in `contracts/docs/ble-fragmentation.md`). Every write and notification is one chunk: a 1-byte header (bit 7 = last chunk, bits 0-6 = chunk index) and up to `negotiated MTU - 4` bytes of the encoded `AppToDevice` / `DeviceToApp`. The limits are generated into `core_interface::ble`: `BLE_MAX_CHUNK_BYTES` (244, one ATT value at the requested MTU `BLE_ATT_MTU` of 247) sizes the GATT characteristics, `BLE_MAX_MESSAGE_BYTES` (4096) is the largest reassembled message. A compile-time check ensures the stack's packet pool supports `BLE_ATT_MTU`.

The splitting and reassembly logic is plain code in `core_interface::ble` (`chunks`, `Reassembler`), host-tested without hardware.

**TX (device to app).** `ble_tx_notify_task` encodes each message and sends its chunks back to back. The MTU is read per message (`conn.raw().att_mtu()` is 23 until the exchange finishes), so the task waits up to 3 s for it to reach `BLE_MIN_ATT_MTU` and logs the negotiated value. If the peer negotiated less, the message is dropped with a warning instead of being sent truncated. Messages over `BLE_MAX_MESSAGE_BYTES` are dropped with a `BLE GATT TX message too large` warning. If a notification fails, the link is gone and the task ends; the app discards the partial message.

**RX (app to device).** Writes use the same framing. `gatt_event_task` feeds each non-empty write to a per-connection `Reassembler` and decodes once the last chunk arrives. An empty write is still the pairing probe and is ignored. A gap, a duplicate chunk, a chunk with no start, a timeout (`BLE_REASSEMBLY_TIMEOUT_MS`) or an oversize message drops the partial message with a warning; the partial message also disappears with the connection.

What keeps messages within the limit:
*   **State updates:** each vehicle crate has a `worst_case_state_update_fits_ble_message` test that builds the state update through `core_interface::ble_state_update` with every field at its longest encoding and checks it against `BLE_MAX_MESSAGE_BYTES`. Adding fields to a vehicle's state fails that test before it can fail on a car.

The E-GMP 12 V battery voltage and the four tire pressures are decoded into `EgmpCarState` but not in the proto yet; fragmentation removes the one-packet limit that kept them out.

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

## Removing phones
`RemovePairedPhone` / `ClearPairedPhones` (only accepted while the pairing window is open) queue a `BondOp` for the BLE task, which owns the bonds. It removes each bond from the stack, the `BondStore` flash, the controller's filter accept and resolving lists, and the paired-phone registry in `core-interface`, and disconnects the phone if it is connected. A removed phone can no longer reconnect. The `device_id` is `ble:` followed by the phone's 6-byte address (the same stamp as `source_device_id`).

The registry only mirrors the bonds (restored from flash at startup, updated on pairing and removal), so `ListPairedPhones` reports real bonds. `UpsertPairedPhone` is ignored on ESP: phones enter by pairing.

`max_bonded_phones` applies to BLE bonds: while that many phones are bonded, a new phone is disconnected even with the pairing window open (a bonded phone can still re-pair). The flash store holds at most 8 bonds regardless.

A normal disconnect by the phone after an encrypted session never touches its bond. While the pairing window is open, a disconnect that signals broken keys wipes that phone's bond so it can pair afresh: `PIN_OR_KEY_MISSING`, `AUTHENTICATION_FAILURE`, `CONN_TERMINATED_DUE_TO_MIC_FAILURE`, or a hang-up (`REMOTE_USER_TERMINATED_CONN`) on a link that never got encrypted. The last one is a phone that forgot the bond: Android gives up and hangs up normally, so the app's "tap Pair again" stale-bond recovery depends on it. trouble-host emits no event when a bonded phone re-encrypts and resets the security level on disconnect, so `gatt_event_task` polls `security_level()` until the link is encrypted and remembers it.

## Pairing events
Passkey display/confirm/input and pairing-complete/failure events are handled in the BLE event loop. On `PairingComplete`, new bond data is persisted to flash and added to the controller filter lists. A warning is logged if `security_level` is not encrypted. `tx_auth` is not touched here — TX unblocking is handled entirely by `ble_tx_notify_task`.

## Known-peer reconnect
Bonded phones reconnect in Mode B and re-encrypt via LL_ENC_REQ using the stored LTK. `trouble-host` has no dedicated "encryption changed" event, so `ble_tx_notify_task` detects this by polling `conn.raw().security_level()` on each 50 ms retry until it returns `Encrypted`, then sets `tx_auth = true` and begins sending. iOS never sends `Pairing_Request` after LL_ENC_REQ so `PairingComplete` never fires on that path — this is the exact case the `security_level()` check was introduced to handle.

## Pairing lifecycle controls
`ble_lifecycle_task` implements active-low pairing-button hold detection (`pairing_button_hold_s`) and opens pairing windows using the transport contract value `contracts/opencar/core/v1/transport.toml` → `[ble.pairing].pairing_window_seconds` via `core_interface::open_pairing_window_for(...)`.

**Boards without a pairing button** (`pairing_button_pin` unset, e.g. the WiCAN profile) open the window at boot only while no phone is bonded, so first setup needs no hardware. The transport task reports how many bonds it loaded and the lifecycle task opens the window only for zero. Once a phone is bonded, a reboot (watchdog reset, brownout) leaves the window closed.

Pairing uses `NoInputNoOutput` (Just Works): it has no MITM protection and no user confirmation, so any phone that can bond is trusted from then on. That is why the window must stay closed outside first setup and button presses.

On a buttonless board with a bonded phone there is currently no way to open the window again: managing phones (`RemovePairedPhone`, `ClearPairedPhones`) needs an open window too. A deliberate action for that (a command from a bonded phone, or a physical one) is still to be designed and needs a contract change; until then such a board is reset by erasing its flash.

## Bondable mode policy
Bondability is set to match the pairing-window state at connect time (via `set_bondable(pairing_open)` in the advertise loop) and kept in sync during active connections via `sync_bondable_with_window()` — which keeps bondable `true` until the link is encrypted on first connect so LTKs can be exchanged, then follows the window state.

## Persistence (power-cycle)
Paired-phone data is persisted in flash using `esp-storage` (single sector, checksum + versioned payload) and restored at BLE startup.

LTKs (`BondInformation`) are persisted via `BleSecurityStore` in the second-to-last flash sector using `stack.get_bond_information()`; the sector before that holds the phone-identity store. On startup, `restore_security_store()` loads the bonds and feeds them back via `stack.add_bond_information()` before `stack.build()`, so the phone can re-encrypt without re-pairing after a power cycle.

**Current caveat:** persistence uses fixed end-of-flash regions (last two sectors), not dedicated partitions. Move to partition-table-backed regions when the OTA partition layout is introduced.
