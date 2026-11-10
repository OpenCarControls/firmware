# BLE (ESP)

Implementation notes for the BLE transport in `boards/esp/src/ble/`. The PC board only mocks BLE. For how BLE fits in the overall design, see [Architecture](architecture.md).

## Stack and versions
BLE uses `esp-radio 0.18` + `trouble-host 0.6.0` (with `bt-hci 0.8.1`), built with features `derive`, `security`, `default-packet-pool-mtu-255`.

`trouble_host::new(controller, resources)` returns `Stack<'resources, C, P>` (outer variable); `stack.build()` takes `&'stack self` and returns non-exhaustive `Host { peripheral, runner, .. }` — the `Stack` is still usable after `build()` for `get_bond_information()`, `add_bond_information()`, and `command()`.

## GATT transport
`ble_transport_task` runs a full advertise/connect loop and bridges GATT RX/TX characteristics to `BLE_RX_CHANNEL` / `BLE_TX_CHANNEL`, and sends the newest vehicle state from `BLE_STATE_TX` after anything waiting in `BLE_TX_CHANNEL`. It reports the link with `set_link_up(Transport::Ble, …)` when a phone connects and when the connection closes; closing clears `BLE_TX_CHANNEL`, so the next phone never gets what was queued for the previous one (see [Architecture](architecture.md#transport-links-and-responses)). It accepts a `name_base: &'static str` 4th parameter (injected by the generated `main.rs` as `BLE_DEVICE_NAME_BASE`).

## Message size and fragmentation
Messages larger than one ATT packet are split into chunks, as defined by the contract (`ble.payload` and `ble.fragmentation` in `transport.toml`, explained in `contracts/docs/ble-fragmentation.md`). Every write and notification is one chunk: a 2-byte header (byte 0: bit 7 = last chunk, bits 0-6 = chunk index; byte 1: message sequence number) and up to `negotiated MTU - 5` bytes of the message stream, which is the encoded `AppToDevice` / `DeviceToApp` followed by its CRC-32. The limits are generated into `core_interface::ble`: `BLE_MAX_CHUNK_BYTES` (244, one ATT value at the requested MTU `BLE_ATT_MTU` of 247) sizes the GATT characteristics, `BLE_MAX_MESSAGE_BYTES` (4096, CRC not included) is the largest reassembled message. A compile-time check ensures the stack's packet pool supports `BLE_ATT_MTU`.

The splitting and reassembly logic is plain code in `core_interface::ble` (`chunks`, `Reassembler`), host-tested without hardware.

**TX (device to app).** `ble_tx_notify_task` encodes each message and sends its chunks back to back, with a sequence number that increments per message. The MTU is read per message (`conn.raw().att_mtu()` is 23 until the exchange finishes), so the task waits up to 3 s for it to reach `BLE_MIN_ATT_MTU` and logs the negotiated value. If the peer negotiated less, the message is dropped with a warning instead of being sent truncated. Messages over `BLE_MAX_MESSAGE_BYTES` are dropped with a `BLE GATT TX message too large` warning. If a notification fails, the link is gone and the task ends; the app discards the partial message.

**RX (app to device).** Writes use the same framing. `gatt_event_task` feeds each non-empty write to a per-connection `Reassembler` and decodes once the last chunk arrives. An empty write is still the pairing probe and is ignored. A gap, a duplicate chunk, a chunk with no start, a sequence number that changes mid-message, a timeout (`BLE_REASSEMBLY_TIMEOUT_MS`), an oversize message or a CRC mismatch drops the partial message with a warning; the partial message also disappears with the connection.

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
*   `source_device_id` in every accepted inbound proto is **overwritten** with the firmware-verified BLE peer address; any app-supplied value is discarded. This makes `source_device_id` a server-side stamp, not a client claim. For a bonded phone it is the bond's identity address, the one the paired-phone registry holds, even when the phone connects with a resolvable private address: a known bond is matched at connect, and a bond stored on the connection (first pairing) switches the stamp to it at once, so a phone that has just paired is recognised as bonded on that same connection.
*   TX notifications are withheld until `ble_tx_notify_task` confirms link encryption via `conn.raw().security_level()` (see below).
*   `request_security()` is used to trigger pairing/encryption when needed.

## TX buffering during encryption
`ble_tx_notify_task` holds a `pending_msg: Option<DeviceToApp>` buffer. If a message arrives from `BLE_TX_CHANNEL` or `BLE_STATE_TX` before the link is encrypted, it is stored in `pending_msg` (not dropped) and retried every 50 ms. The buffer belongs to the connection and goes with it.

Encryption is checked via a per-connection `tx_auth: AtomicBool` (initialized `false`). On each retry while `tx_auth` is `false`, `ble_tx_notify_task` calls `conn.raw().security_level()` — if the result is `Encrypted`, it sets `tx_auth = true` (cached for all subsequent messages) and sends immediately. This single check covers both the first-pair path (encryption established during `PairingComplete`) and the reconnect path (re-encryption via stored LTK, which never fires `PairingComplete` on iOS). `gatt_event_task` does not touch `tx_auth`.

## State on subscribe
trouble-host's `notify` sends nothing to a peer that hasn't enabled notifications (and still returns `Ok`), and the app subscribes only after the link is encrypted, so the first state update after connecting would otherwise be lost. `publish_single_state` keeps the last BLE state message in `core_interface::latest_ble_state()` (unlike `BLE_STATE_TX`, it isn't emptied when sent). When the app writes the TX characteristic's CCCD with notifications enabled, `gatt_event_task` signals `ble_tx_notify_task`, which sends that cached state (full state, original timestamp) through the normal path, `SUBSCRIBE_RESEND_DELAY_MS` (500 ms) later: a notification sent right as the CCCD write is accepted was lost on Android, before the app's notification stream was listening. It is still held until the link is encrypted. Nothing is sent if the vehicle hasn't published a state since boot. The CAN access state (`SystemState.can_access`, sent with the connected phone's `SystemState.phone_access`, see [Phone trust](#phone-trust)) and the command availability (`SystemState.command_availability`) are sent again too (`resend_can_access_state()`, `resend_command_availability()`), and both are repeated every few seconds anyway (see [CAN](can.md#read-write-policy) and [Vehicle commands](commands.md#command-availability)). A message the app was already subscribed for may arrive twice; state updates are idempotent.

## MQTT client ID
The access state update also carries `SystemState.mqtt_client_id`, the `network.mqtt.client_id` this device's topics are built from, and it is unset in a build without MQTT. The app can't know it otherwise, and needs it to use this device's topics. So give each device its own `client_id`.

## CAN read-write commands
`SetCanReadWriteApprovalCommand` and `SetCanDevWriteUnlockCommand` are handled in `core-interface`'s dispatcher, not forwarded to the board: only from a phone in the paired-phone registry (checked against the stamped `source_device_id`), and answered with a `CommandResponse`. Once a phone is validated for the identified car, the approval needs a validated phone (see [Phone trust](#phone-trust)). Unlike phone management they don't need the pairing window. See [CAN](can.md#read-write-policy) for the policy.

## Phone trust
Pairing is Just Works, so bonding during a pairing window doesn't prove the phone belongs to the car's owner. A phone proves it by doing something in the car that needs the car itself. Three levels, each needing the ones above:

| Level | Scope | Allows | Granted by |
|---|---|---|---|
| Bonded | phone | System commands | Pairing during an open window |
| Approved | device + car (`CarDigest`) | The platform's own frames that don't act on the car ([TX classes](can.md#tx-classes)) | The owner's CAN read-write approval, given once at first setup |
| Validated | phone + car | Vehicle commands (every basic and advanced command) | An in-car action the vehicle defines |

A vehicle command needs all three, plus the dev lock in development builds. The dispatcher refuses a command from a phone that isn't validated with `COMMAND_STATUS_CODE_REJECTED_NOT_VALIDATED`, after the read-only check, and hands the command it lets through a `CommandAuthorization` for its phone. The board's TX check reads the validation again when the command's frames are sent ([TX classes](can.md#tx-classes)). MQTT has no phone identity and never gets an authorization; when MQTT commands are authenticated (contracts#8), a phone will have to be validated in the car first.

**Validation session** (`validation.rs`), run by `core-interface` with methods from the vehicle:

*   The vehicle declares its methods once (`declare_validation_methods`): a stable `method_id` the app maps to its text, the kind (code entry: the car shows a code the owner types in the app; in-car action: the owner does something the vehicle detects) and whether it needs the approval (a code shown on the cluster sends frames to the car, so it only starts while the device is read-write). The vehicle also says live whether the car's current state allows each method (`is_available`), which it decides itself: the app reads it from `available` when it lists the methods, a session can't start without it, and it doesn't affect a running session. What a running session doesn't survive is the vehicle's call too (`can_continue`).
*   A bonded phone lists them (`ListPhoneValidationMethods`, with whether each can start now) and starts one (`StartPhoneValidation`). The exact car must be identified, since the validation belongs to it. The session belongs to the phone that started it.
*   The vehicle follows the running challenge (`validation_challenge_receiver` / `current_validation_challenge`): for a code entry method it shows the code, for an in-car action it watches for the action and reports it with `complete_validation_action(session)`. The vehicle never says which phone is validated; `core-interface` validates the session's phone.
*   The limits are contract values, `[ble.phone_validation]` in `transport.toml`, so the app shows the same ones; `core-interface`'s build script generates them (`VALIDATION_SESSION_MS`, `CODE_LENGTH`, `MAX_CODE_ATTEMPTS`, `VALIDATION_COOLDOWN_MS`, `VALIDATION_RESULT_KEEP_MS`) and checks their ranges.
*   One session at a time, `session_seconds` at most. A code entry session uses a fresh `code_length`-digit code from the board's random source (`set_random_source`: the ESP hardware RNG, `/dev/urandom` on PC; code entry is refused without one) and accepts `max_code_attempts` wrong codes (`SubmitPhoneValidationCode`). The session then fails and no session can start for `cooldown_seconds`. A phone that starts again replaces its own session; another phone is refused while it runs. A session ends early on `CancelPhoneValidation`, when the identified car changes, when its method needs read-write mode and the device goes read-only (the approval revoked, a logging session), or when the vehicle says it can't go on.
*   A refused validation command answers with data the app can act on, since the `error_message` is only for logs: the session and its attempts left after a wrong code, otherwise a `PhoneValidationRefusal` with the reason and, during the cooldown, the seconds left. Any phone gets them, including one that didn't cause the cooldown.
*   The connected phone sees its trust level, whether the car has a validated phone, and its session (while it runs and for `result_keep_seconds` after it ends) in `SystemState.phone_access`, sent with the CAN access state. `ListPairedPhones` is answered by `core-interface` with each phone's trust level and which one asked.

**Bound to the car.** Validations are stored with the `CarDigest` they were made in and only apply while that car is identified. On another car of the same platform they stop applying; a bonded phone then does the first setup there, and approving that car erases every validation (the phones stay bonded and validate again). Approving the same car again keeps them. A validation also ends when its phone's bond is removed, when the phone pairs again (new keys may be another phone claiming its address), and when the bonds are reset or cleared. Validations don't expire otherwise.

**Who controls what.** Once a phone is validated for the identified car, only a validated phone can change (approve or revoke) the approval, remove a validated phone or clear the phones; a merely bonded phone gets `REJECTED_NOT_VALIDATED`, so it can't lock the owner out. Before that, any bonded phone can, which is the first setup. The dev lock only needs a bonded phone.

**The connected phone.** The ESP has one BLE connection at a time, so what it publishes over BLE is for that phone: `gatt_event_task` reports its stamped `source_device_id` with `set_ble_peer` at connect (and again once a first pairing stores its bond), and the link going down clears it. The command availability is unavailable while that phone isn't validated (see [Vehicle commands](commands.md#command-availability)). The PC mock has no connections and takes the last phone that sent a command.

The platform-specific methods are added per vehicle (E-GMP: a [code shown on the cluster](can.md#e-gmp-cluster-code), in Ready with media playing); a vehicle that declares none can't validate a phone, so none of its commands run. virtual-car's simulated methods are described in [CAN](can.md#car-identity).

## Removing phones
`RemovePairedPhone` / `ClearPairedPhones` (only accepted while the pairing window is open) queue a `BondOp` for the BLE task, which owns the bonds. Removing a validated phone, or clearing them while one is validated, needs a validated phone; the dispatcher answers a refused one with `REJECTED_NOT_VALIDATED` and doesn't forward it. It removes each bond from the stack, the `BondStore` flash, the controller's filter accept and resolving lists, and the paired-phone registry in `core-interface`, and disconnects the phone if it is connected. A removed phone can no longer reconnect. The `device_id` is `ble:` followed by the phone's 6-byte address (the same stamp as `source_device_id`).

The registry only mirrors the bonds (restored from flash at startup, updated on pairing and removal), so `ListPairedPhones` reports real bonds; `core-interface` answers it with each phone's trust level. Removing a phone from the registry also ends its validation. `UpsertPairedPhone` is ignored on ESP: phones enter by pairing.

`max_bonded_phones` applies to BLE bonds: while that many phones are bonded, a new phone is disconnected even with the pairing window open (a bonded phone can still re-pair). The flash store holds at most 8 bonds regardless.

A normal disconnect by the phone after an encrypted session never touches its bond. While the pairing window is open, a disconnect that signals broken keys wipes that phone's bond so it can pair afresh: `PIN_OR_KEY_MISSING`, `AUTHENTICATION_FAILURE`, `CONN_TERMINATED_DUE_TO_MIC_FAILURE`, or a hang-up (`REMOTE_USER_TERMINATED_CONN`) on a link that never got encrypted. The last one is a phone that forgot the bond: Android gives up and hangs up normally, so the app's "tap Pair again" stale-bond recovery depends on it. trouble-host emits no event when a bonded phone re-encrypts and resets the security level on disconnect, so `gatt_event_task` polls `security_level()` until the link is encrypted and remembers it.

## Pairing events
Passkey display/confirm/input and pairing-complete/failure events are handled in the BLE event loop. On `PairingComplete`, new bond data is persisted to flash and added to the controller filter lists. A warning is logged if `security_level` is not encrypted. `tx_auth` is not touched here — TX unblocking is handled entirely by `ble_tx_notify_task`.

## Known-peer reconnect
Bonded phones reconnect in Mode B and re-encrypt via LL_ENC_REQ using the stored LTK. `trouble-host` has no dedicated "encryption changed" event, so `ble_tx_notify_task` detects this by polling `conn.raw().security_level()` on each 50 ms retry until it returns `Encrypted`, then sets `tx_auth = true` and begins sending. iOS never sends `Pairing_Request` after LL_ENC_REQ so `PairingComplete` never fires on that path — this is the exact case the `security_level()` check was introduced to handle.

## Pairing lifecycle controls
`ble_lifecycle_task` implements active-low pairing-button hold detection (`pairing_button_hold_s`) and opens pairing windows using the transport contract value `contracts/opencar/core/v1/transport.toml` → `[ble.pairing].pairing_window_seconds` via `core_interface::open_pairing_window_for(...)`.

**Boards without a pairing button** (`pairing_button_pin` unset, e.g. the WiCAN profile) open the window at boot only while no phone is bonded, so first setup needs no hardware. That first-setup window lasts `[ble.pairing].first_setup_pairing_window_seconds` (longer than the button window, so the owner has time to plug the device in, open the app and finish the wizard; bounded so an abandoned setup doesn't stay open). The transport task reports how many bonds it loaded and the lifecycle task opens the window only for zero; the transport starts advertising only once that is decided, so a phone already retrying a connection at boot finds the window open. Once a phone is bonded, a reboot (watchdog reset, brownout) leaves the window closed; see [Bond reset by power-cycling](#bond-reset-by-power-cycling) for an owner whose phone lost its bond.

That first-setup window closes as soon as the first bond is stored (and kept, i.e. not revoked by `max_bonded_phones`), instead of running its full timer, so a second phone can't bond in the time left. A stranger who bonds first makes the owner's pairing fail visibly instead of both phones ending up bonded. Because the window is then closed, the broken-bond wipe below no longer applies to that bond: if the phone loses its side of it, recovery is the bond reset by power-cycling below. Windows opened with the pairing button are not closed early.

Pairing uses `NoInputNoOutput` (Just Works): it has no MITM protection and no user confirmation, so any phone that can bond is trusted with system commands from then on. That is why the window must stay closed outside first setup and button presses. Vehicle commands also need the phone to be validated in the car (see [Phone trust](#phone-trust)).

### Bond reset by power-cycling
On a buttonless board with a bonded phone nothing else opens the window again, and managing phones (`RemovePairedPhone`, `ClearPairedPhones`) needs an open window too. An owner whose phone lost its bond (forgotten in Android's Bluetooth settings, app reinstalled, new phone) can't even find the device, since it advertises without its name or service UUID. Power-cycling it quickly resets its bonds instead: after `[ble.pairing].power_cycle_reset_short_boots` power-ons in a row that each lose power within `power_cycle_reset_uptime_seconds` of booting, the next power-on clears every bond. The board then has no bond, so it opens the first-setup window as a new device does. The app's recovery instructions use the same contract values.

*   The count is kept in flash (`ble/power_cycle.rs` and `BondStore`): the BLE task bumps it before loading the bonds, and stores 0 once the boot passes the uptime threshold, from the advertise loop or, while a phone is connected, from `gatt_event_task`. The advertise loop only checks between cycles, so a boot that lasts a little past the threshold can still count as short.
*   Only power-on resets count (`SocResetReason::ChipPowerOn`). Any other reset (watchdog, panic, brown-out detector, software) stores 0, so a crash loop between power cycles can't add up to a reset. The power-on code is shared with a chip-level brown-out, which is a real loss of power, and with the super watchdog, which esp-hal disables at init.
*   Clearing the bonds, rather than only opening the window, also removes the lost phone's stale bond, which would otherwise hold a `max_bonded_phones` slot. It also ends every phone validation (`prune_phone_validations` at boot drops those of phones no longer bonded). The CAN read-write approval is kept: it only covers the platform's frames that don't act on the car, so whoever pairs again still has to validate in the car before any vehicle command.
*   Boards with a pairing button don't have the gesture (the generated `main.rs` passes `None`): the button opens the window, and `ClearPairedPhones` works in it.
*   Someone near the car can't trigger it without cutting the device's power, and the first-setup window still closes as soon as the first phone bonds.

## Bondable mode policy
Bondability is set to match the pairing-window state at connect time (via `set_bondable(pairing_open)` in the advertise loop) and kept in sync during active connections via `sync_bondable_with_window()` — which keeps bondable `true` until the link is encrypted on first connect so LTKs can be exchanged, then follows the window state.

## Persistence (power-cycle)
Paired-phone data is persisted in flash using `esp-storage` (single sector, checksum + versioned payload) and restored at BLE startup.

LTKs (`BondInformation`) are persisted by `BondStore` (`sequential-storage` map keyed by the phone's address) in the last two flash sectors. On startup the BLE task loads the bonds and feeds them back via `stack.add_bond_information()` before `stack.build()`, so the phone can re-encrypt without re-pairing after a power cycle.

The two sectors before them hold the owner's CAN read-write approval (`StoredApproval`: a format version and the approved car's digest, see [CAN](can.md#read-write-policy)) and, under a second key, the validated phones (the bytes of `encode_phone_validations()`: a format version, the car digest and the device IDs, see [Phone trust](#phone-trust)). The BLE task owns the flash, so it restores both before the stack starts (pruning the validations once the bonds are loaded) and writes each change `core-interface` hands it (`can_read_write_approval_to_store()`, `phone_validations_to_store()`), from the advertise loop or, during a connection, from `gatt_event_task`. If the range can't be written it is erased and both items are written again from their current values. Erasing the flash clears the bonds, the approval and the validations. The PC board keeps the validations in the file `[hardware.pc] phone_validations_file`.

The two sectors before the approval hold the count of short boots for the [bond reset by power-cycling](#bond-reset-by-power-cycling) (a single `u8`). It has its own range so that a write failing on it (which erases that range) can never touch the bonds or the approval.

**Current caveat:** persistence uses fixed end-of-flash regions (last six sectors), not dedicated partitions. Move to partition-table-backed regions when the OTA partition layout is introduced.
