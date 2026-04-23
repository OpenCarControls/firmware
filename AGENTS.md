# Context: Open Car Firmware (Rust)

## 📌 Project Role
This is the embedded Rust monorepo for the hardware controller (ESP32 / Raspberry Pi). It handles CAN bus reading/writing, BLE/WiFi/LTE connections, and OTA updates.

## 🏗️ Architecture: Cargo Workspace & xtask
*   **`core-interface`:** The intermediary between transport layers (BLE/MQTT) and vehicle logic. Knows nothing about specific cars or boards. Uses Embassy throughout (`embassy_time`, `embassy_sync`, `embassy_executor`). Defines 12 static `Channel`s as the inter-crate contract, shared types (`Transport`, `InboundCommand`, `VehicleStatePayload`, `CanRawCapture`, `CanDebugFilter`), and 5 pure-logic `#[embassy_executor::task]` functions (`process_ble_commands_task`, `process_mqtt_commands_task`, `route_responses_task`, `publish_state_task`, `publish_can_debug_task`). Also exposes `pub fn passes_filter()`, `pub fn is_can_read_only()`, `pub fn set_can_read_only(bool)`, CAN debug query functions (`is_can_debug_active`, `can_debug_wants_bus`, `increment_can_debug_dropped`), and 5 inner async helpers (`handle_ble_message`, `handle_mqtt_message`, `route_single_response`, `publish_single_state`, `publish_single_debug_batch`) extracted from the task loops so they can be unit-tested without running an executor. Uses `#![cfg_attr(not(test), no_std)]` so host tests compile without a `no_std` target. Internally organised into six modules: `types`, `channels`, `can`, `can_debug`, `dispatch`, `routing`; all public symbols are re-exported from `lib.rs` and the external API is unchanged.
*   **Board crates (`board-esp`, `board-pc`):** Compiled as `rlib`. Each exports a single `pub fn start(spawner: &Spawner)` that spawns the 4 `core-interface` tasks. Board crates have no knowledge of vehicle logic or protos — they only drive hardware peripherals (radio, CAN, LTE) by reading/writing the edge channels (`BLE_RX_CHANNEL`, `BLE_TX_CHANNEL`, `MQTT_RX_CHANNEL`, `MQTT_TX_CHANNEL`).
*   **Vehicle Crates (e.g., `virtual-car-controller`):** Compiled as `rlib`. Each exports 3 `#[embassy_executor::task]` functions: `handle_basic_commands_task`, `handle_advanced_commands_task`, and `state_update_task`. They read from `BASIC_CMD_CHANNEL` / `ADVANCED_CMD_CHANNEL`, write results to `CMD_RESP_CHANNEL`, and push state updates to `VEHICLE_STATE_CHANNEL`. They depend on `core-interface` for the channel statics and shared types, but never touch board code. Vehicle crates that use CAN also export a `can_rx_task` (reads `CAN_RX_CHANNEL`) and a `pub static CAN_FILTERS: &[CanFilter]` that xtask passes to the board's CAN init functions.
*   **`cars/virtual-car` (`virtual-car-controller`):** A mock vehicle implementation used strictly for UI/UX testing without physical hardware. Contains real Embassy tasks and real proto encoding, just no CAN bus.
*   **`.app_build` (ephemeral, xtask-generated):** The actual binary entry point. Generated fresh by `xtask` before every build from a `main.template.rs` file living in the board's folder. It is the only crate that imports both the board crate and the vehicle crate, wiring them together by spawning all tasks. It is excluded from the Cargo workspace.
*   **Dynamic Builds:** `xtask` reads `config.toml`, resolves the board and vehicle crate paths, reads the vehicle's `contracts/.../meta.toml` to inject the `PLATFORM_ID` constant, and writes `.app_build/Cargo.toml` + `.app_build/src/main.rs` before invoking `cargo build`. The `board` and `platform` values in `config.toml` can be overridden at the CLI with `--board` and `--platform` flags, e.g. `cargo xtask build --board pc`. For ESP builds, the generated `.app_build/Cargo.toml` carries `esp-radio = { version = "0.17.0", features = ["wifi", "ble", "smoltcp", "unstable"] }` and `esp-storage = { version = "0.8.1", features = ["critical-section"] }` as **direct** dependencies so xtask can forward chip features as `esp-radio/{mcu}` and `esp-storage/{mcu}`; it also pins `esp-alloc = "0.9.0"` to match the radio/RTOS stack and avoid allocator conflicts.

## 🔌 Channel Architecture (core-interface)

**Channels are the contract.** No traits connect the board, vehicle, and core — only `static Channel` statics defined in `core-interface`. All crates are compiled independently and cooperate purely through these shared memory queues. All channels use `CriticalSectionRawMutex`.

| Channel | Direction | Type | Notes |
|---|---|---|---|
| `BLE_RX_CHANNEL` | board → core | `AppToDevice` | Raw inbound proto from BLE driver |
| `BLE_TX_CHANNEL` | core → board | `DeviceToApp` | Outbound proto to BLE driver |
| `MQTT_RX_CHANNEL` | board → core | `AppToDevice` | Raw inbound proto from MQTT driver |
| `MQTT_TX_CHANNEL` | core → board | `DeviceToApp` | Outbound proto to MQTT driver |
| `SYSTEM_COMMAND_CHANNEL` | core → board | `SystemCommand` | BLE only; board handles restart etc. |
| `BASIC_CMD_CHANNEL` | core → vehicle | `InboundCommand` | From BLE and MQTT |
| `ADVANCED_CMD_CHANNEL` | core → vehicle | `InboundCommand` | BLE only |
| `CMD_RESP_CHANNEL` | vehicle → core | `(Transport, CommandResponse)` | Transport tag routes response back |
| `VEHICLE_STATE_CHANNEL` | vehicle → core | `VehicleStatePayload` | BLE gets full; MQTT gets basic only |
| `CAN_RX_CHANNEL` | board → vehicle | `CanFrame` | Hardware-received CAN frame |
| `CAN_TX_CHANNEL` | vehicle → board | `CanFrame` | Frame to transmit on a CAN bus |
| `CAN_DEBUG_RX_CHANNEL` | board → core | `CanRawCapture` | Raw (pre-filter) frames for debug streaming; consumed by `publish_can_debug_task` → BLE only |

**`InboundCommand`** carries `{ message_id: u64, transport: Transport, bytes: Vec<u8> }`. The vehicle echoes the `Transport` tag back in `CMD_RESP_CHANNEL` so `route_responses_task` can route the response to the correct radio without the vehicle knowing anything about transports.

**`platform_id`** is a CRC32 of the proto package name (e.g. `0xF7544D7E` for `opencar.cars.virtual_car.v1`), pre-computed and stored in `contracts/.../meta.toml`. xtask injects it as a compile-time constant into `.app_build/src/main.rs`. `core_interface::init(platform_id)` stores it in an `AtomicU32`; the dispatcher tasks validate every inbound message against it and silently drop mismatches.

**MQTT restrictions:** `process_mqtt_commands_task` only forwards `BasicCommandBytes`. `SystemCommand` and `AdvancedCommandBytes` are restricted to BLE and silently dropped from MQTT.

## 🔌 Board / Core-Interface Contract
Embassy is used freely inside `core-interface`. Boards adapt to it — not the other way around.

**Task pattern:** `#[embassy_executor::task]` cannot take generic (`impl Trait`) parameters. The split is:
*   `core-interface` defines all `static Channel`s and `#[embassy_executor::task]` functions containing all protocol logic.
*   Board crates define thin **driver tasks** with concrete hardware types that read/write only the four edge channels (`BLE_RX/TX`, `MQTT_RX/TX`). These are the only hardware-specific tasks.
*   Vehicle crates define their own Embassy tasks that read/write only the four inner channels (`BASIC_CMD`, `ADVANCED_CMD`, `CMD_RESP`, `VEHICLE_STATE`).
*   `.app_build` spawns all tasks from all three layers in `main`.

**Per-subsystem approach:**
*   **CAN:** Vehicle crates define their filter list (`&'static [CanFilter]`). Board crates own all hardware; `CAN_RX_CHANNEL` and `CAN_TX_CHANNEL` are the shared contract. Each board provides driver functions (`init_*` / `run_*_loop`) and async loop tasks that software-filter received frames against the vehicle's filter list. See [CAN Bus Architecture](#-can-bus-architecture) below.
*   **MQTT / networking:** `core-interface` logic uses `embassy-net::TcpSocket`. Each board provides an `embassy-net::driver::Driver` implementation (`esp-wifi` on ESP, tuntap shim on PC).
*   **BLE:** No single cross-platform embassy BLE abstraction exists. `core-interface` defines its own traits (e.g. `trait BleGatt`). ESP implements them with `esp-wifi`'s BLE stack; PC implements them as mocks.

**Board-specific requirements:**
*   **ESP (`board-esp`):** Compiled as `rlib`. `esp-rtos` (Embassy executor + time driver), `esp-alloc` (global heap), `esp-backtrace`, and `RUSTFLAGS="-C link-arg=-Tlinkall.x"` all live in `.app_build`, not in `board-esp` itself, because they must be in the final binary crate. The xtask uses `+esp -Zbuild-std=core,alloc` for xtensa targets. Embassy crate versions must match what `esp-rtos` uses (currently `embassy-executor = "0.9.1"`). CAN driver loops run on **core 1** via `esp_rtos::start_second_core`; comms and vehicle tasks run on core 0. All ESP HAL dependencies are behind an optional `hardware` Cargo feature; firmware builds (via xtask) enable it automatically. Without the feature, the crate compiles on the host for unit testing (only `embedded-can` and `core-interface` are unconditional). Pure logic that is host-testable (e.g. `compute_mcp_masks`) is kept outside `#[cfg(feature = "hardware")]` blocks. WiFi/MQTT is now wired against **`esp-radio 0.17.0`** using the release API: call `esp_radio::init()` first, then `esp_radio::wifi::new(&controller, peripherals.WIFI, esp_radio::wifi::Config::default())`, configure the client with `ModeConfig::Client(ClientConfig::default().with_ssid(String::from(ssid)).with_password(String::from(password)))`, and use `interfaces.sta` (not `station`). **Radio init is split from WiFi init:** `board_esp::init_radio()` initialises the `esp_radio::Controller<'static>` inside a `StaticCell` and returns a `&'static SharedRadioController`; it must be called exactly once (before BLE or WiFi). `board_esp::init_wifi` now takes `(radio: &'static SharedRadioController, spawner, peripherals.WIFI, WIFI_SSID, WIFI_PASSWORD)` — radio is passed in so BLE and WiFi share the same controller without double-initialising. **WiFi can be disabled** via `[network.wifi] enabled = false` in `config.toml`; when disabled, xtask omits the WiFi/MQTT constants (`WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_*`) and the `mqtt_driver_task` spawn from the generated `main.rs`, but `init_radio()` and `ble_transport_task` are always generated. With `embassy-net 0.9`, the stack handle is `Stack<'static>` and TCP sockets are created with `TcpSocket::new(*stack, ...)`. In the final binary crate, `esp-rtos` must enable `features = ["embassy", "esp-radio", "esp-alloc"]`; omitting `esp-radio` leads to unresolved `esp_rtos_*` linker errors.

### BLE Implementation Status (ESP)
*   **Host stack / versions:** BLE uses `esp-radio` + `trouble-host 0.5.x` (kept at `0.5.x` to stay compatible with `bt-hci 0.6` used by `esp-radio 0.17`). `trouble-host` is built with `derive`, `peripheral`, `gatt`, `security`, and `default-packet-pool-mtu-255`.
*   **GATT transport:** `ble_transport_task` runs a full advertise/connect loop and bridges GATT RX/TX characteristics to `BLE_RX_CHANNEL` / `BLE_TX_CHANNEL`. It accepts a `name_base: &'static str` 4th parameter (injected by the generated `main.rs` as `BLE_DEVICE_NAME_BASE`).
*   **Device name:** Configurable base name via `[hardware.esp.ble] device_name` in `config.toml` (default `"OpenCar"`). At runtime `ble_transport_task` appends a 2-byte MAC suffix using `build_ble_device_name(base, mac)` → `"{base}-{mac[4]:02X}{mac[5]:02X}"` (e.g. `"OpenCar-ABCD"`). The result is stored in a `StaticCell<heapless::String<33>>` for the `'static` lifetime required by `PeripheralConfig`.
*   **UUIDs / advertise data:** The OpenCar 128-bit service UUID is included in scan response (`AdStructure::ServiceUuids128`) so phone apps can filter devices by service.
*   **Stable BLE identity:** Device address is derived from efuse base MAC (`Efuse::read_base_mac_address()`) and converted to a static-random BLE address (`mac_to_ble_address`) so identity is stable across reboots.
*   **Security enforcement model:** `trouble-host 0.5` derive macros do not support characteristic-level `permissions(...)`, so security is enforced at runtime in layers:
	*   RX writes are rejected with `AttErrorCode::INSUFFICIENT_ENCRYPTION` when link is not encrypted.
	*   After the encryption check, writes are further rejected with `AttErrorCode::INSUFFICIENT_AUTHORISATION` (British spelling — that is the constant name in `trouble-host`) if the peer is not in the paired-phone registry AND the pairing window is currently closed.
	*   `source_device_id` in every accepted inbound proto is **overwritten** with the firmware-verified BLE peer address; any app-supplied value is discarded. This makes `source_device_id` a server-side stamp, not a client claim.
	*   TX notifications are blocked until link encryption is active.
	*   `request_security()` is used to trigger pairing/encryption when needed.
*   **TX buffering during encryption:** `ble_tx_notify_task` holds a `pending_msg: Option<DeviceToApp>` buffer. If a message arrives from `BLE_TX_CHANNEL` before the link is encrypted, it is stored in `pending_msg` (not dropped) and retried every 50 ms until encryption becomes active.
*   **Pairing events:** Passkey display/confirm/input and pairing-complete/failure events are handled in the BLE event loop. Pairing-complete updates the paired-phone registry.
*   **Known-peer reconnect:** When a previously paired phone reconnects, `ble_transport_task` detects the peer via `is_phone_paired(&peer_id)` and automatically calls `open_pairing_window_for(BLE_REPAIR_WINDOW_S)` (30 s) so the link can re-encrypt without the user pressing the pairing button again.
*   **Pairing lifecycle controls:** `ble_lifecycle_task` implements active-low pairing-button hold detection (`pairing_button_hold_s`) and opens pairing windows (`pairing_window_s`) via `core_interface::open_pairing_window_for(...)`.
*   **Bondable mode policy:** Bondability is synced to pairing-window state both at connect time and during active connections.
*   **Persistence (power-cycle):** Paired-phone data is persisted in flash using `esp-storage` (single sector, checksum + versioned payload) and restored at BLE startup.
*   **Current caveat:** Persistence currently uses a fixed end-of-flash region, not a dedicated partition. Move to a partition-table-backed region when OTA partition layout is introduced. `trouble-host 0.5` has no public `BondStore`/`SecurityManager` API — LTK cannot be persisted; the known-peer reconnect window is the accepted workaround.
*   **PC (`board-pc`):** Compiled as `rlib`. `embassy-executor` with `arch-std` + `executor-thread` features, `embassy-time` with `std` feature, and `critical-section` with `std` feature all live in `.app_build`. No `tokio` — embassy's std driver covers timing and critical sections. CAN is implemented via SocketCAN (`socketcan` crate) using an `#[embassy_executor::task(pool_size = 4)]` per bus. Frame conversion between `socketcan` and `core-interface` types is extracted into `pub(crate) fn socketcan_to_core_frame` / `core_to_socketcan_frame` helpers for unit testing.

**`main.template.rs`:** Each board folder contains a `main.template.rs` with placeholder tokens (`{PLATFORM_ID}`, `{VEHICLE_CRATE_IDENT}`, `{CAN_TX_PIN}`, `{MTLS_CERTS}`, etc.). xtask reads this file and substitutes the tokens to produce `.app_build/src/main.rs`. Edit the template to change the entry-point structure; edit the builder to change what gets substituted.

**`tests.template.rs` (ESP only):** `boards/esp/tests.template.rs` is the counterpart for on-hardware testing. xtask generates `.app_test_build/src/main.rs` from it, injects `{PLATFORM_ID}` and the contents of `boards/esp/tests/hardware.rs` (`{ON_HARDWARE_TESTS}`), then compiles and flashes via `probe-rs`. The template uses `embedded-test` with the Embassy executor.

## 🚌 CAN Bus Architecture

**Shared channels** (defined in `core-interface`):

| Channel | Direction | Type |
|---|---|---|
| `CAN_RX_CHANNEL` | board → vehicle | `CanFrame` |
| `CAN_TX_CHANNEL` | vehicle → board | `CanFrame` |

**`CanFrame`** carries `{ bus_id: u8, id: embedded_can::Id, data: [u8; 8], dlc: u8 }`. `bus_id` is a 0-based index matching the order of `[[hardware.esp.can_buses]]` / `[[hardware.pc.can_buses]]` entries in `config.toml`.

**`CanFilter`** carries `{ bus_id: u8, id: embedded_can::Id, mask: u32 }`. Vehicle crates export a `pub static CAN_FILTERS: &[CanFilter]` that xtask passes to the board's init functions. A frame passes if `(frame_id_raw & mask) == (filter_id_raw & mask)`.

**Filter strategy:** Hardware filters are set to accept-all where driver constraints apply (TWAI on ESP). All matching is done in software inside the driver loop against the vehicle-supplied filter list. MCP2515 also programs hardware RX masks/filters from the same list as a first-pass optimisation.

**ESP two-core split:**

| Core | Tasks |
|---|---|
| Core 1 (`esp_rtos::start_second_core`) | `run_twai_loop`, `run_mcp2515_loop` — all CAN I/O |
| Core 0 (`#[esp_rtos::main]` executor) | BLE/MQTT comms, vehicle logic, `core-interface` plumbing |

**ESP CAN driver modes:**
*   **TWAI (built-in):** `Twai<'static, esp_hal::Blocking>`. `esp_hal::Async` is `!Send` (`PhantomData<*const ()>`) and cannot cross the core boundary. The loop uses `nb` polling with a 1 ms `embassy_time::Timer` yield when no frame is available.
*   **MCP2515 (SPI):** `MCP2515<ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>>`. RX is interrupt-driven via `int_pin.wait_for_falling_edge().await` (async GPIO, which IS `Send`). `CanSpeed` / `McpSpeed` are re-exported from `board_esp` so the generated `main.rs` can reference them without a direct `mcp2515` dep.

**PC CAN driver:** `socket_can_task` (pool of 4) opens a non-blocking `SocketCAN` socket per bus and polls it every 1 ms via Embassy timer. Outbound frames are drained from `CAN_TX_CHANNEL` after each RX sweep.

**xtask CAN codegen** (in `xtask_builder.rs` per board):
*   Reads `[[hardware.<board>.can_buses]]` entries from `config.toml`.
*   For each entry emits peripheral/pin construction code + an `init_*` call into `{CAN_HARDWARE_INIT}`.
*   Emits the corresponding `s.spawn(run_*_loop(...))` call into `{CORE1_TASK_SPAWNS}` (ESP) or `spawner.spawn(socket_can_task(...))` (PC).
*   MCP2515 entries require `interface = "mcp2515"`, `cs_pin`, `clk_pin`, `mosi_pin`, `miso_pin`, `int_pin`, `can_speed`, and `mcp_speed` fields. The SPI peripheral is auto-assigned by xtask (starting from SPI2). TWAI entries require `interface = "twai"`, `tx_pin`, and `rx_pin` fields. The TWAI peripheral is hardcoded as `TWAI0`.

## 🧪 Testing

**Command:** `cargo xtask test` — runs all host-side tests in sequence without hardware.

**Host tests** (run with standard `cargo test`, no ESP toolchain needed):

| Crate | Tests | Command |
|---|---|---|
| `core-interface` | 53 — `passes_filter`, BLE/MQTT dispatch (platform_id gating, routing, `source_device_id` stamping, empty-source rejection), response routing, state publish split, CAN read-only flag, CAN debug streaming (batch filtering, blocklist, dropped-frame counter) | `cargo test -p core-interface -- --test-threads=1` |
| `virtual-car-controller` | 19 — `encode_state` proto encoding, CAN frame → state updates, `process_basic_command` | `cargo test -p virtual-car-controller -- --test-threads=1` |
| `board-pc` | 8 — `socketcan_to_core_frame` and `core_to_socketcan_frame` round-trips | `cargo test -p board-pc -- --test-threads=1` |
| `board-esp` | 14 — `compute_mcp_masks` + BLE helpers/proto size & round-trip tests (`mac_to_ble_address`, UUID uniqueness, `AppToDevice`/`DeviceToApp` encode/decode bounds, `ble_device_name_appends_mac_suffix`) | `cargo test -p board-esp -- --test-threads=1` |

`--test-threads=1` is required — Embassy channels are process-global statics; parallel threads corrupt each other's state.

**Verified builds:** `cargo xtask build --board pc` and `cargo xtask build --board esp` both succeed in the current repo state.

**On-hardware tests** (ESP32, requires `probe-rs`):
```
cargo xtask test --board esp --on-hardware
```
xtask generates `.app_test_build/` (parallel to `.app_build/`), compiles it with the Xtensa toolchain using `embedded-test` + `defmt-rtt`, and flashes/runs via `probe-rs test`. The template (`boards/esp/tests.template.rs`) includes channel round-trip and `passes_filter` smoke tests. Add custom on-device tests to `boards/esp/tests/hardware.rs` — xtask injects that file automatically.

**`no_std` / test gating pattern:**
*   `core-interface` and all vehicle crates use `#![cfg_attr(not(test), no_std)]`.
*   `board-esp` uses `#![cfg_attr(not(test), no_std)]` — always `no_std` except during host tests. HAL deps are optional behind the `hardware` feature (enabled by xtask for firmware builds). Only allocation-free, HAL-free code is available without the feature.
*   `board-pc` is always `std`.

**CI:** The `test` job in `.github/workflows/ci.yml` uses git diff to detect which crate directories changed and runs only the affected packages. Changes to `boards/pc/` trigger `board-pc` tests; changes to `boards/esp/` trigger `board-esp` tests; changes to `core-interface/` trigger all four crates.

## 🔒 CAN Read-Only Mode

**Purpose:** Prevent accidental CAN TX until the vehicle crate has positively identified the connected car.

**Flag location:** `core-interface` — `static CAN_READ_ONLY: AtomicBool = AtomicBool::new(true)`. Defaults to `true` (locked) at boot so no frame can reach the bus before validation.

**Public API (in `core-interface`):**
*   `pub fn is_can_read_only() -> bool` — vehicle tasks call this to check whether TX is currently blocked.
*   `pub fn set_can_read_only(enabled: bool)` — vehicle tasks call this to unlock (`false`) after validating inbound CAN frames confirm the correct car, or re-lock (`true`) if an error or inconsistent data is detected at any time.

**Enforcement:** The drop happens at the board TX drain loops — not at `CAN_TX_CHANNEL` enqueue. Vehicle code is free to push frames to `CAN_TX_CHANNEL` at any time; the board loop checks `is_can_read_only()` for each frame addressed to its `bus_id` and silently discards it if the flag is set. Three sites enforce this:
*   `board-pc` — `socket_can_task` TX arm (before `socket.write_frame()`)
*   `board-esp` — `run_twai_loop` TX arm (before the `tx.transmit()` spin-loop)
*   `board-esp` — `run_mcp2515_loop` TX arm (before `driver.send_message()`)

**Intended vehicle workflow:**
1. Boot: `CAN_READ_ONLY = true`. Vehicle receives CAN frames via `can_rx_task`.
2. Vehicle validates frame IDs, data ranges, or handshake frames against its expected car profile.
3. On success: `set_can_read_only(false)` — CAN TX is now live.
4. On any subsequent error or unexpected data: `set_can_read_only(true)` re-engages the lock.

## 🏷️ Versioning & CI/CD
*   **Independent Crates:** Managed via `release-plz` (not implemented yet, waiting for at least one car implementation to work). Changes to the core cascade safely to vehicle crates.
*   **Tagging:** Uses prefixed tags (e.g., `car-hmg-v0.2.1`).
*   **Agnostic Pipelines:** GitHub Actions trigger on `*-v*.*.*`. The YAML dynamically parses the tag to extract the car name, passes it to `xtask`, builds the `.bin`, and uploads it to GitHub Releases.

## 📡 OTA Manifest Generation
The CI/CD pipeline generates static `manifest.json` files hosted on GitHub Pages. This includes a Compatibility Matrix (`min_app_version`) to ensure the hardware is never updated to a Protobuf version the phone app cannot decode.
