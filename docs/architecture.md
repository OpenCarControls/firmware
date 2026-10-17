# Architecture

Open Car Firmware is an embedded Rust monorepo for the hardware controller (ESP32 / Raspberry Pi). It handles CAN bus reading/writing, BLE/WiFi/LTE connections, and OTA updates.

Related docs: [BLE](ble.md) · [CAN bus](can.md) · [Testing](testing.md) · [Releases & OTA](releases.md) · [Hardware setup](hardware-setup.md)

## Cargo workspace & xtask

### `core-interface`
The intermediary between transport layers (BLE/MQTT) and vehicle logic. Knows nothing about specific cars or boards. Uses Embassy throughout (`embassy_time`, `embassy_sync`, `embassy_executor`).

*   Defines 12 static `Channel`s as the inter-crate contract (see [Channels](#channels)).
*   Defines shared types: `Transport`, `InboundCommand`, `VehicleStatePayload`, `CanRawCapture`, `CanDebugFilter`.
*   Defines 5 pure-logic `#[embassy_executor::task]` functions: `process_ble_commands_task`, `process_mqtt_commands_task`, `route_responses_task`, `publish_state_task`, `publish_can_debug_task`.
*   Exposes `pub fn passes_filter()`, `pub fn is_can_read_only()`, `pub fn set_can_read_only(bool)`, and the CAN debug query functions (`is_can_debug_active`, `can_debug_wants_bus`, `increment_can_debug_dropped`).
*   Has 5 inner async helpers (`handle_ble_message`, `handle_mqtt_message`, `route_single_response`, `publish_single_state`, `publish_single_debug_batch`) extracted from the task loops so they can be unit-tested without running an executor.
*   **`publish_single_state` is intentionally non-blocking:** it uses `try_send` on both `BLE_TX_CHANNEL` and `MQTT_TX_CHANNEL`; if either is full it evicts the oldest pending state update and enqueues the fresher one. This guarantees `publish_state_task` never stalls, keeping `VEHICLE_STATE_CHANNEL` always drainable and preventing a deadlock cascade through `state_update_task` and command-handler tasks.
*   Uses `#![cfg_attr(not(test), no_std)]` so host tests compile without a `no_std` target.
*   Internally organised into modules (`types`, `channels`, `can`, `can_debug`, `dispatch`, `routing`, …); all public symbols are re-exported from `lib.rs`.

### Board crates (`board-esp`, `board-pc`)
Compiled as `rlib`. Each exports a single `pub fn start(spawner: &Spawner)` that spawns the `core-interface` tasks. Board crates have no knowledge of vehicle logic or protos — they only drive hardware peripherals (radio, CAN, LTE) by reading/writing the edge channels (`BLE_RX_CHANNEL`, `BLE_TX_CHANNEL`, `MQTT_RX_CHANNEL`, `MQTT_TX_CHANNEL`).

### Vehicle crates (`cars/*`)
Compiled as `rlib`. Each exports 3 `#[embassy_executor::task]` functions: `handle_basic_commands_task`, `handle_advanced_commands_task`, and `state_update_task`. They read from `BASIC_CMD_CHANNEL` / `ADVANCED_CMD_CHANNEL`, write results to `CMD_RESP_CHANNEL`, and push state updates to `VEHICLE_STATE_CHANNEL`. They depend on `core-interface` for the channel statics and shared types, but never touch board code.

Vehicle crates that use CAN also export a `can_rx_task` (reads `CAN_RX_CHANNEL`) and a `pub static CAN_FILTERS: &[CanFilter]` that xtask passes to the board's CAN init functions.

### `cars/virtual-car` (`virtual-car-controller`)
A mock vehicle implementation used strictly for UI/UX testing without physical hardware. Contains real Embassy tasks and real proto encoding, just no CAN bus.

*   `VirtualCarState` is `pub` with `pub` fields and initialises with concrete defaults (`odometer: 0`, `is_driving: false`, `are_doors_locked: false`, `speed: 0`, `gear: 0`).
*   `pub fn tick_simulation(state: &mut VirtualCarState, tick: u64)` advances a 20-tick (≈ 100 s) drive cycle: ticks 0–4 ramp speed 0 → 80 kph, 5–9 cruise at 80 kph, 10–14 ramp 80 → 0 kph, 15–19 park at 0 kph. `gear` and `is_driving` are derived from `speed`; `odometer` increments by 1 km at tick 19; `are_doors_locked` is intentionally untouched (command-controlled only).
*   `state_update_task` calls `tick_simulation` on each 5-second fire and increments an internal counter, so the app receives realistic-looking state updates over MQTT without any CAN frames.

### `.app_build` (ephemeral, xtask-generated)
The actual binary entry point. Generated fresh by `xtask` before every build from a `main.template.rs` file living in the board's folder. It is the only crate that imports both the board crate and the vehicle crate, wiring them together by spawning all tasks. It is excluded from the Cargo workspace.

## Dynamic builds (xtask)

`xtask` reads `config.toml`, resolves the board and vehicle crate paths, reads the vehicle's `contracts/.../meta.toml` to inject the `PLATFORM_ID` constant, and writes `.app_build/Cargo.toml` + `.app_build/src/main.rs` before invoking `cargo build`.

*   The `board` and `platform` values in `config.toml` can be overridden at the CLI with `--board` and `--platform`, e.g. `cargo xtask build --board pc`.
*   `--debug` produces an unoptimised build (no `--release`; default is release). The debug binary is what the VS Code probe-rs debugger launch config points at.
*   **`config.toml` is gitignored.** It holds per-developer settings (WiFi credentials, CAN pins, etc.) and must never be committed. The tracked template is `config.toml.example`. On first invocation, if `config.toml` is absent and `config.toml.example` exists, xtask copies the example into place so a bare clone compiles without manual setup. This auto-copy only triggers for the default `"config.toml"` path — passing an explicit positional config path bypasses it.
*   For ESP builds, the generated `.app_build/Cargo.toml` carries `esp-radio = { version = "0.18.0", features = ["wifi", "ble", "unstable"] }` and `esp-storage = { version = "0.9.0", features = ["critical-section"] }` as **direct** dependencies so xtask can forward chip features as `esp-radio/{mcu}` and `esp-storage/{mcu}`; it also pins `esp-alloc = "0.10.0"`.
*   All ESP-specific crate versions are sourced by xtask from `boards/esp/Cargo.toml`: direct deps use `[dependencies]`; versions needed only in the generated `.app_build` crate (e.g. `esp-backtrace`, `esp-bootloader-esp-idf`, `embedded-test`) live in `[package.metadata.xtask-only-deps]` so they are co-located with the crates they apply to.

### Template files
*   **`main.template.rs`:** Each board folder contains one with placeholder tokens (`{PLATFORM_ID}`, `{VEHICLE_CRATE_IDENT}`, `{CAN_TX_PIN}`, `{MTLS_CERTS}`, etc.). xtask substitutes the tokens to produce `.app_build/src/main.rs`. Edit the template to change the entry-point structure; edit the builder (`xtask_builder.rs`) to change what gets substituted.
*   **`tests.template.rs` (ESP only):** `boards/esp/tests.template.rs` is the counterpart for on-hardware testing. xtask generates `.app_test_build/src/main.rs` from it, injects `{PLATFORM_ID}` and the contents of `boards/esp/tests/hardware.rs` (`{ON_HARDWARE_TESTS}`), then compiles and flashes via `probe-rs`. The template uses `embedded-test` with the Embassy executor. See [Testing](testing.md).

## Channels

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
| `CAN_TX_CHANNELS[bus_id]` | vehicle → board | `CanFrame` | Frame to transmit on that CAN bus (one queue per bus) |
| `CAN_DEBUG_RX_CHANNEL` | board → core | `CanRawCapture` | Raw (pre-filter) frames for debug streaming; consumed by `publish_can_debug_task` → BLE only |

**`InboundCommand`** carries `{ message_id: u64, transport: Transport, bytes: Vec<u8> }`. The vehicle echoes the `Transport` tag back in `CMD_RESP_CHANNEL` so `route_responses_task` can route the response to the correct radio without the vehicle knowing anything about transports.

**`platform_id`** is a CRC32 of the proto package name (e.g. `0xF7544D7E` for `opencar.cars.virtual_car.v1`), pre-computed and stored in `contracts/.../meta.toml`. xtask injects it as a compile-time constant into `.app_build/src/main.rs`. `core_interface::init(platform_id)` stores it in an `AtomicU32`; the dispatcher tasks validate every inbound message against it and silently drop mismatches.

**MQTT restrictions:** `process_mqtt_commands_task` only forwards `BasicCommandBytes`. `SystemCommand` and `AdvancedCommandBytes` are restricted to BLE and silently dropped from MQTT.

### Channel flow

```
BLE driver  ──BLE_RX──▶  core-interface  ──BASIC_CMD────▶  vehicle
            ◀─BLE_TX──                  ◀─CMD_RESP─────
                         ──ADVANCED_CMD──▶
                         ──SYSTEM_CMD───▶  board

MQTT driver ──MQTT_RX─▶  core-interface  ──BASIC_CMD────▶  vehicle
            ◀─MQTT_TX─                  ◀─CMD_RESP─────

CAN driver  ──CAN_RX──▶  vehicle
            ◀─CAN_TX──
```

### MQTT driver resilience
Both board MQTT driver tasks (`board-pc` and `board-esp`) apply two rules to stay live under all broker conditions:

1.  **Inbound `try_send`:** When a PUBLISH arrives, the payload is forwarded to `MQTT_RX_CHANNEL` via `try_send` (not `.send().await`). Blocking here would deadlock if downstream tasks (`process_mqtt_commands_task` → `BASIC_CMD_CHANNEL` → vehicle → `CMD_RESP_CHANNEL` → `route_responses_task` → `MQTT_TX_CHANNEL`) are all simultaneously full, preventing the driver from draining its own outbound channel. A full `MQTT_RX_CHANNEL` logs a warning and drops the inbound command; the app can retry.
2.  **`abort()` skipped for broker `DISCONNECT`:** When the broker sends a `DISCONNECT` packet (e.g. `SessionTakenOver` from a duplicate `client_id`), `poll()` returns `Err(MqttError::Disconnect { .. })` while the TCP stream is still healthy. Calling `abort()` in this case panics inside rust-mqtt. The driver only calls `abort()` for IO/protocol errors, never for `MqttError::Disconnect`. Either way the session ends and the reconnect loop retries after 5 s.

## Board / core-interface contract

Embassy is used freely inside `core-interface`. Boards adapt to it — not the other way around.

**Task pattern:** `#[embassy_executor::task]` cannot take generic (`impl Trait`) parameters. The split is:
*   `core-interface` defines all `static Channel`s and `#[embassy_executor::task]` functions containing all protocol logic.
*   Board crates define thin **driver tasks** with concrete hardware types that read/write only the four edge channels (`BLE_RX/TX`, `MQTT_RX/TX`). These are the only hardware-specific tasks.
*   Vehicle crates define their own Embassy tasks that read/write only the four inner channels (`BASIC_CMD`, `ADVANCED_CMD`, `CMD_RESP`, `VEHICLE_STATE`).
*   `.app_build` spawns all tasks from all three layers in `main`.

**Per-subsystem approach:**
*   **CAN:** Vehicle crates define their filter list (`&'static [CanFilter]`). Board crates own all hardware; `CAN_RX_CHANNEL` and the per-bus `CAN_TX_CHANNELS` are the shared contract. Each board provides driver functions (`init_*` / `run_*_loop`) and async loop tasks that software-filter received frames against the vehicle's filter list. See [CAN bus](can.md).
*   **MQTT / networking:** `core-interface` logic uses `embassy-net::TcpSocket`. Each board provides an `embassy-net::driver::Driver` implementation (`esp-radio` WiFi on ESP, tuntap shim on PC).
*   **BLE:** No single cross-platform embassy BLE abstraction exists. `core-interface` defines its own traits (e.g. `trait BleGatt`). ESP implements them with `esp-radio` + `trouble-host`; PC implements them as mocks. See [BLE](ble.md).

## Board-specific requirements

### ESP (`board-esp`)
*   Compiled as `rlib`. `esp-rtos` (Embassy executor + time driver), `esp-alloc` (global heap), `esp-backtrace`, and `RUSTFLAGS="-C link-arg=-Tlinkall.x"` all live in `.app_build`, not in `board-esp` itself, because they must be in the final binary crate.
*   xtask uses `+esp -Zbuild-std=core,alloc` for xtensa targets.
*   Embassy crate versions must match what `esp-rtos` uses (currently `embassy-executor = "0.10.0"`). In 0.10 the spawn API changed: task constructors return `Result<SpawnToken, SpawnError>` and `spawner.spawn()` returns `()` — pattern is `spawner.spawn(task(args).unwrap())`.
*   CAN driver loops run on **core 1** via `esp_rtos::start_second_core` (4-arg form: `cpu_ctrl`, `SoftwareInterrupt::<1>`, `&mut stack`, `closure`); comms and vehicle tasks run on core 0.
*   All ESP HAL dependencies are behind an optional `hardware` Cargo feature; firmware builds (via xtask) enable it automatically. Without the feature, the crate compiles on the host for unit testing (only `embedded-can` and `core-interface` are unconditional). Pure logic that is host-testable (e.g. `compute_mcp_masks`) is kept outside `#[cfg(feature = "hardware")]` blocks.
*   WiFi/MQTT is wired against **`esp-radio 0.18.0`**. There is no shared radio controller — BLE and WiFi initialise independently.
*   `board-esp` enables esp-radio's `coex` feature: every ESP has a single 2.4 GHz RF front end, and without `coex` WiFi and BLE are not time-sliced and interfere with each other.
*   BLE: `ble::controller::BleConnector::new(peripherals.BT, ble::Config::default())` (2 args, no `init_radio()` call).
*   WiFi: `esp_radio::wifi::new(peripherals.WIFI, WifiControllerConfig::default())` returns `(controller, interfaces)`; use `interfaces.station` (not `.sta`); configure with `WifiModeConfig::Station(StationConfig::default().with_ssid(...).with_password(...))`.
*   **WiFi can be disabled** via `[network.wifi] enabled = false` in `config.toml`; when disabled, xtask omits the WiFi/MQTT constants and `mqtt_driver_task` spawn from the generated `main.rs`, but `ble_transport_task` is always spawned.
*   With `embassy-net 0.9`, the stack handle is `Stack<'static>` and TCP sockets are created with `TcpSocket::new(*stack, ...)`.
*   In the final binary crate, `esp-rtos` must enable `features = ["embassy", "esp-radio", "esp-alloc"]`; omitting `esp-radio` leads to unresolved `esp_rtos_*` linker errors.
*   **Heap/stack tuning warning:** `esp_alloc::heap_allocator!(size = ...)` and the linker-defined CPU0 stack share the same SRAM budget. If heap is too small, esp-radio/RTOS task creation can panic with `Failed to allocate stack`; if heap is too large, CPU0 can hit stack-guard panics. Keep large BLE/network buffers (for example `HostResources`) in `StaticCell` instead of task locals, and for ESP32-S3 with the current task mix prefer around `160 * 1024` heap unless profiling indicates otherwise.
*   On the RISC-V chips (C3/C6) xtask also adds a 64 KB heap region in RAM reclaimed from the 2nd-stage bootloader (`#[esp_hal::ram(reclaimed)]`, a separate region from stack/heap), on top of the `83 * 1024` main heap.

### PC (`board-pc`)
*   Compiled as `rlib`. `embassy-executor` with `platform-std` + `executor-thread` features, `embassy-time` with `std` feature, and `critical-section` with `std` feature all live in `.app_build`. No `tokio` — embassy's std driver covers timing and critical sections.
*   CAN is implemented via SocketCAN (`socketcan` crate) using an `#[embassy_executor::task(pool_size = 4)]` per bus. Frame conversion between `socketcan` and `core-interface` types is extracted into `pub(crate) fn socketcan_to_core_frame` / `core_to_socketcan_frame` helpers for unit testing.
*   `board-pc` is always `std`.

## `no_std` / test gating pattern
*   `core-interface` and all vehicle crates use `#![cfg_attr(not(test), no_std)]`.
*   `board-esp` uses `#![cfg_attr(not(test), no_std)]` — always `no_std` except during host tests. HAL deps are optional behind the `hardware` feature (enabled by xtask for firmware builds). Only allocation-free, HAL-free code is available without the feature.
*   `board-pc` is always `std`.
