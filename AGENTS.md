# Context: Open Car Firmware (Rust)

## 📌 Project Role
This is the embedded Rust monorepo for the hardware controller (ESP32 / Raspberry Pi). It handles CAN bus reading/writing, BLE/WiFi/LTE connections, and OTA updates.

## 🏗️ Architecture: Cargo Workspace & xtask
*   **`core-interface`:** The intermediary between transport layers (BLE/MQTT) and vehicle logic. Knows nothing about specific cars or boards. Uses Embassy throughout (`embassy_time`, `embassy_sync`, `embassy_executor`). Defines 9 static `Channel`s as the inter-crate contract, shared types (`Transport`, `InboundCommand`, `VehicleStatePayload`), and 4 pure-logic `#[embassy_executor::task]` functions (`process_ble_commands_task`, `process_mqtt_commands_task`, `route_responses_task`, `publish_state_task`).
*   **Board crates (`board-esp`, `board-pc`):** Compiled as `rlib`. Each exports a single `pub fn start(spawner: &Spawner)` that spawns the 4 `core-interface` tasks. Board crates have no knowledge of vehicle logic or protos — they only drive hardware peripherals (radio, CAN, LTE) by reading/writing the edge channels (`BLE_RX_CHANNEL`, `BLE_TX_CHANNEL`, `MQTT_RX_CHANNEL`, `MQTT_TX_CHANNEL`).
*   **Vehicle Crates (e.g., `virtual-car-controller`):** Compiled as `rlib`. Each exports 3 `#[embassy_executor::task]` functions: `handle_basic_commands_task`, `handle_advanced_commands_task`, and `state_update_task`. They read from `BASIC_CMD_CHANNEL` / `ADVANCED_CMD_CHANNEL`, write results to `CMD_RESP_CHANNEL`, and push state updates to `VEHICLE_STATE_CHANNEL`. They depend on `core-interface` for the channel statics and shared types, but never touch board code.
*   **`cars/virtual-car` (`virtual-car-controller`):** A mock vehicle implementation used strictly for UI/UX testing without physical hardware. Contains real Embassy tasks and real proto encoding, just no CAN bus.
*   **`.app_build` (ephemeral, xtask-generated):** The actual binary entry point. Generated fresh by `xtask` before every build from a `main.template.rs` file living in the board's folder. It is the only crate that imports both the board crate and the vehicle crate, wiring them together by spawning all tasks. It is excluded from the Cargo workspace.
*   **Dynamic Builds:** `xtask` reads `config.toml`, resolves the board and vehicle crate paths, reads the vehicle's `contracts/.../meta.toml` to inject the `PLATFORM_ID` constant, and writes `.app_build/Cargo.toml` + `.app_build/src/main.rs` before invoking `cargo build`. The `board`, `brand`, and `platform` values in `config.toml` can be overridden at the CLI with `--board`, `--brand`, and `--platform` flags, e.g. `cargo xtask build --board pc`.

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
*   **CAN:** `core-interface` logic uses `embedded-can` / `embedded-hal-async` traits. Each board provides a concrete driver task feeding a CAN channel.
*   **MQTT / networking:** `core-interface` logic uses `embassy-net::TcpSocket`. Each board provides an `embassy-net::driver::Driver` implementation (`esp-wifi` on ESP, tuntap shim on PC).
*   **BLE:** No single cross-platform embassy BLE abstraction exists. `core-interface` defines its own traits (e.g. `trait BleGatt`). ESP implements them with `esp-wifi`'s BLE stack; PC implements them as mocks.

**Board-specific requirements:**
*   **ESP (`board-esp`):** Compiled as `rlib`. `esp-rtos` (Embassy executor + time driver), `esp-alloc` (global heap), `esp-backtrace`, and `RUSTFLAGS="-C link-arg=-Tlinkall.x"` all live in `.app_build`, not in `board-esp` itself, because they must be in the final binary crate. The xtask uses `+esp -Zbuild-std=core,alloc` for xtensa targets. Embassy crate versions must match what `esp-rtos` uses (currently `embassy-executor = "0.9.1"`).
*   **PC (`board-pc`):** Compiled as `rlib`. `embassy-executor` with `arch-std` + `executor-thread` features, `embassy-time` with `std` feature, and `critical-section` with `std` feature all live in `.app_build`. No `tokio` — embassy's std driver covers timing and critical sections.

**`main.template.rs`:** Each board folder contains a `main.template.rs` with placeholder tokens (`{PLATFORM_ID}`, `{VEHICLE_CRATE_IDENT}`, `{CAN_TX_PIN}`, `{MTLS_CERTS}`, etc.). xtask reads this file and substitutes the tokens to produce `.app_build/src/main.rs`. Edit the template to change the entry-point structure; edit the builder to change what gets substituted.

## 🏷️ Versioning & CI/CD
*   **Independent Crates:** Managed via `release-plz` (not implemented yet, waiting for at least one car implementation to work). Changes to the core cascade safely to vehicle crates.
*   **Tagging:** Uses prefixed tags (e.g., `car-hmg-v0.2.1`).
*   **Agnostic Pipelines:** GitHub Actions trigger on `*-v*.*.*`. The YAML dynamically parses the tag to extract the car name, passes it to `xtask`, builds the `.bin`, and uploads it to GitHub Releases.

## 📡 OTA Manifest Generation
The CI/CD pipeline generates static `manifest.json` files hosted on GitHub Pages. This includes a Compatibility Matrix (`min_app_version`) to ensure the hardware is never updated to a Protobuf version the phone app cannot decode.
