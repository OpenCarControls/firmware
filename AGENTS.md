# Context: Open Car Firmware (Rust)

## 📌 Project Role
This is the embedded Rust monorepo for the hardware controller (ESP32 / Raspberry Pi). It handles CAN bus reading/writing, BLE/WiFi/LTE connections, and OTA updates.

## 🏗️ Architecture: Cargo Workspace & xtask
*   **`core-interface`:** The generic hardware and networking layer. Knows nothing about specific cars or boards. Uses Embassy throughout (`embassy_time`, `embassy_sync`, `embassy_executor`). Defines static channels and pure-logic tasks. Wraps specific byte payloads into the `core.proto` MessageEnvelope and broadcasts them.
*   **Vehicle Crates (e.g., `car-hmg`):** Contain CAN mappings and logic. They depend on `core-interface`.
*   **`car-virtual`:** A mock implementation used strictly for UI/UX testing without physical hardware.
*   **Dynamic Builds:** A custom `xtask` reads a `config.toml` file to dynamically select the board target and inject the correct vehicle crate into a final, ephemeral `.app_build` crate. The `board`, `brand`, and `platform` values in `config.toml` can be overridden at the CLI with `--board`, `--brand`, and `--platform` flags, e.g. `cargo xtask build --board pc`.

## 🔌 Board / Core-Interface Contract
Embassy is used freely inside `core-interface`. Boards adapt to it — not the other way around.

**Task pattern:** `#[embassy_executor::task]` cannot take generic (`impl Trait`) parameters. The split is:
*   `core-interface` defines `static Channel`s and `#[embassy_executor::task]` functions that contain all logic and communicate via those channels. Tasks that need no hardware at all (e.g. a blink timer) live here directly.
*   Board crates define thin **driver tasks** with concrete hardware types that read/write the channels. These are the only board-specific tasks.

**Per-subsystem approach:**
*   **CAN:** `core-interface` logic uses `embedded-can` / `embedded-hal-async` traits. Each board provides a concrete driver task feeding a CAN channel.
*   **MQTT / networking:** `core-interface` logic uses `embassy-net::TcpSocket`. Each board provides an `embassy-net::driver::Driver` implementation (`esp-wifi` on ESP, tuntap shim on PC).
*   **BLE:** No single cross-platform embassy BLE abstraction exists. `core-interface` defines its own traits (e.g. `trait BleGatt`). ESP implements them with `esp-wifi`'s BLE stack; PC implements them as mocks.

**Board-specific requirements:**
*   **ESP (`board-esp`):** Uses `esp-rtos` (Embassy executor + time driver), `esp-alloc` (global heap, initialised at runtime via `esp_alloc::heap_allocator!`), and `RUSTFLAGS="-C link-arg=-Tlinkall.x"` passed by xtask to pick up `hal-defaults.x` → `device.x` interrupt symbol stubs. Embassy crate versions must match what `esp-rtos` uses (currently `embassy-executor = "0.9.1"`).
*   **PC (`board-pc`):** Uses `embassy-executor` with `arch-std` + `executor-thread` features, `embassy-time` with `std` feature, and `critical-section` with `std` feature. No `tokio` — embassy's std driver covers timing and critical sections.

## 🏷️ Versioning & CI/CD
*   **Independent Crates:** Managed via `release-plz` (not implemented yet, waiting for at least one car implementation to work). Changes to the core cascade safely to vehicle crates.
*   **Tagging:** Uses prefixed tags (e.g., `car-hmg-v0.2.1`).
*   **Agnostic Pipelines:** GitHub Actions trigger on `*-v*.*.*`. The YAML dynamically parses the tag to extract the car name, passes it to `xtask`, builds the `.bin`, and uploads it to GitHub Releases.

## 📡 OTA Manifest Generation
The CI/CD pipeline generates static `manifest.json` files hosted on GitHub Pages. This includes a Compatibility Matrix (`min_app_version`) to ensure the hardware is never updated to a Protobuf version the phone app cannot decode.
