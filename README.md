# 🚗 Open Car Firmware

This repository contains the **embedded Rust-based firmware** for an open-source car hardware controller. It is designed to run on hardware like the ESP32 or Raspberry Pi, handling CAN bus communication, wireless connectivity (BLE/WiFi/LTE), and Over-the-Air (OTA) updates.

## 🚀 Getting Started

The project is built using a custom `xtask` command that dynamically generates a build crate in the `.app_build` directory based on a `config.toml` file.

To build or run the firmware, use the following commands from the root of the repository:

```bash
# Build the firmware
cargo xtask build [path/to/your/config.toml]

# Run the firmware (if supported by the target)
cargo xtask run [path/to/your/config.toml]
```

If no config file is provided, `config.toml` in the root directory will be used by default.

## 🏗️ Architecture

The firmware is structured as a Cargo workspace with a few key components:

*   **`core-interface`**: A generic hardware and networking layer. It is responsible for wrapping and broadcasting vehicle-agnostic data payloads.
*   **Vehicle Crates**: Located in the `cars/` directory, these crates contain the specific CAN bus mappings and logic for a particular vehicle brand and platform.
*   **Board Crates**: Located in the `boards/` directory, these crates provide the hardware-specific implementations for different microcontroller or single-board computer targets.
*   **`xtask`**: A custom build system that reads a `config.toml` to select the target board and vehicle, then assembles the final application crate for compilation.

## 📂 Project Structure

```
.
├── boards/         # Board-specific crates (e.g., ESP32, Raspberry Pi)
├── cars/           # Vehicle-specific crates
├── core-interface/ # The generic hardware and networking layer
├── xtask/          # Custom build and automation scripts
├── config.toml     # Example configuration file
└── ...
```

## � Running Tests

All host tests run without any hardware. `--test-threads=1` is required because the channel statics are process-global — parallel threads corrupt each other’s state.

### All host tests at once (via xtask)

```bash
cargo xtask test
```

This runs `core-interface`, `board-pc`, `board-esp`, and the configured vehicle crate in order.

### Individual crates

```bash
cargo test -p core-interface -- --test-threads=1          # channel dispatch, filter, routing
cargo test -p virtual-car-controller -- --test-threads=1  # CAN frame handling, command processing
cargo test -p board-pc -- --test-threads=1                # SocketCAN frame conversion helpers
cargo test -p board-esp -- --test-threads=1               # MCP2515 mask computation (no HAL needed)
```

> **Why not `--workspace`?** That would also compile `board-esp` with its ESP HAL deps, which require the Xtensa toolchain. Use explicit `-p` flags or `cargo xtask test` instead.

### On-hardware tests (ESP32, requires `probe-rs`)

For tests that need real hardware — CAN loopback, channel round-trips, etc.:

```bash
cargo xtask test --board esp --on-hardware
```

This generates `.app_test_build/` (mirroring `.app_build/`), compiles the [embedded-test](https://github.com/embassy-rs/embedded-test) harness with the ESP toolchain, and flashes/runs it via `probe-rs`. Test results are reported over RTT.

Add your own on-device tests to `boards/esp/tests/hardware.rs` — xtask injects that file into the generated test binary automatically.

> **Note:** `probe-rs` must be installed: `cargo install probe-rs-tools`

## �🤖 CI/CD and Versioning

*   **Versioning**: Crate versioning is managed with `release-plz`. *This is not implemented yet, it will be implemented later once we have a working car implementation.*
*   **Continuous Integration**: GitHub Actions are used to build the firmware for different vehicle targets. The workflow dynamically parses the git tag to determine the correct configuration and build parameters.
*   **Releases**: On new version tags, the CI pipeline builds the firmware binary and attaches it to a GitHub Release.

## 📡 OTA Updates

The CI/CD pipeline also generates a static `manifest.json` file for each release, which is hosted on GitHub Pages. This manifest includes a compatibility matrix to ensure that devices in the field only download and apply compatible firmware updates, preventing issues with breaking protocol changes.

## 📜 License

This project is licensed under the terms of the LICENSE file.
