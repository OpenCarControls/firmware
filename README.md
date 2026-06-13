# 🚗 Open Car Firmware

Embedded Rust firmware for an open-source car hardware controller. Runs on ESP32 family chips or Linux-based boards (PC/Raspberry Pi), handling CAN bus communication, wireless connectivity (BLE/WiFi/LTE), and Over-the-Air (OTA) updates.

## 📋 Prerequisites

- **Rust** (stable toolchain) — `rustup` recommended
- **ESP target** (ESP32 only): the `esp` toolchain from [esp-rs](https://github.com/esp-rs/rust-build), installed via `espup`
- **On-hardware tests / flashing** (ESP32 only): `probe-rs` and `espflash` — both pre-installed in the dev container

> **Note for Windows Users:** If you prefer to work locally without dev containers, please see the [Native Windows Setup Guide](docs/windows-setup.md).

## 🔌 Flashing & Debugging Hardware

The ESP dev containers (`esp-xtensa`, `esp-riscv`) run with `--privileged`, giving them direct access to any USB device that appears on the host. The setup depends on where Docker is running.

### Dev container running locally

Plug in the ESP32. The device appears immediately inside the container as `/dev/ttyACM0` (native USB) or `/dev/ttyUSB0` (UART-chip boards). No extra setup needed.

### Dev container in a remote VM (VS Code Remote SSH)

The USB device is on your local PC. You need to forward it to the VM using [usbipd](https://github.com/dorssel/usbipd-win) on your Windows PC, then pull it from inside the container. `usbip` is pre-installed in the dev container image.

**One-time Windows PC setup:**

```powershell
winget install usbipd
```

Then allow inbound TCP port 3240 through Windows Firewall (usbipd does not do this automatically for non-WSL clients):

```powershell
New-NetFirewallRule -DisplayName "usbipd" -Direction Inbound -Protocol TCP -LocalPort 3240 -Action Allow
```

**One-time VM setup** (run on the VM over SSH):

```bash
sudo modprobe vhci-hcd
echo "vhci-hcd" | sudo tee -a /etc/modules   # persist across VM reboots
```

**Per session — two steps:**

1. On your **Windows PC** (in any terminal), make the device available for sharing (one-time per device, survives reboots):

```powershell
# Find your ESP32's BUSID
# Native USB boards (e.g. LILYGO T-SIM7670G S3) show as "Espressif"
# UART-chip boards (CP2102, CH340) show as "Silicon Labs" or "QinHeng Electronics"
usbipd list

usbipd bind --busid <BUSID>   # first time per device only
```

2. Inside the **VS Code terminal** (runs in the container), pull the device:

```bash
sudo usbip attach -r <windows-pc-ip> -b <busid>
```

The device appears immediately as `/dev/ttyACM0` (native USB) or `/dev/ttyUSB0` (UART-chip boards). No container restart or VS Code reconnect needed. If you physically unplug and replug the ESP32, re-run `usbip attach` from the container terminal.

> **Native USB boards (e.g. LILYGO T-SIM7670G S3):** Normal flash cycles are stable — `espflash` uses a USB control request to reset into the bootloader without re-enumerating. However, pressing the physical **RST** button causes a full USB re-enumeration which drops the usbip session. Re-run `usbip attach` after a manual RST press.

### Debugging

The `esp-xtensa` dev container includes the `probe-rs-debugger` VS Code extension. Build with `--debug`, then select **"Debug: ESP32-S3 (probe-rs)"** in the Run & Debug panel and press F5. probe-rs flashes the binary and halts the CPU at reset, giving you breakpoints, variable inspection, and step execution.

### Serial logs

The ESP32-S3's USB_SERIAL_JTAG peripheral exposes a second CDC-ACM interface alongside the JTAG debug channel. Logs written via `esp-println` / the `log` crate appear on `/dev/ttyACM0` independently of the debugger. The `esp-xtensa` container includes the **Serial Monitor** extension (View → Serial Monitor), pre-configured for `/dev/ttyACM0` at 115200 baud. You can also read logs directly:

```bash
stty -F /dev/ttyACM0 raw 115200 && cat /dev/ttyACM0
```

### Flashing

```bash
cargo xtask build --board esp
espflash flash --port /dev/ttyACM0 .app_build/target/xtensa-esp32s3-none-elf/release/app-build
```

### On-hardware tests

```bash
cargo xtask test --board esp --on-hardware
```

## 🚀 Getting Started

The project uses a custom `xtask` command that reads `config.toml`, generates an ephemeral `.app_build/` crate, and invokes `cargo build` against it.

```bash
# Build with the default config.toml
cargo xtask build

# Build with a custom config file
cargo xtask build path/to/your/config.toml

# Build for a specific board (overrides config.toml)
cargo xtask build --board pc
cargo xtask build --board esp

# Build unoptimised (for use with the VS Code probe-rs debugger)
cargo xtask build --board esp --debug

# Run (PC board only)
cargo xtask run

# Run clippy on the generated app crate
cargo xtask clippy
```

## ⚙️ Configuration (`config.toml`)

`config.toml` at the repo root is the single build configuration file. Key sections:

```toml
[target]
board    = "esp"          # "esp" or "pc"
brand    = "virtual-car"  # subfolder under cars/
platform = "virtual-car"  # subfolder under cars/<brand>/platforms/

[hardware.esp]
mcu          = "esp32s3"  # esp32 | esp32s3 | esp32c3 | esp32c6
modem_tx_pin = 17
modem_rx_pin = 16

# CAN buses — order determines bus_id (0-based)
[[hardware.esp.can_buses]]
interface = "twai"   # built-in CAN peripheral
tx_pin    = 5
rx_pin    = 4

# [[hardware.esp.can_buses]]
# interface  = "mcp2515"   # external SPI CAN controller
# cs_pin     = 10
# clk_pin    = 11
# mosi_pin   = 12
# miso_pin   = 13
# int_pin    = 14
# can_speed  = "500kbps"   # 100/125/250/500/1000kbps
# mcp_speed  = "16mhz"     # 8mhz | 16mhz

[hardware.pc]
[[hardware.pc.can_buses]]
interface = "vcan0"   # SocketCAN interface name

[network.mqtt]
broker_url = "mqtts://192.168.0.100:8883"
client_id  = "dev-car"
auth_mode  = "basic"   # "basic" or "mtls"
username   = "user"
password   = "password"
```

The `board`, `brand`, and `platform` values can be overridden at the CLI with `--board`, `--brand`, and `--platform` flags without editing the file.

## 🏗️ Architecture

The firmware is a Cargo workspace of independently compiled crates that cooperate solely through 11 static Embassy `Channel`s defined in `core-interface`. No traits link the layers — only shared-memory queues.

```
┌───────────────────────────────────────────────────────────┐
│                      .app_build/                          │
│  (ephemeral binary crate generated by xtask per build)    │
│  Spawns all tasks; only crate that imports board + vehicle │
└───────┬──────────────────┬────────────────────────────────┘
        │                  │
┌───────▼──────┐   ┌───────▼──────────────┐
│  board-esp   │   │  virtual-car-ctrl    │  ← vehicle crate
│  board-pc    │   │  (or any other car)  │
│              │   │                      │
│  BLE/WiFi/   │   │  CAN decoding,       │
│  LTE, CAN HW │   │  command handling,   │
│  drivers     │   │  state publishing    │
└───────┬──────┘   └───────┬──────────────┘
        │                  │
┌───────▼──────────────────▼──────────────┐
│              core-interface             │
│  11 static Channels · 4 Embassy tasks  │
│  platform_id gating · BLE/MQTT routing │
└─────────────────────────────────────────┘
```

**Key crates:**

| Crate | Role |
|---|---|
| `core-interface` | Channel definitions, shared types, 4 Embassy dispatcher tasks, `passes_filter()` |
| `board-esp` | ESP32 hardware drivers: TWAI, MCP2515 (SPI CAN); HAL deps behind optional `hardware` feature |
| `board-pc` | Linux/PC drivers: SocketCAN via `socket_can_task`; frame conversion helpers |
| `cars/<brand>/platforms/<platform>` | Vehicle logic: CAN decoding, command handling, state encoding; exports Embassy tasks and `CAN_FILTERS` |
| `xtask` | Build orchestrator: reads `config.toml`, generates `.app_build/`, invokes cargo commands |

**Channel flow (board ↔ core ↔ vehicle):**

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

**ESP two-core split:** CAN driver loops run on core 1 via `esp_rtos::start_second_core`; all comms and vehicle tasks run on core 0.

> See [AGENTS.md](AGENTS.md) for the full architectural reference including all channel types, template tokens, and the complete CAN bus architecture.

## 📂 Project Structure

```
.
├── boards/
│   ├── esp/            # ESP32 board crate + main.template.rs + xtask_builder.rs
│   └── pc/             # PC/Linux board crate + main.template.rs + xtask_builder.rs
├── cars/
│   └── virtual-car/
│       └── platforms/
│           └── virtual-car/   # virtual-car-controller (mock for UI/UX testing)
├── contracts/          # Protobuf definitions + meta.toml per vehicle platform
├── core-interface/     # Shared channels, types, and dispatcher tasks
├── xtask/              # Build system (cargo xtask ...)
├── config.toml         # Default build configuration
└── ...
```

## 🧪 Running Tests

All host tests run without hardware. `--test-threads=1` is required because the channel statics are process-global — parallel threads corrupt each other's state.

### All host tests (via xtask)

```bash
cargo xtask test
```

Runs all four crates in sequence.

### Individual crates

```bash
cargo test -p core-interface -- --test-threads=1          # 53 tests: dispatch, filter, routing, CAN debug
cargo test -p virtual-car-controller -- --test-threads=1  # 25 tests: CAN frames, commands, simulation
cargo test -p board-pc -- --test-threads=1                #  8 tests: SocketCAN frame conversion
cargo test -p board-esp -- --test-threads=1               # 14 tests: MCP2515 masks, BLE helpers
```

> **Why not `--workspace`?** That would also compile `board-esp` with its ESP HAL deps, which require the Xtensa toolchain. Use explicit `-p` flags or `cargo xtask test` instead.

### On-hardware tests (ESP32, requires `probe-rs`)

```bash
cargo xtask test --board esp --on-hardware
```

Generates `.app_test_build/`, compiles an [embedded-test](https://github.com/probe-rs/embedded-test) harness with the Xtensa toolchain, and flashes/runs it via `probe-rs run`. Results are reported via semihosting.

Add custom on-device tests to `boards/esp/tests/hardware.rs` — xtask injects that file automatically.

## 📦 Build A Ready-To-Flash ESP BIN

When building inside a dev container, generate the firmware image in-container and flash it later from your host machine.

1. Build the ESP firmware ELF:

```bash
cargo xtask build --board esp
```

2. Export a flashable BIN with `espflash`:

```bash
# ESP32-S3 example (single merged image)
espflash save-image \
        --chip esp32s3 \
        --merge \
        .app_build/target/xtensa-esp32s3-none-elf/release/app-build \
        ./dist/open-car-esp32s3-merged.bin
```

`--ignore-app-descriptor` and `--min-chip-rev` are no longer needed: the firmware now calls `esp_bootloader_esp_idf::esp_app_desc!()`, which emits a valid `esp_app_desc_t` with `min_efuse_blk_rev_full = 0`. Without this, the ESP-IDF bootloader reads garbage data from that field and rejects the image with `Image requires efuse blk rev >= vXXX.XX`.

3. Copy the generated `./dist/*.bin` to your local machine and flash there.

Target triples by MCU:

- `esp32` → `xtensa-esp32-none-elf`
- `esp32s3` → `xtensa-esp32s3-none-elf`
- `esp32c3` → `riscv32imc-unknown-none-elf`
- `esp32c6` → `riscv32imac-unknown-none-elf`

## 🤖 CI/CD and Versioning

- **Continuous Integration:** GitHub Actions runs `fmt`, `clippy`, host tests (diff-based, only affected crates), and full builds for both `esp` and `pc` targets.
- **Versioning:** Managed with `release-plz` (not yet active — pending a complete vehicle implementation). Tags use the format `car-<brand>-v<semver>`.
- **Releases:** On new version tags, CI builds the firmware binary and attaches it to a GitHub Release.

## 📡 OTA Updates

CI generates a static `manifest.json` per release, hosted on GitHub Pages. It includes a compatibility matrix (`min_app_version`) so devices in the field never apply a firmware update whose Protobuf schema the phone app cannot decode.

## 📜 License

This project is licensed under the terms of the [LICENSE](LICENSE) file.
