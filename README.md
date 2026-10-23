# 🚗 Open Car Firmware

> [!WARNING]
> **This project is under active construction. Do not use it on a real vehicle.**
> It is experimental, largely untested on real cars, and may behave unpredictably.
> Interfering with a vehicle's CAN bus can cause malfunctions, disable safety
> systems, void warranties, or put people at risk. There is no warranty (see
> [LICENSE](LICENSE)); you use this entirely at your own risk.
>
> This is an independent project. It is not affiliated with, endorsed by, or
> sponsored by any vehicle manufacturer. Brand and model names are used only to
> describe compatibility.

Embedded Rust firmware for an open-source car hardware controller. Runs on ESP32 family chips or Linux-based boards (PC/Raspberry Pi), handling CAN bus communication, wireless connectivity (BLE/WiFi/LTE), and Over-the-Air (OTA) updates.

## 📋 Prerequisites

- **Rust** (stable toolchain) — `rustup` recommended
- **ESP target** (ESP32 only): the `esp` toolchain from [esp-rs](https://github.com/esp-rs/rust-build), installed via `espup`
- **On-hardware tests / flashing** (ESP32 only): `probe-rs` and `espflash` — both pre-installed in the dev container

The recommended environment is the dev container (`.devcontainer/`). It has the toolchains for every board, so the same container is used for PC and ESP work, including flashing. To work locally on Windows instead, see the [Native Windows Setup Guide](docs/windows-setup.md).

## 🚀 Getting Started

The project uses a custom `xtask` command that reads `config.toml`, generates an ephemeral `.app_build/` crate, and invokes `cargo build` against it.

```bash
cp config.toml.example config.toml   # optional: xtask copies it automatically on first run

cargo xtask build                          # build with the default config.toml
cargo xtask build path/to/config.toml      # build with a custom config file
cargo xtask build --board pc               # override the board (pc | esp)
cargo xtask build --board esp --debug      # unoptimised, for the VS Code probe-rs debugger
cargo xtask run                            # run (PC board only)
cargo xtask flash --port /dev/ttyACM0      # build and flash (ESP)
cargo xtask clippy                         # run clippy on the generated app crate
cargo xtask test                           # run all host tests
```

See [Flashing & debugging hardware](docs/hardware-setup.md) for USB setup, flashing, debugging and serial logs.

## ⚙️ Configuration

`config.toml` at the repo root is the single, per-developer build configuration. It is gitignored because it holds WiFi and MQTT credentials; start from the tracked [`config.toml.example`](config.toml.example), which documents every option.

```toml
[target]
hardware_profile = "meatpi-wican-esp32c3"  # optional preset from hardware-profiles/
platform         = "virtual-car"           # vehicle crate under cars/

[network.wifi]
ssid     = "YourSSID"
password = "YourPassword"

[network.mqtt]
broker_url = "mqtt://192.168.0.100:1883"
auth_mode  = "basic"                       # only "basic" works for now; "mtls" is not implemented yet
```

Per-board settings live under `[hardware.esp]` / `[hardware.pc]`, including CAN buses (`[[hardware.<board>.can_buses]]`, where order determines `bus_id`) and BLE options. The `board` and `platform` values can be overridden with `--board` and `--platform`. To reverse engineer a car, `[network.can_logger] enabled = true` streams the CAN buses to SavvyCAN over WiFi (see [CAN bus](docs/can.md#savvycan-logging)). Hardware profiles in `hardware-profiles/` are ready-made pin/CAN setups for specific devices; anything set in `config.toml`, including `board`, wins over the profile.

### Switching boards in the IDE

rust-analyzer needs to know which board you're working on: which features to enable (`board-esp`'s chip feature, e.g. `esp32c3`), which target to analyse for, and for Xtensa chips, which toolchain to use. `cargo xtask ide` derives that from the same config the build uses. It writes the rust-analyzer settings to `.vscode/settings.json` and a matching **Firmware: debug …** config (probe-rs with the right chip for ESP, lldb for PC) to `.vscode/launch.json`. Both files are gitignored and anything else in them is kept; rust-analyzer picks up the change automatically:

```bash
cargo xtask ide                # board/MCU from config.toml (e.g. the hardware profile's ESP32-C3)
cargo xtask ide --board pc     # temporarily work on the PC board
```

The dev container runs it once on creation. In VS Code, the **IDE: use board from config.toml** and **IDE: use PC board** tasks do the same. For ESP targets, std-only crates (`xtask`, `build-utils`, `board-pc`) and test targets are left out of rust-analyzer's `cargo check`, since they can't compile for a no_std target.

## 🏗️ Architecture

The firmware is a Cargo workspace of independently compiled crates that cooperate solely through static Embassy `Channel`s defined in `core-interface`. No traits link the layers — only shared-memory queues.

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
│  static Channels · Embassy tasks        │
│  platform_id gating · BLE/MQTT routing  │
└─────────────────────────────────────────┘
```

| Crate | Role |
|---|---|
| `core-interface` | Channel definitions, shared types, dispatcher tasks, CAN filtering |
| `boards/esp`, `boards/pc` | Hardware drivers (BLE, WiFi/MQTT, CAN); ESP HAL deps sit behind an optional `hardware` feature |
| `cars/*` | Vehicle logic: CAN decoding, command handling, state encoding |
| `xtask` | Build orchestrator: reads `config.toml`, generates `.app_build/`, invokes cargo |

Details: [Architecture](docs/architecture.md) · [BLE](docs/ble.md) · [CAN bus](docs/can.md)

## 📂 Project Structure

```
.
├── boards/
│   ├── esp/            # ESP32 board crate + main.template.rs + xtask_builder.rs
│   └── pc/             # PC/Linux board crate + main.template.rs + xtask_builder.rs
├── build-utils/        # Shared helpers for build scripts
├── cars/
│   ├── egmp/           # Hyundai/Kia E-GMP vehicle crate
│   └── virtual-car/    # virtual-car-controller (mock for UI/UX testing)
├── contracts/          # Protobuf definitions + meta.toml per vehicle platform
├── core-interface/     # Shared channels, types, and dispatcher tasks
├── docs/               # Architecture, BLE, CAN, testing, release docs
├── hardware-profiles/  # Per-device hardware presets (pins, CAN setup)
├── xtask/              # Build system (cargo xtask ...)
├── config.toml.example # Template for the gitignored config.toml
└── AGENTS.md           # Rules for AI agents (CLAUDE.md imports it)
```

## 🧪 Running Tests

```bash
cargo xtask test                                  # all host tests, no hardware needed
cargo xtask test --board esp --on-hardware        # on-device tests (ESP32, requires probe-rs)
```

When running `cargo test -p <crate>` directly, pass `-- --test-threads=1`: the channel statics are process-global, so parallel threads corrupt each other's state. Avoid `--workspace`, which would compile `board-esp` with its ESP HAL deps. See [Testing](docs/testing.md) for the per-crate commands and how the suites are organised.

## 📦 Releases

CI runs `fmt`, diff-based host tests, and `clippy` plus full builds for both `esp` and `pc` on every affected vehicle platform. On every push to `main`, `auto-release.yml` tags each platform with changes as `car-<platform>-v<semver>` (bumped from the conventional commits); `release-plz` is not used yet. Release builds attach one ESP image per hardware profile to a GitHub Release. CI also publishes a static OTA `manifest.json` with a `min_app_version` compatibility matrix.

To produce a flashable `.bin` yourself, or for the details of versioning and OTA, see [Releases, flash images & OTA](docs/releases.md).

## 🤝 Contributing

Contributions are welcome; please read [CONTRIBUTING.md](CONTRIBUTING.md) first and open an issue before starting anything large. Report security problems privately as described in [SECURITY.md](SECURITY.md).

## 📜 License

This project is licensed under the terms of the [LICENSE](LICENSE) file.
