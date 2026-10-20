# Testing

The README has the quick-start commands; this page covers how the suites are organised and why.

## Host tests
`cargo xtask test` runs all host-side tests in sequence without hardware. They use standard `cargo test` and need no ESP toolchain.

| Crate | Covers | Command |
|---|---|---|
| `core-interface` | `passes_filter`, BLE/MQTT dispatch (platform_id gating, routing, `source_device_id` stamping, empty-source rejection), response routing, state publish split, CAN read-only flag and its change notifications, per-bus CAN TX channels, CAN debug streaming (batch filtering, blocklist, dropped-frame counter, splitting batches to fit one BLE message) | `cargo test -p core-interface -- --test-threads=1` |
| `virtual-car-controller` | `encode_state` proto encoding, worst-case state update fits one BLE message, CAN frame → state updates, `process_basic_command`, `tick_simulation` (initial defaults, ramp-up, park phase, odometer increment, door-lock preservation, cycle wraparound) | `cargo test -p virtual-car-controller -- --test-threads=1` |
| `egmp-controller` | New state publishes no invented values, worst-case state update fits one BLE message, decoding of real CAN frames (charge port, windows, hazards), charge port state through AC and DC sessions, battery power, range, charging times, charge limits, climate state, temperatures in both units, preconditioning states, battery temperatures and 12 V voltage, distance unit, regen mode and level, tire pressures | `cargo test -p egmp-controller -- --test-threads=1` |
| `board-pc` | `socketcan_to_core_frame` / `core_to_socketcan_frame` round-trips + BLE helpers | `cargo test -p board-pc -- --test-threads=1` |
| `board-esp` | `compute_mcp_masks`, `to_core_frame`, CAN RX stats counters, the `StoredBond` flash layout and round-trips, BLE helpers (device name and fallback, random-static address detection) | `cargo test -p board-esp -- --test-threads=1` |

**`--test-threads=1` is required** — Embassy channels are process-global statics; parallel threads corrupt each other's state.

## On-hardware tests (ESP32, requires `probe-rs`)
```
cargo xtask test --board esp --on-hardware
```
xtask generates `.app_test_build/` (parallel to `.app_build/`), compiles it with the Xtensa toolchain using `embedded-test 0.7` (features: `external-executor`, `embassy-09`, `xtensa-semihosting`, `panic-handler`) and flashes/runs via `probe-rs run` (invoked with `sudo` because the USB device is `root:root` in the dev container).

Test results are reported via semihosting — no defmt or RTT involved. The template (`boards/esp/tests.template.rs`) uses plain `assert!`/`assert_eq!` and includes channel round-trip and `passes_filter` smoke tests. Add custom on-device tests to `boards/esp/tests/hardware.rs` — xtask injects that file automatically.

## `no_std` gating
See [Architecture](architecture.md#no_std--test-gating-pattern) for how crates compile on the host for testing.

## CI
The `test` job in `.github/workflows/ci.yml` uses git diff to detect which crate directories changed and runs only the affected packages:
*   Changes to `boards/pc/` trigger `board-pc` tests.
*   Changes to `boards/esp/` trigger `board-esp` tests.
*   Changes to `core-interface/` trigger all four crates.

The `build-and-lint` job runs `cargo xtask clippy` and `cargo xtask build` for each board (`esp`, `pc`) and each affected vehicle platform, using `config.toml.example`. A platform is affected when its `cars/<platform>/` directory changed. Changes to shared code (`core-interface/`, `boards/`, `build-utils/`, `xtask/`, `hardware-profiles/`, the `contracts` pin, the workspace `Cargo.toml`/`Cargo.lock`, `config.toml.example` or `ci.yml`) check every platform.
Their check names change with the platforms, so branch protection requires the summary `Build & Lint` job instead, which passes only when formatting, platform detection and every build-and-lint run passed (or no platform needed building).
