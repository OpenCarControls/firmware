# Native Windows Setup (Optional)

While the dev container is the primary supported environment, you can set up a local Windows environment by installing the necessary tools directly on your host machine.

The PC board is used for local debugging and testing without real hardware. Compiling it natively on Windows is supported and automatically uses a dummy CAN implementation, since the real `socketcan` Linux API is not available.

## Toolchain Prerequisites

### 1. Install Rust
Install Rust using `rustup` by downloading `rustup-init.exe` from [rustup.rs](https://rustup.rs/), or using `winget`:
```powershell
winget install Rustlang.Rustup
```

### 2. Install ESP Tooling
To compile for ESP targets natively, you will need Espressif's Rust tooling. We recommend `cargo-binstall` to fetch pre-compiled binaries and save compilation time.

```powershell
# Install cargo-binstall
Set-ExecutionPolicy Unrestricted -Scope Process; Invoke-Expression (Invoke-WebRequest -Uri https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.ps1 -UseBasicParsing).Content

# Install Espressif tools
cargo binstall -y espup espflash probe-rs-tools
```

### 3. Install ESP Toolchains
Run `espup` to install the Xtensa and RISC-V toolchains.
```powershell
espup install
```
**CRITICAL:** `espup` generates an environment export script (e.g., `export-esp.ps1` in your home directory). You **must** source this script in your PowerShell profile or in the terminal session before running `cargo xtask build --board esp` to ensure the correct compilers are in your `PATH`.

## VS Code rust-analyzer
Run `cargo xtask ide` to point rust-analyzer at the board selected in `config.toml`, or `cargo xtask ide --board pc` for the PC board. It writes the matching features, target and toolchain to `.vscode/settings.json`; see [Switching boards in the IDE](../README.md#switching-boards-in-the-ide).

## Compiling and Running
To build the PC board natively:
```powershell
cargo xtask build --board pc
```

To run the firmware natively (testing UI over MQTT/BLE without real hardware):
```powershell
cargo run -p app-build
```
*(Ensure you have copied `config.toml.example` to `config.toml` and populated it first).*
