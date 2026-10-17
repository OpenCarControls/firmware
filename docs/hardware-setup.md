# Flashing & debugging hardware

The ESP dev container (`Open Car — ESP`, see `.devcontainer/esp/`) runs with `--privileged` and bind-mounts `/dev`, giving it direct access to any USB device that appears on the host. The setup depends on where Docker is running.

## Dev container running locally

Plug in the ESP32. The device appears immediately inside the container as `/dev/ttyACM0` (native USB) or `/dev/ttyUSB0` (UART-chip boards). No extra setup needed.

## Dev container in a remote VM (VS Code Remote SSH)

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

## Flashing

```bash
cargo xtask flash --port /dev/ttyACM0 --board esp   # builds, then flashes; add --monitor to attach to the serial console
```

or manually:

```bash
cargo xtask build --board esp
espflash flash --port /dev/ttyACM0 .app_build/target/xtensa-esp32s3-none-elf/release/app-build
```

To build an image in the container and flash it from the host instead, see [Releases](releases.md#building-a-flashable-bin).

## Debugging

The ESP dev container includes the `probe-rs-debugger` VS Code extension. Run `cargo xtask ide` so the debug config matches your chip, then select **"Firmware: debug ESP32-… (probe-rs)"** in the Run & Debug panel and press F5. It builds with `--debug` first. probe-rs flashes the binary and halts the CPU at reset, giving you breakpoints, variable inspection, and step execution.

## Serial logs

The ESP32-S3's USB_SERIAL_JTAG peripheral exposes a second CDC-ACM interface alongside the JTAG debug channel. Logs written via `esp-println` / the `log` crate appear on `/dev/ttyACM0` independently of the debugger. The ESP container includes the **Serial Monitor** extension (View → Serial Monitor), pre-configured for `/dev/ttyACM0` at 115200 baud. You can also read logs directly:

```bash
stty -F /dev/ttyACM0 raw 115200 && cat /dev/ttyACM0
```
