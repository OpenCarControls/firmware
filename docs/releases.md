# Releases, flash images & OTA

## Versioning & CI/CD
*   **Independent crates:** managed via `release-plz` (not implemented yet, waiting for at least one car implementation to work). Changes to the core cascade safely to vehicle crates.
*   **Tagging:** prefixed tags (e.g. `car-virtual-car-v0.2.1`).
*   **Release workflow (`auto-release.yml`):** on every push to `main`, each platform with changes since its last tag (in `cars/<platform>/`, `core-interface/`, `boards/` or `hardware-profiles/`) gets a new `car-<platform>-v<semver>` tag, bumped from the conventional commits. Several platforms can be tagged on the same commit.
*   **Release assets:** one merged ESP image per hardware profile in `hardware-profiles/`, named `open-car-<platform>-<profile>-v<version>.bin`, plus `checksums.sha256`. Pins, LEDs and the CAN transceiver are specific to each device, so there is no generic per-MCU image; releasing for a new device means adding its profile. Profiles with fewer CAN buses than the platform needs are skipped (`.github/scripts/release_profiles.py`). The PC board isn't released.
*   Release images are built from `config.toml.example` with only `hardware_profile` changed, so they carry its placeholder WiFi and MQTT settings.

## OTA manifest generation
The CI/CD pipeline generates static `manifest.json` files hosted on GitHub Pages. This includes a compatibility matrix (`min_app_version`) to ensure the hardware is never updated to a Protobuf version the phone app cannot decode.

## Building a flashable BIN
Building inside a dev container? Generate the image in-container and flash it later from your host machine.

1.  Build the ELF: `cargo xtask build --board esp`.
2.  Export it with `espflash save-image` (see below).
3.  Copy `./dist/*.bin` to your local machine and flash there.

Target triples by MCU (the `<target-triple>` in the paths below):

*   `esp32` → `xtensa-esp32-none-elf`
*   `esp32s3` → `xtensa-esp32s3-none-elf`
*   `esp32c3` → `riscv32imc-unknown-none-elf`
*   `esp32c6` → `riscv32imac-unknown-none-elf`

## ESP flash image export (espflash)
*   `cargo xtask build --board esp` produces an `esp-hal` ELF at `.app_build/target/<target-triple>/release/app-build`.
*   `--ignore-app-descriptor` and `--min-chip-rev` are not needed. `espflash save-image` works on `esp-hal` binaries because the firmware calls `esp_bootloader_esp_idf::esp_app_desc!()`, which emits a valid `esp_app_desc_t` struct with `min_efuse_blk_rev_full = 0`. Without this, the ESP-IDF 2nd stage bootloader reads garbage from that field and rejects the image with `Image requires efuse blk rev >= vXXX.XX`.
*   Working command (single merged bin):
    ```
    espflash save-image --chip <mcu> --merge .app_build/target/<target-triple>/release/app-build ./dist/<name>.bin
    ```
*   Example for ESP32-S3:
    ```
    espflash save-image --chip esp32s3 --merge .app_build/target/xtensa-esp32s3-none-elf/release/app-build ./dist/open-car-esp32s3-merged.bin
    ```
*   The merged output is ready to flash from a host machine outside the dev container.
