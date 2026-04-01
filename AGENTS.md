# Context: Open Car Firmware (Rust)

## 📌 Project Role
This is the embedded Rust monorepo for the hardware controller (ESP32 / Raspberry Pi). It handles CAN bus reading/writing, BLE/WiFi/LTE connections, and OTA updates. 

## 🏗️ Architecture: Cargo Workspace & xtask
*   **`core-interface`:** The generic hardware and networking layer. Knows nothing about specific cars. Wraps specific byte payloads into the `core.proto` MessageEnvelope and broadcasts them.
*   **Vehicle Crates (e.g., `car-hmg`):** Contain CAN mappings and logic. They depend on `core-interface`.
*   **`car-virtual`:** A mock implementation used strictly for UI/UX testing without physical hardware.
*   **Dynamic Builds:** A custom `xtask` reads a `config.toml` file to dynamically select the board target and inject the correct vehicle crate into a final, ephemeral `.app_build` crate.

## 🏷️ Versioning & CI/CD
*   **Independent Crates:** Managed via `release-plz` (not implemented yet, waiting for at least one car implementation to work). Changes to the core cascade safely to vehicle crates.
*   **Tagging:** Uses prefixed tags (e.g., `car-hmg-v0.2.1`).
*   **Agnostic Pipelines:** GitHub Actions trigger on `*-v*.*.*`. The YAML dynamically parses the tag to extract the car name, passes it to `xtask`, builds the `.bin`, and uploads it to GitHub Releases.

## 📡 OTA Manifest Generation
The CI/CD pipeline generates static `manifest.json` files hosted on GitHub Pages. This includes a Compatibility Matrix (`min_app_version`) to ensure the hardware is never updated to a Protobuf version the phone app cannot decode.
