use std::io::Result;

fn main() -> Result<()> {
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/core.proto");
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/system.proto");
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/transport.toml");

    prost_build::compile_protos(
        &["../contracts/opencar/core/v1/core.proto"],
        &["../contracts/"],
    )?;

    generate_ble_transport_constants()?;

    Ok(())
}

fn generate_ble_transport_constants() -> Result<()> {
    let toml_str = std::fs::read_to_string("../contracts/opencar/core/v1/transport.toml")
        .expect("Failed to read contracts/opencar/core/v1/transport.toml");

    let transport: toml::Value =
        toml::from_str(&toml_str).expect("Failed to parse transport.toml");

    let service_uuid = parse_uuid(
        transport["ble"]["service"]["uuid"]
            .as_str()
            .expect("ble.service.uuid must be a string"),
        "ble.service.uuid",
    );
    let rx_uuid = parse_uuid(
        transport["ble"]["characteristics"]["app_to_device"]["uuid"]
            .as_str()
            .expect("ble.characteristics.app_to_device.uuid must be a string"),
        "ble.characteristics.app_to_device.uuid",
    );
    let tx_uuid = parse_uuid(
        transport["ble"]["characteristics"]["device_to_app"]["uuid"]
            .as_str()
            .expect("ble.characteristics.device_to_app.uuid must be a string"),
        "ble.characteristics.device_to_app.uuid",
    );
    let pairing_window_s = transport["ble"]["pairing"]["pairing_window_seconds"]
        .as_integer()
        .expect("ble.pairing.pairing_window_seconds must be an integer");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_path = std::path::Path::new(&out_dir).join("ble_transport.rs");
    std::fs::write(
        &out_path,
        format!(
            "// @generated — do not edit. Source: contracts/opencar/core/v1/transport.toml\n\
             pub const GATT_SERVICE_UUID: u128 = 0x{service_uuid:032x};\n\
             pub const GATT_RX_UUID: u128 = 0x{rx_uuid:032x};\n\
             pub const GATT_TX_UUID: u128 = 0x{tx_uuid:032x};\n\
             pub const BLE_PAIRING_WINDOW_SECONDS: u32 = {pairing_window_s};\n"
        ),
    )?;

    Ok(())
}

fn parse_uuid(uuid: &str, field: &str) -> u128 {
    let hex: String = uuid.chars().filter(|c| *c != '-').collect();
    assert!(
        hex.len() == 32,
        "transport.toml {field} must be a 128-bit UUID (32 hex digits), got: {uuid}"
    );
    u128::from_str_radix(&hex, 16)
        .unwrap_or_else(|_| panic!("transport.toml {field} is not valid hex: {uuid}"))
}
