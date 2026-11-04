use std::io::Result;

fn main() -> Result<()> {
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/core.proto");
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/system.proto");
    println!("cargo:rerun-if-changed=../contracts/opencar/core/v1/transport.toml");

    // `BTreeMap` for proto maps: the default `HashMap` needs std, and this crate is no_std.
    prost_build::Config::new().btree_map(["."]).compile_protos(
        &["../contracts/opencar/core/v1/core.proto"],
        &["../contracts/"],
    )?;

    generate_ble_transport_constants()?;
    generate_phone_validation_constants()?;

    Ok(())
}

fn generate_ble_transport_constants() -> Result<()> {
    let toml_str = std::fs::read_to_string("../contracts/opencar/core/v1/transport.toml")
        .expect("Failed to read contracts/opencar/core/v1/transport.toml");

    let transport: toml::Value = toml::from_str(&toml_str).expect("Failed to parse transport.toml");

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
    // Added after the other keys: point at a stale contracts checkout rather
    // than failing with a bare "index not found".
    let payload_key = |key: &str| {
        transport["ble"]
            .get("payload")
            .and_then(|p| p.get(key))
            .and_then(|v| v.as_integer())
            .unwrap_or_else(|| {
                panic!("transport.toml is missing integer ble.payload.{key}; update the contracts submodule")
            })
    };
    let att_mtu = payload_key("att_mtu");
    let max_chunk_bytes = payload_key("max_chunk_bytes");
    let max_message_bytes = payload_key("max_message_bytes");
    // A notification or write carries the ATT MTU minus the 3-byte ATT header.
    assert!(
        max_chunk_bytes == att_mtu - 3,
        "transport.toml ble.payload.max_chunk_bytes ({max_chunk_bytes}) must be att_mtu - 3 ({})",
        att_mtu - 3
    );
    let frag_key = |key: &str| {
        transport["ble"]
            .get("fragmentation")
            .and_then(|p| p.get(key))
            .and_then(|v| v.as_integer())
            .unwrap_or_else(|| {
                panic!("transport.toml is missing integer ble.fragmentation.{key}; update the contracts submodule")
            })
    };
    let header_bytes = frag_key("header_bytes");
    let last_chunk_flag = frag_key("last_chunk_flag");
    let chunk_index_mask = frag_key("chunk_index_mask");
    let max_chunks = frag_key("max_chunks");
    let crc_bytes = frag_key("crc_bytes");
    let max_chunk_data_bytes = frag_key("max_chunk_data_bytes");
    let min_att_mtu = frag_key("min_att_mtu");
    let reassembly_timeout_ms = frag_key("reassembly_timeout_ms");
    assert!(
        header_bytes == 2
            && last_chunk_flag == 0x80
            && chunk_index_mask == 0x7f
            && max_chunks == chunk_index_mask + 1
            && crc_bytes == 4,
        "transport.toml ble.fragmentation header layout changed; update core_interface::ble"
    );
    assert!(
        max_chunk_data_bytes == max_chunk_bytes - header_bytes,
        "transport.toml ble.fragmentation.max_chunk_data_bytes must be max_chunk_bytes - header_bytes"
    );
    assert!(
        max_chunks * (min_att_mtu - 3 - header_bytes) >= max_message_bytes + crc_bytes,
        "transport.toml min_att_mtu cannot carry max_message_bytes and its CRC in max_chunks chunks"
    );

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let out_path = std::path::Path::new(&out_dir).join("ble_transport.rs");
    std::fs::write(
        &out_path,
        format!(
            "// @generated — do not edit. Source: contracts/opencar/core/v1/transport.toml\n\
             pub const GATT_SERVICE_UUID: u128 = 0x{service_uuid:032x};\n\
             pub const GATT_RX_UUID: u128 = 0x{rx_uuid:032x};\n\
             pub const GATT_TX_UUID: u128 = 0x{tx_uuid:032x};\n\
             pub const BLE_PAIRING_WINDOW_SECONDS: u32 = {pairing_window_s};\n\
             /// ATT MTU the app requests; the BLE stack must support at least this.\n\
             pub const BLE_ATT_MTU: u16 = {att_mtu};\n\
             /// Largest encoded `AppToDevice`/`DeviceToApp` ATT value (one write or\n\
             /// notification): one chunk, header included.\n\
             pub const BLE_MAX_CHUNK_BYTES: usize = {max_chunk_bytes};\n\
             /// Largest reassembled `AppToDevice`/`DeviceToApp` message.\n\
             pub const BLE_MAX_MESSAGE_BYTES: usize = {max_message_bytes};\n\
             /// Chunk header size, in bytes.\n\
             pub const BLE_CHUNK_HEADER_BYTES: usize = {header_bytes};\n\
             /// Chunk header bit set on the last chunk of a message.\n\
             pub const BLE_CHUNK_LAST_FLAG: u8 = {last_chunk_flag};\n\
             /// Chunk header bits holding the chunk index.\n\
             pub const BLE_CHUNK_INDEX_MASK: u8 = {chunk_index_mask};\n\
             /// CRC-32 bytes after the message in the message stream.\n\
             pub const BLE_CRC_BYTES: usize = {crc_bytes};\n\
             /// Most chunks a message can have.\n\
             pub const BLE_MAX_CHUNKS: usize = {max_chunks};\n\
             /// Smallest negotiated ATT MTU that can carry `BLE_MAX_MESSAGE_BYTES`.\n\
             pub const BLE_MIN_ATT_MTU: u16 = {min_att_mtu};\n\
             /// A partial message is dropped if the next chunk is later than this.\n\
             pub const BLE_REASSEMBLY_TIMEOUT_MS: u64 = {reassembly_timeout_ms};\n"
        ),
    )?;

    Ok(())
}

/// `[ble.phone_validation]`: the session limits, shared with the app.
fn generate_phone_validation_constants() -> Result<()> {
    let toml_str = std::fs::read_to_string("../contracts/opencar/core/v1/transport.toml")
        .expect("Failed to read contracts/opencar/core/v1/transport.toml");
    let transport: toml::Value = toml::from_str(&toml_str).expect("Failed to parse transport.toml");
    let key = |key: &str| {
        transport["ble"]
            .get("phone_validation")
            .and_then(|p| p.get(key))
            .and_then(|v| v.as_integer())
            .unwrap_or_else(|| {
                panic!("transport.toml is missing integer ble.phone_validation.{key}; update the contracts submodule")
            })
    };
    let session_seconds = key("session_seconds");
    let code_length = key("code_length");
    let max_code_attempts = key("max_code_attempts");
    let cooldown_seconds = key("cooldown_seconds");
    let result_keep_seconds = key("result_keep_seconds");
    assert!(
        session_seconds > 0 && cooldown_seconds >= 0 && result_keep_seconds >= 0,
        "transport.toml ble.phone_validation durations must not be negative, and session_seconds > 0"
    );
    // The code is drawn from a u32, so at most 9 digits.
    assert!(
        (4..=9).contains(&code_length),
        "transport.toml ble.phone_validation.code_length must be 4 to 9 digits"
    );
    assert!(
        max_code_attempts > 0,
        "transport.toml ble.phone_validation.max_code_attempts must be > 0"
    );

    let out_dir = std::env::var("OUT_DIR").unwrap();
    std::fs::write(
        std::path::Path::new(&out_dir).join("phone_validation.rs"),
        format!(
            "// @generated — do not edit. Source: contracts/opencar/core/v1/transport.toml\n\
             /// Digits in a validation code.\n\
             pub const CODE_LENGTH: usize = {code_length};\n\
             /// Wrong codes a session accepts before it fails.\n\
             pub const MAX_CODE_ATTEMPTS: u32 = {max_code_attempts};\n\
             /// How long a session runs before it times out.\n\
             pub const VALIDATION_SESSION_MS: u64 = {session_seconds} * 1_000;\n\
             /// How long no session can start after one used all its attempts.\n\
             pub const VALIDATION_COOLDOWN_MS: u64 = {cooldown_seconds} * 1_000;\n\
             /// How long an ended session is still reported to its phone, so the app sees how\n\
             /// it ended.\n\
             pub const VALIDATION_RESULT_KEEP_MS: u64 = {result_keep_seconds} * 1_000;\n"
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
