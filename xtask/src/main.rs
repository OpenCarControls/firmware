use serde::Deserialize;
use std::env;
use std::fs;
use std::process::{Command, exit};

// ==========================================
// 1. TOML Configuration Data Models
// ==========================================

#[derive(Deserialize)]
pub struct Config {
    pub target: TargetConfig,
    pub hardware: toml::Value,
    pub network: NetworkConfig,
    /// Populated at runtime from `[workspace.dependencies]` in the root `Cargo.toml`.
    /// Not deserialized from `config.toml` — set manually after parsing.
    #[serde(skip, default = "default_empty_toml_table")]
    pub workspace_deps: toml::Value,
}

fn default_empty_toml_table() -> toml::Value {
    toml::Value::Table(Default::default())
}

#[derive(Deserialize)]
pub struct TargetConfig {
    pub board: String, // e.g., "esp" or "pc"
    pub platform: String,
}

#[derive(Deserialize)]
pub struct NetworkConfig {
    pub mqtt: MqttConfig,
    #[serde(default)]
    pub wifi: WifiConfig,
}

#[derive(Deserialize, Default)]
pub struct WifiConfig {
    #[serde(default = "wifi_enabled_default")]
    pub enabled: bool,
    pub ssid: Option<String>,
    pub password: Option<String>,
}

fn wifi_enabled_default() -> bool {
    true
}

#[derive(Deserialize)]
pub struct MqttConfig {
    pub broker_url: String,
    pub client_id: String,
    pub auth_mode: String,
    // Used for mTLS
    pub ca_cert_file: Option<String>,
    pub client_cert_file: Option<String>,
    pub client_key_file: Option<String>,
    // Used for basic auth
    pub username: Option<String>,
    pub password: Option<String>,
}

#[derive(Deserialize)]
pub struct TransportConfig {
    pub mqtt: TransportMqtt,
    pub ble: TransportBle,
}

#[derive(Deserialize)]
pub struct TransportMqtt {
    pub command_topic_template: String,
    pub data_topic_template: String,
}

#[derive(Deserialize)]
pub struct TransportBle {
    pub service: TransportBleService,
    pub characteristics: TransportBleCharacteristics,
    pub pairing: TransportBlePairing,
}

#[derive(Deserialize)]
pub struct TransportBlePairing {
    pub pairing_window_seconds: u32,
}

#[derive(Deserialize)]
pub struct TransportBleService {
    pub name: String,
    pub uuid: String,
}

#[derive(Deserialize)]
pub struct TransportBleCharacteristics {
    pub app_to_device: TransportBleCharacteristic,
    pub device_to_app: TransportBleCharacteristic,
}

#[derive(Deserialize)]
pub struct TransportBleCharacteristic {
    pub uuid: String,
    pub payload: String,
    pub properties: Vec<String>,
    pub direction: String,
}

/// Returns the version string for a `[workspace.dependencies]` entry.
/// Handles both `dep = "1.2.3"` (plain string) and `dep = { version = "1.2.3", ... }` (table).
pub fn ws_dep_version(workspace_deps: &toml::Value, name: &str) -> String {
    let dep = workspace_deps.get(name).unwrap_or_else(|| {
        panic!(
            "❌ Missing workspace dependency '{}'. Add it to [workspace.dependencies] in Cargo.toml.",
            name
        )
    });
    if let Some(s) = dep.as_str() {
        s.to_string()
    } else if let Some(v) = dep.get("version").and_then(|v| v.as_str()) {
        v.to_string()
    } else {
        panic!("❌ Workspace dependency '{}' has no 'version' field.", name)
    }
}

/// Strips the scheme from a broker URL and returns `(host, port)` as strings.
/// Falls back to port 1883 for `mqtt://` and 8883 for `mqtts://`.
pub fn parse_broker_url(url: &str) -> (String, u16) {
    let (scheme, rest) = if let Some(r) = url.strip_prefix("mqtts://") {
        ("mqtts", r)
    } else if let Some(r) = url.strip_prefix("mqtt://") {
        ("mqtt", r)
    } else {
        eprintln!(
            "❌ broker_url must start with mqtt:// or mqtts://, got: {}",
            url
        );
        std::process::exit(1);
    };
    let default_port: u16 = if scheme == "mqtts" { 8883 } else { 1883 };
    // strip any path component
    let host_port = rest.split('/').next().unwrap_or(rest);
    if let Some(colon) = host_port.rfind(':') {
        let host = &host_port[..colon];
        let port_str = &host_port[colon + 1..];
        let port = port_str.parse::<u16>().unwrap_or_else(|_| {
            eprintln!("❌ Invalid port in broker_url: {}", port_str);
            std::process::exit(1);
        });
        (host.to_string(), port)
    } else {
        (host_port.to_string(), default_port)
    }
}

pub fn load_transport_contract() -> TransportConfig {
    let path = "contracts/opencar/core/v1/transport.toml";
    let content = fs::read_to_string(path)
        .unwrap_or_else(|_| panic!("❌ Failed to read transport contract: {}", path));
    toml::from_str(&content).expect("❌ Failed to parse transport.toml")
}

pub fn render_topic_from_template(template: &str, client_id: &str, field_name: &str) -> String {
    if !template.contains("{client_id}") {
        eprintln!(
            "❌ transport.toml {} must include '{{client_id}}': {}",
            field_name, template
        );
        exit(1);
    }
    template.replace("{client_id}", client_id)
}

pub fn escape_rust_string(value: &str) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}

pub fn parse_uuid_u128(uuid: &str, field_name: &str) -> u128 {
    let hex: String = uuid.chars().filter(|c| *c != '-').collect();
    if hex.len() != 32 {
        eprintln!(
            "❌ transport.toml {} must be a 128-bit UUID, got: {}",
            field_name, uuid
        );
        exit(1);
    }
    u128::from_str_radix(&hex, 16).unwrap_or_else(|_| {
        eprintln!(
            "❌ transport.toml {} is not valid hex UUID: {}",
            field_name, uuid
        );
        exit(1);
    })
}

pub fn validate_ble_transport_contract(transport: &TransportConfig) {
    let app_to_device = &transport.ble.characteristics.app_to_device;
    let device_to_app = &transport.ble.characteristics.device_to_app;

    if app_to_device.payload != "opencar.core.v1.AppToDevice" {
        eprintln!(
            "❌ transport.toml ble.characteristics.app_to_device.payload must be opencar.core.v1.AppToDevice"
        );
        exit(1);
    }
    if app_to_device.direction != "app -> device" {
        eprintln!(
            "❌ transport.toml ble.characteristics.app_to_device.direction must be 'app -> device'"
        );
        exit(1);
    }
    if app_to_device.properties.len() != 2
        || !app_to_device.properties.iter().any(|p| p == "write")
        || !app_to_device
            .properties
            .iter()
            .any(|p| p == "write_without_response")
    {
        eprintln!(
            "❌ transport.toml ble.characteristics.app_to_device.properties must be [\"write\", \"write_without_response\"]"
        );
        exit(1);
    }

    if device_to_app.payload != "opencar.core.v1.DeviceToApp" {
        eprintln!(
            "❌ transport.toml ble.characteristics.device_to_app.payload must be opencar.core.v1.DeviceToApp"
        );
        exit(1);
    }
    if device_to_app.direction != "device -> app" {
        eprintln!(
            "❌ transport.toml ble.characteristics.device_to_app.direction must be 'device -> app'"
        );
        exit(1);
    }
    if device_to_app.properties.len() != 1 || device_to_app.properties[0] != "notify" {
        eprintln!(
            "❌ transport.toml ble.characteristics.device_to_app.properties must be [\"notify\"]"
        );
        exit(1);
    }

    let _ = parse_uuid_u128(&transport.ble.service.uuid, "ble.service.uuid");
    let _ = parse_uuid_u128(
        &transport.ble.characteristics.app_to_device.uuid,
        "ble.characteristics.app_to_device.uuid",
    );
    let _ = parse_uuid_u128(
        &transport.ble.characteristics.device_to_app.uuid,
        "ble.characteristics.device_to_app.uuid",
    );

    if transport.ble.pairing.pairing_window_seconds == 0 {
        eprintln!("❌ transport.toml ble.pairing.pairing_window_seconds must be > 0");
        exit(1);
    }
}

pub fn validate_esp_ble_uuid_constants_against_contract(transport: &TransportConfig) {
    let path = "boards/esp/src/ble/mod.rs";
    let src = fs::read_to_string(path)
        .unwrap_or_else(|_| panic!("❌ Failed to read ESP BLE module: {}", path));

    let service_expected = format!(
        "GATT_SERVICE_UUID: u128 = 0x{:032x}",
        parse_uuid_u128(&transport.ble.service.uuid, "ble.service.uuid")
    );
    let rx_expected = format!(
        "GATT_RX_UUID: u128 = 0x{:032x}",
        parse_uuid_u128(
            &transport.ble.characteristics.app_to_device.uuid,
            "ble.characteristics.app_to_device.uuid"
        )
    );
    let tx_expected = format!(
        "GATT_TX_UUID: u128 = 0x{:032x}",
        parse_uuid_u128(
            &transport.ble.characteristics.device_to_app.uuid,
            "ble.characteristics.device_to_app.uuid"
        )
    );

    // Ignore readability separators so the check is resilient to underscore formatting.
    let normalized_src = src.replace('_', "");
    let service_ok = normalized_src.contains(&service_expected.replace('_', ""));
    let rx_ok = normalized_src.contains(&rx_expected.replace('_', ""));
    let tx_ok = normalized_src.contains(&tx_expected.replace('_', ""));

    if !(service_ok && rx_ok && tx_ok) {
        eprintln!(
            "❌ ESP BLE UUID constants in {} are out of sync with contracts/opencar/core/v1/transport.toml",
            path
        );
        eprintln!(
            "   Expected service/rx/tx UUIDs: {}, {}, {}",
            transport.ble.service.uuid,
            transport.ble.characteristics.app_to_device.uuid,
            transport.ble.characteristics.device_to_app.uuid
        );
        exit(1);
    }
}

// ==========================================
// 2. The Main CLI Entrypoint
// ==========================================

fn main() {
    let args: Vec<String> = env::args().collect();

    let command = args.get(1).map(|s| s.as_str()).unwrap_or("");
    match command {
        "build" | "run" | "clippy" | "test" => {}
        _ => {
            eprintln!(
                "Usage: cargo xtask <build|run|clippy|test> [config_file.toml] [--board <board>] [--platform <platform>] [--on-hardware]"
            );
            exit(1);
        }
    }

    // Parse optional positional config path and named overrides from remaining args
    let mut config_path = "config.toml";
    let mut override_board: Option<String> = None;
    let mut override_platform: Option<String> = None;
    let mut on_hardware = false;

    let mut remaining = args[2..].iter();
    while let Some(arg) = remaining.next() {
        match arg.as_str() {
            "--board" => {
                override_board = remaining.next().cloned();
            }
            "--platform" => {
                override_platform = remaining.next().cloned();
            }
            "--on-hardware" => {
                on_hardware = true;
            }
            other if !other.starts_with("--") => {
                config_path = other;
            }
            other => {
                eprintln!("Unknown argument: {}", other);
                exit(1);
            }
        }
    }

    // Auto-create config.toml from the example template on first use
    if config_path == "config.toml" && !std::path::Path::new("config.toml").exists() {
        if std::path::Path::new("config.toml.example").exists() {
            fs::copy("config.toml.example", "config.toml")
                .expect("❌ Failed to copy config.toml.example → config.toml");
            println!("📋 Created config.toml from config.toml.example — edit it to match your setup.");
        }
    }

    println!("🚀 Starting custom {} using: {}", command, config_path);

    // Read and parse the config file, then apply any CLI overrides
    let config_str = fs::read_to_string(config_path)
        .unwrap_or_else(|_| panic!("Failed to read config file: {}", config_path));
    let mut config: Config =
        toml::from_str(&config_str).expect("Failed to parse TOML configuration");

    // Load workspace dependency versions from root Cargo.toml so builders don't hardcode them.
    let ws_toml_str =
        fs::read_to_string("Cargo.toml").expect("❌ Could not read workspace Cargo.toml");
    let ws_toml: toml::Value =
        toml::from_str(&ws_toml_str).expect("❌ Failed to parse workspace Cargo.toml");
    config.workspace_deps = ws_toml["workspace"]["dependencies"].clone();

    if let Some(board) = override_board {
        config.target.board = board;
    }
    if let Some(platform) = override_platform {
        config.target.platform = platform;
    }

    // Host tests: no board-specific build needed
    if command == "test" && !on_hardware {
        run_host_tests(&config);
        return;
    }

    let builder = get_builder(&config.target.board);

    builder.validate(&config);

    match command {
        "test" => {
            // on_hardware is true here (handled above otherwise)
            builder.generate_app_test_build(&config);
            builder.test_hardware(&config);
        }
        _ => {
            builder.generate_app_build(&config);
            match command {
                "run" => builder.run(&config),
                "clippy" => builder.clippy(&config),
                "build" => builder.compile(&config),
                _ => unreachable!(),
            }
        }
    }
}

// ==========================================
// 3. Host Test Runner
// ==========================================

fn run_host_tests(config: &Config) {
    // Discover the configured vehicle crate name
    let vehicle_cargo_path = format!(
        "cars/{}/Cargo.toml",
        config.target.platform
    );
    let vehicle_cargo_str = fs::read_to_string(&vehicle_cargo_path).unwrap_or_else(|_| {
        panic!(
            "❌ Could not read vehicle Cargo.toml: {}",
            vehicle_cargo_path
        )
    });
    let vehicle_cargo: toml::Value =
        toml::from_str(&vehicle_cargo_str).expect("❌ Invalid vehicle Cargo.toml");
    let vehicle_crate = vehicle_cargo["package"]["name"]
        .as_str()
        .expect("❌ Missing [package.name] in vehicle Cargo.toml")
        .to_string();

    let packages = [
        "core-interface",
        "board-pc",
        "board-esp",
        vehicle_crate.as_str(),
    ];

    for pkg in packages {
        println!("🧪 Testing {}...", pkg);
        let status = Command::new("cargo")
            .args(["test", "-p", pkg, "--", "--test-threads=1"])
            .status()
            .expect("❌ cargo test failed to spawn");
        if !status.success() {
            eprintln!("❌ Tests failed for package '{}'", pkg);
            exit(status.code().unwrap_or(1));
        }
    }

    println!("✅ All host tests passed.");
}

// ==========================================
// 4. Platform Builders (Strategy Pattern)
// ==========================================

pub trait TargetBuilder {
    fn validate(&self, config: &Config);

    fn generate_app_build(&self, config: &Config);

    fn compile(&self, config: &Config);

    // Default implementation for running clippy
    fn clippy(&self, _config: &Config) {
        let status = Command::new("cargo")
            .arg("clippy")
            .current_dir(".app_build")
            .status()
            .expect("Failed to execute cargo clippy");
        if !status.success() {
            exit(status.code().unwrap_or(1));
        }
    }

    // Default implementation for running the project
    fn run(&self, _config: &Config) {
        let status = Command::new("cargo")
            .arg("run")
            .current_dir(".app_build")
            .status()
            .expect("Failed to execute cargo run");
        if !status.success() {
            exit(status.code().unwrap_or(1));
        }
    }

    // On-hardware test support (opt-in per board)
    fn generate_app_test_build(&self, _config: &Config) {
        eprintln!("❌ On-hardware tests are not supported for this board.");
        exit(1);
    }

    fn test_hardware(&self, _config: &Config) {
        eprintln!("❌ On-hardware tests are not supported for this board.");
        exit(1);
    }
}

// Automatically discovered and linked board builders
include!(concat!(env!("OUT_DIR"), "/board_registry.rs"));
