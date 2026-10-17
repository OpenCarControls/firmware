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
    /// Populated at runtime from `[patch]` in the root `Cargo.toml`.
    #[serde(skip, default = "default_empty_toml_table")]
    pub workspace_patch: toml::Value,
}

fn default_empty_toml_table() -> toml::Value {
    toml::Value::Table(Default::default())
}

#[derive(Deserialize)]
pub struct TargetConfig {
    #[serde(default)]
    pub board: Option<String>, // e.g., "esp" or "pc"
    pub platform: String,
    pub hardware_profile: Option<String>,
}

fn merge_toml(base: &mut toml::Value, override_val: &toml::Value) {
    match (base, override_val) {
        (toml::Value::Table(base_map), toml::Value::Table(override_map)) => {
            for (k, v) in override_map {
                if base_map.contains_key(k) {
                    merge_toml(base_map.get_mut(k).unwrap(), v);
                } else {
                    base_map.insert(k.clone(), v.clone());
                }
            }
        }
        (b, o) => {
            *b = o.clone();
        }
    }
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

/// The root `Cargo.toml`'s `[patch]` table, rendered for a generated crate's
/// Cargo.toml so it resolves the same patched crates as the workspace.
pub fn workspace_patch_section(config: &Config) -> String {
    let mut root = toml::Table::new();
    root.insert("patch".into(), config.workspace_patch.clone());
    format!("\n{}", toml::to_string(&root).expect("Failed to serialize [patch] table"))
}

/// Returns the version string for a `[workspace.dependencies]` entry.
/// Handles both `dep = "1.2.3"` (plain string) and `dep = { version = "1.2.3", ... }` (table).
pub fn ws_dep_version(workspace_deps: &toml::Value, name: &str) -> String {
    let dep = workspace_deps.get(name).unwrap_or_else(|| {
        panic!(
            "Missing workspace dependency '{}'. Add it to [workspace.dependencies] in Cargo.toml.",
            name
        )
    });
    if let Some(s) = dep.as_str() {
        s.to_string()
    } else if let Some(v) = dep.get("version").and_then(|v| v.as_str()) {
        v.to_string()
    } else {
        panic!("Workspace dependency '{}' has no 'version' field.", name)
    }
}

/// Reads `contracts/opencar/cars/<platform>/v1/meta.toml` and returns `(platform_id, can_bus_count)`.
/// `platform` is the hyphenated name from `config.toml` (e.g. `"virtual-car"`).
pub fn load_platform_meta(platform: &str) -> (u32, usize) {
    #[derive(Deserialize)]
    struct PlatformMeta {
        platform_id: String,
        can_bus_count: usize,
    }
    let platform_underscore = platform.replace('-', "_");
    let meta_path = format!(
        "contracts/opencar/cars/{}/v1/meta.toml",
        platform_underscore
    );
    let meta_str = fs::read_to_string(&meta_path)
        .unwrap_or_else(|_| panic!("Could not read platform meta: {}", meta_path));
    let meta: PlatformMeta = toml::from_str(&meta_str).expect("Invalid platform meta.toml format");
    let hex = meta
        .platform_id
        .trim_start_matches("0x")
        .trim_start_matches("0X");
    let platform_id = u32::from_str_radix(hex, 16).expect("Invalid platform_id hex in meta.toml");
    (platform_id, meta.can_bus_count)
}

/// Reads `cars/<platform>/Cargo.toml` and returns `(crate_name, crate_ident)`.
/// `crate_ident` has hyphens replaced by underscores so it is valid in generated Rust code.
pub fn load_vehicle_crate_info(platform: &str) -> (String, String) {
    let path = format!("cars/{}/Cargo.toml", platform);
    let cargo_str = fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("Could not read vehicle Cargo.toml: {}", path));
    let cargo: toml::Value = toml::from_str(&cargo_str).expect("Invalid vehicle Cargo.toml");
    let name = cargo["package"]["name"]
        .as_str()
        .expect("Missing [package.name] in vehicle Cargo.toml")
        .to_string();
    let ident = name.replace('-', "_");
    (name, ident)
}

/// Generates `include_bytes!` constants for mTLS certs when `auth_mode = "mtls"`,
/// or an empty string for other auth modes.
/// The paths are relative to the generated `.app_build/src/main.rs` (hence `../../`).
pub fn generate_mtls_certs(config: &Config) -> String {
    if config.network.mqtt.auth_mode != "mtls" {
        return String::new();
    }
    let ca = config
        .network
        .mqtt
        .ca_cert_file
        .as_ref()
        .expect("[network.mqtt].ca_cert_file is required when auth_mode = \"mtls\"");
    let cert = config
        .network
        .mqtt
        .client_cert_file
        .as_ref()
        .expect("[network.mqtt].client_cert_file is required when auth_mode = \"mtls\"");
    let key = config
        .network
        .mqtt
        .client_key_file
        .as_ref()
        .expect("[network.mqtt].client_key_file is required when auth_mode = \"mtls\"");
    format!(
        "const CA_CERT: &[u8] = include_bytes!(\"../../{}\");\n\
         const CLIENT_CERT: &[u8] = include_bytes!(\"../../{}\");\n\
         const CLIENT_KEY: &[u8] = include_bytes!(\"../../{}\");",
        ca, cert, key
    )
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
            "broker_url must start with mqtt:// or mqtts://, got: {}",
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
            eprintln!("Invalid port in broker_url: {}", port_str);
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
        .unwrap_or_else(|_| panic!("Failed to read transport contract: {}", path));
    toml::from_str(&content).expect("Failed to parse transport.toml")
}

pub fn render_topic_from_template(template: &str, client_id: &str, field_name: &str) -> String {
    if !template.contains("{client_id}") {
        eprintln!(
            "transport.toml {} must include '{{client_id}}': {}",
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
            "transport.toml {} must be a 128-bit UUID, got: {}",
            field_name, uuid
        );
        exit(1);
    }
    u128::from_str_radix(&hex, 16).unwrap_or_else(|_| {
        eprintln!(
            "transport.toml {} is not valid hex UUID: {}",
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
            "transport.toml ble.characteristics.app_to_device.payload must be opencar.core.v1.AppToDevice"
        );
        exit(1);
    }
    if app_to_device.direction != "app -> device" {
        eprintln!(
            "transport.toml ble.characteristics.app_to_device.direction must be 'app -> device'"
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
            "transport.toml ble.characteristics.app_to_device.properties must be [\"write\", \"write_without_response\"]"
        );
        exit(1);
    }

    if device_to_app.payload != "opencar.core.v1.DeviceToApp" {
        eprintln!(
            "transport.toml ble.characteristics.device_to_app.payload must be opencar.core.v1.DeviceToApp"
        );
        exit(1);
    }
    if device_to_app.direction != "device -> app" {
        eprintln!(
            "transport.toml ble.characteristics.device_to_app.direction must be 'device -> app'"
        );
        exit(1);
    }
    if device_to_app.properties.len() != 1 || device_to_app.properties[0] != "notify" {
        eprintln!(
            "transport.toml ble.characteristics.device_to_app.properties must be [\"notify\"]"
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
        eprintln!("transport.toml ble.pairing.pairing_window_seconds must be > 0");
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
        "build" | "run" | "clippy" | "test" | "flash" | "ide" => {}
        _ => {
            eprintln!(
                "Usage: cargo xtask <build|run|clippy|test|flash|ide> [config_file.toml] [--board <board>] [--platform <platform>] [--on-hardware] [--debug] [--port <port>] [--monitor]"
            );
            exit(1);
        }
    }

    // Parse optional positional config path and named overrides from remaining args
    let mut config_path = "config.toml";
    let mut override_board: Option<String> = None;
    let mut override_platform: Option<String> = None;
    let mut on_hardware = false;
    let mut release = true;
    let mut flash_port: Option<String> = None;
    let mut flash_monitor = false;

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
            "--debug" => {
                release = false;
            }
            "--port" => {
                flash_port = remaining.next().cloned();
            }
            "--monitor" => {
                flash_monitor = true;
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
    if config_path == "config.toml"
        && !std::path::Path::new("config.toml").exists()
        && std::path::Path::new("config.toml.example").exists()
    {
        fs::copy("config.toml.example", "config.toml")
            .expect("Failed to copy config.toml.example → config.toml");
        println!("📋 Created config.toml from config.toml.example — edit it to match your setup.");
    }

    println!("Starting custom {} using: {}", command, config_path);

    // Read and parse the config file, then apply any CLI overrides
    let config_str = fs::read_to_string(config_path)
        .unwrap_or_else(|_| panic!("Failed to read config file: {}", config_path));
    let mut config: Config =
        toml::from_str(&config_str).expect("Failed to parse TOML configuration");

    // Load workspace dependency versions from root Cargo.toml so builders don't hardcode them.
    let ws_toml_str =
        fs::read_to_string("Cargo.toml").expect("Could not read workspace Cargo.toml");
    let ws_toml: toml::Value =
        toml::from_str(&ws_toml_str).expect("Failed to parse workspace Cargo.toml");
    config.workspace_deps = ws_toml["workspace"]["dependencies"].clone();
    config.workspace_patch = ws_toml.get("patch").cloned().unwrap_or_else(default_empty_toml_table);

    if let Some(profile) = &config.target.hardware_profile {
        let profile_path = format!("hardware-profiles/{}.toml", profile);
        let profile_str = fs::read_to_string(&profile_path)
            .unwrap_or_else(|_| panic!("Failed to read hardware profile: {}", profile_path));
        let profile_toml: toml::Value =
            toml::from_str(&profile_str).expect("Failed to parse hardware profile TOML");
        if let Some(hw) = profile_toml.get("hardware") {
            let mut base_hw = hw.clone();
            merge_toml(&mut base_hw, &config.hardware);
            config.hardware = base_hw;
        }
        if let Some(tgt) = profile_toml.get("target") {
            if let Some(board) = tgt.get("board").and_then(|v| v.as_str()) {
                config.target.board = Some(board.to_string());
            }
        }
    }

    if let Some(board) = override_board {
        config.target.board = Some(board);
    }
    if let Some(platform) = override_platform {
        config.target.platform = platform;
    }

    // Host tests: no board-specific build needed
    if command == "test" && !on_hardware {
        run_host_tests(&config);
        return;
    }

    let board = config.target.board.as_deref().unwrap_or_else(|| {
        eprintln!("Error: 'board' must be specified in the [target] section of config.toml or inherited from a hardware profile.");
        exit(1);
    });
    let builder = get_builder(board);

    builder.validate(&config);

    if command == "ide" {
        write_vscode_config(
            board,
            &builder.rust_analyzer_config(&config),
            builder.debug_launch_config(&config),
        );
        return;
    }

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
                "build" => builder.compile(&config, release),
                "flash" => {
                    builder.compile(&config, release);
                    builder.flash(&config, flash_port.as_deref(), flash_monitor, release);
                }
                _ => unreachable!(),
            }
        }
    }
}

/// rust-analyzer settings owned by `cargo xtask ide`: removed and rewritten on
/// every run. Anything else in the settings file is left alone.
const RUST_ANALYZER_KEYS: [&str; 6] = [
    "rust-analyzer.cargo.features",
    "rust-analyzer.cargo.target",
    "rust-analyzer.cargo.extraEnv",
    "rust-analyzer.cargo.extraArgs",
    "rust-analyzer.check.extraArgs",
    "rust-analyzer.check.allTargets",
];

/// Launch configurations whose name starts with this are owned by
/// `cargo xtask ide` and replaced on every run; others are left alone.
pub const LAUNCH_NAME_PREFIX: &str = "Firmware: ";

/// Points VS Code at the selected board by merging into its config files:
/// rust-analyzer settings (features, target, toolchain) into settings.json,
/// and the board's debug configuration into launch.json. rust-analyzer only
/// reads these from editor settings, not from `rust-analyzer.toml`.
///
/// Writes to `.vscode/` (both files are gitignored), or to the directory in
/// `OPENCAR_VSCODE_DIR` when the editor's workspace root is elsewhere (e.g. the
/// multi-repo workspace container). Keys and launch configs not owned by xtask
/// are kept.
fn write_vscode_config(board: &str, ra: &RustAnalyzerConfig, launch: serde_json::Value) {
    use serde_json::{Value, json};

    let dir = env::var("OPENCAR_VSCODE_DIR").unwrap_or_else(|_| ".vscode".into());
    fs::create_dir_all(&dir).unwrap_or_else(|_| panic!("Failed to create {dir}"));

    let settings_path = format!("{dir}/settings.json");
    let mut settings = read_vscode_json(&settings_path);
    for key in RUST_ANALYZER_KEYS {
        settings.remove(key);
    }
    settings.insert("rust-analyzer.cargo.features".into(), json!(ra.features));
    if let Some(target) = &ra.target {
        settings.insert("rust-analyzer.cargo.target".into(), json!(target));
    }
    if !ra.extra_env.is_empty() {
        let env: serde_json::Map<String, Value> =
            ra.extra_env.iter().map(|(k, v)| (k.clone(), json!(v))).collect();
        settings.insert("rust-analyzer.cargo.extraEnv".into(), Value::Object(env));
    }
    if !ra.extra_args.is_empty() {
        settings.insert("rust-analyzer.cargo.extraArgs".into(), json!(ra.extra_args));
    }
    if !ra.check_exclude.is_empty() {
        let args: Vec<&str> = ra.check_exclude.iter().flat_map(|p| ["--exclude", p.as_str()]).collect();
        settings.insert("rust-analyzer.check.extraArgs".into(), json!(args));
    }
    if ra.check_skip_tests {
        settings.insert("rust-analyzer.check.allTargets".into(), json!(false));
    }
    write_vscode_json(&settings_path, settings);

    let launch_path = format!("{dir}/launch.json");
    let mut launch_json = read_vscode_json(&launch_path);
    let mut configurations: Vec<Value> = match launch_json.remove("configurations") {
        Some(Value::Array(list)) => list,
        _ => Vec::new(),
    };
    configurations.retain(|c| {
        !c.get("name").and_then(Value::as_str).is_some_and(|n| n.starts_with(LAUNCH_NAME_PREFIX))
    });
    let launch_name = launch["name"].as_str().unwrap_or_default().to_string();
    configurations.insert(0, launch);
    launch_json.insert("version".into(), json!("0.2.0"));
    launch_json.insert("configurations".into(), Value::Array(configurations));
    write_vscode_json(&launch_path, launch_json);

    let target = ra.target.as_deref().map(|t| format!(", {t}")).unwrap_or_default();
    println!("✅ VS Code now targets the '{board}' board{target}: rust-analyzer settings and the \"{launch_name}\" debug config ({dir}/).");
}

fn read_vscode_json(path: &str) -> serde_json::Map<String, serde_json::Value> {
    match fs::read_to_string(path) {
        Ok(s) if !s.trim().is_empty() => match serde_json::from_str(&s) {
            Ok(serde_json::Value::Object(map)) => map,
            _ => {
                eprintln!("Error: {path} isn't plain JSON (comments aren't supported); fix or delete it and re-run.");
                exit(1);
            }
        },
        _ => serde_json::Map::new(),
    }
}

fn write_vscode_json(path: &str, map: serde_json::Map<String, serde_json::Value>) {
    let contents = serde_json::to_string_pretty(&serde_json::Value::Object(map)).unwrap() + "\n";
    fs::write(path, contents).unwrap_or_else(|_| panic!("Failed to write {path}"));
}

/// Absolute path of the firmware workspace, for generated debug configs: VS
/// Code's `${workspaceFolder}` isn't this directory in the multi-repo workspace.
pub fn firmware_dir() -> String {
    env::current_dir()
        .expect("Failed to read the current directory")
        .to_string_lossy()
        .into_owned()
}

// ==========================================
// 3. Host Test Runner
// ==========================================

fn run_host_tests(config: &Config) {
    // Discover the configured vehicle crate name
    let (vehicle_crate, _) = load_vehicle_crate_info(&config.target.platform);

    let packages = [
        "core-interface",
        "board-pc",
        "board-esp",
        vehicle_crate.as_str(),
    ];

    for pkg in packages {
        println!("Testing {}...", pkg);
        let status = Command::new("cargo")
            .args(["test", "-p", pkg])
            .args(["--", "--test-threads=1"])
            .status()
            .expect("cargo test failed to spawn");
        if !status.success() {
            eprintln!("Tests failed for package '{}'", pkg);
            exit(status.code().unwrap_or(1));
        }
    }

    println!("All host tests passed.");
}

// ==========================================
// 4. Platform Builders (Strategy Pattern)
// ==========================================

/// How rust-analyzer must load the workspace to see the code the way this
/// board builds it. Written to VS Code's settings.json by `cargo xtask ide`.
#[derive(Default)]
pub struct RustAnalyzerConfig {
    pub features: Vec<String>,
    pub target: Option<String>,
    pub extra_env: Vec<(String, String)>,
    pub extra_args: Vec<String>,
    /// Workspace packages to leave out of `cargo check`, e.g. std-only crates
    /// when checking for a no_std target.
    pub check_exclude: Vec<String>,
    /// Check only libs/bins, not tests: host-only dev-dependencies don't
    /// compile for no_std targets.
    pub check_skip_tests: bool,
}

pub trait TargetBuilder {
    fn validate(&self, config: &Config);

    fn generate_app_build(&self, config: &Config);

    fn rust_analyzer_config(&self, config: &Config) -> RustAnalyzerConfig;

    /// VS Code launch configuration that builds and debugs this board's firmware.
    /// Its name must start with `LAUNCH_NAME_PREFIX`.
    fn debug_launch_config(&self, config: &Config) -> serde_json::Value;

    fn compile(&self, config: &Config, release: bool);

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

    // Flash the compiled firmware to a connected device (opt-in per board)
    fn flash(&self, _config: &Config, _port: Option<&str>, _monitor: bool, _release: bool) {
        eprintln!("Flash is not supported for this board.");
        exit(1);
    }

    // On-hardware test support (opt-in per board)
    fn generate_app_test_build(&self, _config: &Config) {
        eprintln!("On-hardware tests are not supported for this board.");
        exit(1);
    }

    fn test_hardware(&self, _config: &Config) {
        eprintln!("On-hardware tests are not supported for this board.");
        exit(1);
    }
}

// Automatically discovered and linked board builders
include!(concat!(env!("OUT_DIR"), "/board_registry.rs"));
