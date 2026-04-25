use crate::{
    Config, TargetBuilder, escape_rust_string, load_transport_contract, parse_broker_url,
    render_topic_from_template, validate_ble_transport_contract,
};
use serde::Deserialize;
use std::fs;
use std::process::{Command, exit};

#[derive(Deserialize)]
struct PcCanBus {
    interface: String,
}

#[derive(Deserialize)]
struct PcBleConfig {
    #[serde(default)]
    device_name: Option<String>,
    #[serde(default = "default_ble_http_port")]
    http_port: u16,
    #[serde(default = "default_max_bonded_phones")]
    max_bonded_phones: u8,
    #[serde(default = "default_paired_phones_file")]
    paired_phones_file: String,
}

impl Default for PcBleConfig {
    fn default() -> Self {
        Self {
            device_name: None,
            http_port: default_ble_http_port(),
            max_bonded_phones: default_max_bonded_phones(),
            paired_phones_file: default_paired_phones_file(),
        }
    }
}

fn default_ble_http_port() -> u16 {
    4242
}

fn default_max_bonded_phones() -> u8 {
    8
}

fn default_paired_phones_file() -> String {
    "/tmp/opencar-paired-phones.txt".to_string()
}

#[derive(Deserialize)]
struct PcHardwareConfig {
    #[serde(default)]
    ble: PcBleConfig,
    #[serde(default)]
    can_buses: Vec<PcCanBus>,
}

#[derive(Deserialize)]
struct PlatformMeta {
    platform_id: String,
    can_bus_count: usize,
}

pub struct Builder;

impl Builder {
    fn get_pc_hw(config: &Config) -> PcHardwareConfig {
        config.hardware.get("pc")
            .expect("\u{274c} Missing [hardware.pc] section in config")
            .clone()
            .try_into()
            .expect("\u{274c} Invalid [hardware.pc] config format")
    }

    fn execute_cargo_command(&self, cargo_cmd: &str) {
        let mut cmd = Command::new("cargo");
        cmd.arg(cargo_cmd)
           .arg("--manifest-path").arg(".app_build/Cargo.toml")
           .arg("--release");
        let status = cmd.status().expect("Failed to execute cargo command");
        if !status.success() {
            exit(status.code().unwrap_or(1));
        }
    }
}

impl TargetBuilder for Builder {
    fn validate(&self, _config: &Config) {
        if cfg!(windows) {
            println!("⚠️ Warning: Running PC simulator on Windows. Native SocketCAN will not be available.");
        }
    }

    fn generate_app_build(&self, config: &Config) {
        let transport = load_transport_contract();
        validate_ble_transport_contract(&transport);

        let pc_hw = Self::get_pc_hw(config);
        let vehicle_platform = &config.target.platform;
        let platform_underscore = vehicle_platform.replace('-', "_");

        // Read platform meta (platform_id + can_bus_count)
        let meta_path = format!("contracts/opencar/cars/{}/v1/meta.toml", platform_underscore);
        let meta_str = fs::read_to_string(&meta_path)
            .unwrap_or_else(|_| panic!("\u{274c} Could not read platform meta: {}", meta_path));
        let meta: PlatformMeta = toml::from_str(&meta_str)
            .expect("\u{274c} Invalid platform meta.toml format");
        let hex = meta.platform_id.trim_start_matches("0x").trim_start_matches("0X");
        let platform_id: u32 = u32::from_str_radix(hex, 16)
            .expect("\u{274c} Invalid platform_id hex in meta.toml");

        // Validate CAN bus count
        if pc_hw.can_buses.len() < meta.can_bus_count {
            eprintln!(
                "\u{274c} Vehicle '{}' requires {} CAN bus(es) but [hardware.pc] only defines {} [[hardware.pc.can_buses]] entries.",
                vehicle_platform, meta.can_bus_count, pc_hw.can_buses.len()
            );
            exit(1);
        }

        // Read vehicle crate name from its Cargo.toml
        let vehicle_cargo_path = format!("cars/{}/Cargo.toml", vehicle_platform);
        let vehicle_cargo_str = fs::read_to_string(&vehicle_cargo_path)
            .unwrap_or_else(|_| panic!("\u{274c} Could not read vehicle Cargo.toml: {}", vehicle_cargo_path));
        let vehicle_cargo: toml::Value = toml::from_str(&vehicle_cargo_str)
            .expect("\u{274c} Invalid vehicle Cargo.toml");
        let vehicle_crate_name = vehicle_cargo["package"]["name"].as_str()
            .expect("\u{274c} Missing [package.name] in vehicle Cargo.toml")
            .to_string();
        let vehicle_crate_ident = vehicle_crate_name.replace('-', "_");

        // Generate one socket_can_task spawn per bus.
        // Only the first `can_bus_count` buses are used by the vehicle; extras are ignored.
        let can_spawns: String = pc_hw.can_buses.iter().take(meta.can_bus_count).enumerate()
            .map(|(bus_id, bus)| format!(
                "    spawner.spawn(board_pc::socket_can_task(\"{}\", {}, {}::CAN_FILTERS)).unwrap();\n",
                bus.interface, bus_id, vehicle_crate_ident
            ))
            .collect();

        // Build .app_build/Cargo.toml
        let mut cargo_toml = String::new();
        cargo_toml.push_str("[package]\nname = \"app-build\"\nversion = \"0.1.0\"\nedition = \"2024\"\n\n");
        cargo_toml.push_str("[dependencies]\n");
        cargo_toml.push_str("core-interface = { path = \"../core-interface\" }\n");
        cargo_toml.push_str("board-pc = { path = \"../boards/pc\" }\n");
        cargo_toml.push_str(&format!(
            "{} = {{ path = \"../cars/{}\" }}\n",
            vehicle_crate_name, vehicle_platform
        ));
        let v = |name: &str| crate::ws_dep_version(&config.workspace_deps, name);
        cargo_toml.push_str(&format!("embassy-executor = {{ version = \"{}\", features = [\"arch-std\", \"executor-thread\"] }}\n", v("embassy-executor")));
        cargo_toml.push_str(&format!("embassy-time = {{ version = \"{}\", features = [\"std\"] }}\n", v("embassy-time")));
        cargo_toml.push_str(&format!("critical-section = {{ version = \"{}\", features = [\"std\"] }}\n", v("critical-section")));
        cargo_toml.push_str("env_logger = \"0.11\"\n");

        if config.network.mqtt.auth_mode == "mtls" {
            // certs are embedded via include_bytes! in main.rs, no extra deps needed
            let _ = (&config.network.mqtt.ca_cert_file, &config.network.mqtt.client_cert_file, &config.network.mqtt.client_key_file);
        }

        // Build .app_build/src/main.rs from template
        let mtls_certs = if config.network.mqtt.auth_mode == "mtls" {
            let ca = config.network.mqtt.ca_cert_file.as_ref().unwrap();
            let cert = config.network.mqtt.client_cert_file.as_ref().unwrap();
            let key = config.network.mqtt.client_key_file.as_ref().unwrap();
            format!(
                "const CA_CERT: &[u8] = include_bytes!(\"../../{}\");\nconst CLIENT_CERT: &[u8] = include_bytes!(\"../../{}\");\nconst CLIENT_KEY: &[u8] = include_bytes!(\"../../{}\");",
                ca, cert, key
            )
        } else {
            String::new()
        };

        // Generate network constants and MQTT driver spawn
        let (broker_host, broker_port) = parse_broker_url(&config.network.mqtt.broker_url);
        let client_id = &config.network.mqtt.client_id;
        let mqtt_username = escape_rust_string(config.network.mqtt.username.as_deref().unwrap_or(""));
        let mqtt_password = escape_rust_string(config.network.mqtt.password.as_deref().unwrap_or(""));
        let mqtt_cmd_topic = escape_rust_string(&render_topic_from_template(
            &transport.mqtt.command_topic_template,
            client_id,
            "mqtt.command_topic_template",
        ));
        let mqtt_data_topic = escape_rust_string(&render_topic_from_template(
            &transport.mqtt.data_topic_template,
            client_id,
            "mqtt.data_topic_template",
        ));
        let ble_device_name = pc_hw
            .ble
            .device_name
            .as_deref()
            .unwrap_or(transport.ble.service.name.as_str());
        let ble_device_name = escape_rust_string(ble_device_name);
        let broker_host = escape_rust_string(&broker_host);
        let client_id = escape_rust_string(client_id);
        let paired_phones_file = escape_rust_string(&pc_hw.ble.paired_phones_file);

        if pc_hw.ble.max_bonded_phones == 0 || pc_hw.ble.http_port == 0 {
            eprintln!(
                "❌ [hardware.pc.ble].max_bonded_phones and http_port must be > 0"
            );
            exit(1);
        }

        let network_constants = format!(
            "const MQTT_BROKER_HOST: &str = \"{host}\";\n\
             const MQTT_BROKER_PORT: u16 = {port};\n\
             const BLE_DEVICE_NAME_BASE: &str = \"{ble_name}\";\n\
             const BLE_HTTP_PORT: u16 = {ble_http_port};\n\
             const BLE_PAIRING_WINDOW_S: u32 = {pair_window};\n\
             const BLE_MAX_BONDED_PHONES: u8 = {max_bonds};\n\
             const BLE_PAIRED_PHONES_FILE: &str = \"{store_path}\";\n\
             const MQTT_CLIENT_ID: &str = \"{cid}\";\n\
             const MQTT_CMD_TOPIC: &str = \"{cmd_topic}\";\n\
             const MQTT_DATA_TOPIC: &str = \"{data_topic}\";\n\
             const MQTT_USERNAME: &str = \"{user}\";\n\
             const MQTT_PASSWORD: &str = \"{pass}\";",
            host = broker_host,
            port = broker_port,
            ble_name = ble_device_name,
            ble_http_port = pc_hw.ble.http_port,
            pair_window = transport.ble.pairing.pairing_window_seconds,
            max_bonds = pc_hw.ble.max_bonded_phones,
            store_path = paired_phones_file,
            cid = client_id,
            cmd_topic = mqtt_cmd_topic,
            data_topic = mqtt_data_topic,
            user = mqtt_username,
            pass = mqtt_password,
        );

        let ble_http_spawn = "    spawner\n\
             .spawn(board_pc::ble_http_task(BLE_HTTP_PORT, BLE_DEVICE_NAME_BASE, BLE_PAIRING_WINDOW_S))\n\
             .unwrap();\n"
            .to_string();

        let mqtt_driver_spawn =
            "    spawner\n\
             .spawn(board_pc::mqtt_driver_task(\n\
             MQTT_BROKER_HOST,\n\
             MQTT_BROKER_PORT,\n\
             MQTT_CLIENT_ID,\n\
             MQTT_CMD_TOPIC,\n\
             MQTT_DATA_TOPIC,\n\
             MQTT_USERNAME,\n\
             MQTT_PASSWORD,\n\
             ))\n\
             .unwrap();\n".to_string();

        let template = fs::read_to_string("boards/pc/main.template.rs")
            .expect("\u{274c} Could not read boards/pc/main.template.rs");
        let main_rs = template
            .replace("{PLATFORM_ID}", &format!("0x{:08X}", platform_id))
            .replace("{VEHICLE_CRATE_IDENT}", &vehicle_crate_ident)
            .replace("{CAN_SPAWNS}", &can_spawns)
            .replace("{BLE_HTTP_SPAWN}", &ble_http_spawn)
            .replace("{MTLS_CERTS}", &mtls_certs)
            .replace("{NETWORK_CONSTANTS}", &network_constants)
            .replace("{MQTT_DRIVER_SPAWN}", &mqtt_driver_spawn);

        fs::create_dir_all(".app_build/src").expect("Failed to create .app_build/src");
        fs::write(".app_build/Cargo.toml", cargo_toml).expect("Failed to write .app_build/Cargo.toml");
        fs::write(".app_build/src/main.rs", main_rs).expect("Failed to write .app_build/src/main.rs");
    }

    fn compile(&self, _config: &Config) {
        println!("⚙️  Compiling the PC simulator natively...");
        self.execute_cargo_command("build");
    }

    fn run(&self, _config: &Config) {
        println!("🚀 Running the PC simulator natively...");
        self.execute_cargo_command("run");
    }

    fn clippy(&self, _config: &Config) {
        println!("🔍 Running clippy on the PC simulator...");
        self.execute_cargo_command("clippy");
    }
}
