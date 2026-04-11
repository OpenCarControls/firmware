use crate::{Config, TargetBuilder};
use serde::Deserialize;
use std::fs;
use std::process::{Command, exit};

#[derive(Deserialize)]
struct PcCanBus {
    interface: String,
}

#[derive(Deserialize)]
struct PcHardwareConfig {
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
        let pc_hw = Self::get_pc_hw(config);
        let brand = &config.target.brand;
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
        let vehicle_cargo_path = format!("cars/{}/platforms/{}/Cargo.toml", brand, vehicle_platform);
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
            "{} = {{ path = \"../cars/{}/platforms/{}\" }}\n",
            vehicle_crate_name, brand, vehicle_platform
        ));
        cargo_toml.push_str("embassy-executor = { version = \"0.9.1\", features = [\"arch-std\", \"executor-thread\"] }\n");
        cargo_toml.push_str("embassy-time = { version = \"0.5.1\", features = [\"std\"] }\n");
        cargo_toml.push_str("critical-section = { version = \"1\", features = [\"std\"] }\n");

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

        let template = fs::read_to_string("boards/pc/main.template.rs")
            .expect("\u{274c} Could not read boards/pc/main.template.rs");
        let main_rs = template
            .replace("{PLATFORM_ID}", &format!("0x{:08X}", platform_id))
            .replace("{VEHICLE_CRATE_IDENT}", &vehicle_crate_ident)
            .replace("{CAN_SPAWNS}", &can_spawns)
            .replace("{MTLS_CERTS}", &mtls_certs);

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

