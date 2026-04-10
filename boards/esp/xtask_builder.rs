use crate::{Config, TargetBuilder};
use serde::Deserialize;
use std::fs;
use std::process::{Command, exit};

#[derive(Deserialize)]
struct EspConfig {
    mcu: String,
    can_tx_pin: u8,
    can_rx_pin: u8,
    modem_tx_pin: u8,
    modem_rx_pin: u8,
}

#[derive(Deserialize)]
struct PlatformMeta {
    platform_id: String,
}

pub struct Builder;

impl Builder {
    fn get_esp_config(config: &Config) -> EspConfig {
        config.hardware.get("esp")
            .expect("❌ Missing [hardware.esp] section in config")
            .clone()
            .try_into()
            .expect("❌ Invalid [hardware.esp] config format")
    }

    fn execute_cargo_command(&self, config: &Config, cargo_cmd: &str) {
        let esp_hw = Self::get_esp_config(config);
        let mcu = &esp_hw.mcu;

        let target_arch = match mcu.as_str() {
            "esp32"  => "xtensa-esp32-none-elf",
            "esp32s2" => "xtensa-esp32s2-none-elf",
            "esp32s3" => "xtensa-esp32s3-none-elf",
            "esp32c3" => "riscv32imc-unknown-none-elf",
            "esp32c6" => "riscv32imac-unknown-none-elf",
            _ => unreachable!(),
        };

        let mut cmd = Command::new("cargo");

        if target_arch.starts_with("xtensa") {
            cmd.arg("+esp");
            cmd.arg("-Zbuild-std=core,alloc");
        }

        cmd.arg(cargo_cmd)
            .arg("--manifest-path").arg(".app_build/Cargo.toml")
            .arg("--target").arg(target_arch)
            .arg("--release");

        let features = format!("esp-hal/{mcu},esp-rtos/{mcu},esp-backtrace/{mcu},esp-println/{mcu}");
        cmd.arg("--features").arg(features);

        cmd.env("RUSTFLAGS", "-C link-arg=-Tlinkall.x");

        let status = cmd.status().expect("Failed to execute cargo command");
        if !status.success() {
            exit(status.code().unwrap_or(1));
        }
    }
}

impl TargetBuilder for Builder {
    fn validate(&self, config: &Config) {
        let esp = Self::get_esp_config(config);
        if esp.mcu == "esp32s2" {
            eprintln!("❌ Error: The ESP32-S2 chip does not have Bluetooth/BLE capabilities. This project strictly requires BLE for phone app configuration and control.");
            exit(1);
        }
        let supported_mcus = ["esp32", "esp32s3", "esp32c3", "esp32c6"];
        if !supported_mcus.contains(&esp.mcu.as_str()) {
            eprintln!("❌ Error: Unsupported ESP MCU '{}'. Supported MCUs are: {:?}", esp.mcu, supported_mcus);
            exit(1);
        }
    }

    fn generate_app_build(&self, config: &Config) {
        let esp_hw = Self::get_esp_config(config);
        let brand = &config.target.brand;
        let vehicle_platform = &config.target.platform;
        let platform_underscore = vehicle_platform.replace('-', "_");

        // Read platform_id from contracts meta.toml
        let meta_path = format!("contracts/opencar/cars/{}/v1/meta.toml", platform_underscore);
        let meta_str = fs::read_to_string(&meta_path)
            .unwrap_or_else(|_| panic!("\u{274c} Could not read platform meta: {}", meta_path));
        let meta: PlatformMeta = toml::from_str(&meta_str)
            .expect("\u{274c} Invalid platform meta.toml format");
        let hex = meta.platform_id.trim_start_matches("0x").trim_start_matches("0X");
        let platform_id: u32 = u32::from_str_radix(hex, 16)
            .expect("\u{274c} Invalid platform_id hex in meta.toml");

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

        // Build .app_build/Cargo.toml
        let mut cargo_toml = String::new();
        cargo_toml.push_str("[package]\nname = \"app-build\"\nversion = \"0.1.0\"\nedition = \"2024\"\n\n");
        cargo_toml.push_str("[dependencies]\n");
        cargo_toml.push_str("core-interface = { path = \"../core-interface\" }\n");
        cargo_toml.push_str("board-esp = { path = \"../boards/esp\" }\n");
        cargo_toml.push_str(&format!(
            "{} = {{ path = \"../cars/{}/platforms/{}\" }}\n",
            vehicle_crate_name, brand, vehicle_platform
        ));
        cargo_toml.push_str("esp-hal = { version = \"1.0.0\", features = [\"unstable\"] }\n");
        cargo_toml.push_str("esp-rtos = { version = \"0.2.0\", features = [\"embassy\"] }\n");
        cargo_toml.push_str("esp-backtrace = { version = \"0.18.1\", features = [\"panic-handler\", \"println\"] }\n");
        cargo_toml.push_str("esp-println = { version = \"0.16.1\", features = [\"log-04\"] }\n");
        cargo_toml.push_str("esp-alloc = \"0.7.0\"\n");
        cargo_toml.push_str("embassy-executor = \"0.9.1\"\n");
        cargo_toml.push_str("embassy-time = \"0.5.1\"\n");

        if config.network.mqtt.auth_mode == "mtls" {
            let ca = config.network.mqtt.ca_cert_file.as_ref().unwrap();
            let cert = config.network.mqtt.client_cert_file.as_ref().unwrap();
            let key = config.network.mqtt.client_key_file.as_ref().unwrap();
            let _ = (ca, cert, key); // embedded via include_bytes! in main.rs
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

        let template = fs::read_to_string("boards/esp/main.template.rs")
            .expect("\u{274c} Could not read boards/esp/main.template.rs");
        let main_rs = template
            .replace("{PLATFORM_ID}", &format!("0x{:08X}", platform_id))
            .replace("{CAN_TX_PIN}", &esp_hw.can_tx_pin.to_string())
            .replace("{CAN_RX_PIN}", &esp_hw.can_rx_pin.to_string())
            .replace("{VEHICLE_CRATE_IDENT}", &vehicle_crate_ident)
            .replace("{MTLS_CERTS}", &mtls_certs);

        fs::create_dir_all(".app_build/src").expect("Failed to create .app_build/src");
        fs::write(".app_build/Cargo.toml", cargo_toml).expect("Failed to write .app_build/Cargo.toml");
        fs::write(".app_build/src/main.rs", main_rs).expect("Failed to write .app_build/src/main.rs");
    }

    fn compile(&self, config: &Config) {
        println!("⚙️ Compiling the bare-metal firmware...");
        self.execute_cargo_command(config, "build");
    }

    fn run(&self, config: &Config) {
        println!("🚀 Running the firmware build pipeline...");
        self.execute_cargo_command(config, "run");
    }

    fn clippy(&self, config: &Config) {
        println!("🔍 Running clippy on the firmware build pipeline...");
        self.execute_cargo_command(config, "clippy");
    }
}

