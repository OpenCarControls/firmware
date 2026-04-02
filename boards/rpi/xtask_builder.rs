use crate::{Config, TargetBuilder};
use serde::Deserialize;
use std::process::{Command, exit};

#[derive(Deserialize)]
struct RpiConfig {
    can_interface: String,
}

pub struct Builder;

impl Builder {
    // Helper method to dynamically extract and parse the RPi hardware config
    fn get_rpi_config(config: &Config) -> RpiConfig {
        config.hardware.get("rpi")
            .expect("❌ Missing [hardware.rpi] section in config")
            .clone()
            .try_into() // Converts the raw toml::Value into our RpiConfig struct
            .expect("❌ Invalid [hardware.rpi] config format")
    }
}

impl TargetBuilder for Builder {
    fn validate(&self, _config: &Config) {}
    fn extend_cargo_toml(&self, _config: &Config, _toml: &mut String) {}
    fn generate_main_rs(&self, config: &Config, main_rs: &mut String) {
        let rpi_hw = Self::get_rpi_config(config);
        let platform = config.target.platform.replace("-", "_");

        let template = include_str!("main.template.rs");
        let generated = template
            .replace("{CAN_INTERFACE}", &rpi_hw.can_interface.to_string())
            .replace("{PLATFORM}", &platform);

        main_rs.push_str(&generated);
    }

    fn compile(&self, _config: &Config) {
        println!("⚙️  Compiling the final firmware in .app_build/...");
        let mut cmd = Command::new("cargo");
        cmd.current_dir(".app_build").arg("build").arg("--release");

        let status = cmd.status().expect("Failed to execute cargo build");
        if status.success() {
            println!("✅ Firmware built successfully!");
        } else {
            eprintln!("❌ Firmware compilation failed.");
            exit(1);
        }
    }
}
