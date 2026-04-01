use crate::{Config, TargetBuilder};
use std::process::{Command, exit};

pub struct Builder;

impl TargetBuilder for Builder {
    fn validate(&self, _config: &Config) {}
    fn extend_cargo_toml(&self, _config: &Config, _toml: &mut String) {}
    fn generate_main_rs(&self, _config: &Config, main_rs: &mut String) {
        main_rs.push_str(
            r#"fn main() {
    println!("⚠️  Running in RPi Development Mode");
    println!("ℹ️  Note: BLE hardware is required for full remote control functionality.");
    // RPi init logic here...
}
"#,
        );
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
