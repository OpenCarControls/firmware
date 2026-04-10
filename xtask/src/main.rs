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
}

#[derive(Deserialize)]
pub struct TargetConfig {
    pub board: String, // e.g., "esp" or "rpi"
    pub brand: String,
    pub platform: String,
}

#[derive(Deserialize)]
pub struct NetworkConfig {
    pub mqtt: MqttConfig,
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

// ==========================================
// 2. The Main CLI Entrypoint
// ==========================================

fn main() {
    let args: Vec<String> = env::args().collect();

    let command = args.get(1).map(|s| s.as_str()).unwrap_or("");
    match command {
        "build" | "run" | "clippy" => {}
        _ => {
            eprintln!("Usage: cargo xtask <build|run|clippy> [config_file.toml] [--board <board>] [--brand <brand>] [--platform <platform>]");
            exit(1);
        }
    }

    // Parse optional positional config path and named overrides from remaining args
    let mut config_path = "config.toml";
    let mut override_board: Option<String> = None;
    let mut override_brand: Option<String> = None;
    let mut override_platform: Option<String> = None;

    let mut remaining = args[2..].iter();
    while let Some(arg) = remaining.next() {
        match arg.as_str() {
            "--board"    => { override_board    = remaining.next().cloned(); }
            "--brand"    => { override_brand    = remaining.next().cloned(); }
            "--platform" => { override_platform = remaining.next().cloned(); }
            other if !other.starts_with("--") => { config_path = other; }
            other => {
                eprintln!("Unknown argument: {}", other);
                exit(1);
            }
        }
    }

    println!("🚀 Starting custom {} using: {}", command, config_path);

    // Read and parse the config file, then apply any CLI overrides
    let config_str = fs::read_to_string(config_path)
        .unwrap_or_else(|_| panic!("Failed to read config file: {}", config_path));
    let mut config: Config = toml::from_str(&config_str).expect("Failed to parse TOML configuration");

    if let Some(board)    = override_board    { config.target.board    = board; }
    if let Some(brand)    = override_brand    { config.target.brand    = brand; }
    if let Some(platform) = override_platform { config.target.platform = platform; }

    let builder = get_builder(&config.target.board);

    builder.validate(&config);

    builder.generate_app_build(&config);

    match command {
        "run" => builder.run(&config),
        "clippy" => builder.clippy(&config),
        "build" => builder.compile(&config),
        _ => unreachable!(),
    }
}

// ==========================================
// 3. Platform Builders (Strategy Pattern)
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
}

// Automatically discovered and linked board builders
include!(concat!(env!("OUT_DIR"), "/board_registry.rs"));
