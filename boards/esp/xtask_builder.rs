use crate::{Config, TargetBuilder};
use serde::Deserialize;
use std::fs;
use std::process::{Command, exit};

#[derive(Deserialize)]
struct EspConfig {
    mcu: String,
    #[allow(dead_code)]
    modem_tx_pin: u8,
    #[allow(dead_code)]
    modem_rx_pin: u8,
    #[serde(default)]
    can_buses: Vec<toml::Value>,
}

#[derive(Deserialize)]
struct PlatformMeta {
    platform_id: String,
    can_bus_count: usize,
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
        if esp_hw.can_buses.len() < meta.can_bus_count {
            eprintln!(
                "\u{274c} Vehicle '{}' requires {} CAN bus(es) but [hardware.esp] only defines {} [[hardware.esp.can_buses]] entries.",
                vehicle_platform, meta.can_bus_count, esp_hw.can_buses.len()
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

        // Generate per-bus CAN hardware init expressions and task definitions.
        // Only the first `can_bus_count` buses are wired up; extras in config are ignored.
        let mut can_hardware_init = String::new();
        let mut can_task_defs = String::new();
        let mut can_task_spawns = String::new();
        let mut mcp_spi_idx: u8 = 2; // SPI0/SPI1 reserved for flash; MCP2515 buses start at SPI2
        let filters_expr = format!("{}::CAN_FILTERS", vehicle_crate_ident);

        for (bus_id, bus) in esp_hw.can_buses.iter().take(meta.can_bus_count).enumerate() {
            let interface = bus.get("interface")
                .and_then(|v| v.as_str())
                .unwrap_or_else(|| { eprintln!("\u{274c} can_buses entry {} is missing 'interface'", bus_id); exit(1); });

            match interface {
                "twai" => {
                    let tx = bus.get("tx_pin").and_then(|v| v.as_integer())
                        .unwrap_or_else(|| { eprintln!("\u{274c} TWAI bus {} missing 'tx_pin'", bus_id); exit(1); });
                    let rx = bus.get("rx_pin").and_then(|v| v.as_integer())
                        .unwrap_or_else(|| { eprintln!("\u{274c} TWAI bus {} missing 'rx_pin'", bus_id); exit(1); });
                    can_hardware_init.push_str(&format!(
                        "    let can_bus_{0} = board_esp::init_twai(peripherals.TWAI0, peripherals.GPIO{1}, peripherals.GPIO{2}, {3});\n",
                        bus_id, tx, rx, filters_expr
                    ));
                    can_task_defs.push_str(&format!(
                        "    #[embassy_executor::task]\n    async fn can_bus_{0}_task(driver: board_esp::TwaiDriver) {{\n        board_esp::run_twai_loop(driver, {0}, {1}).await;\n    }}\n",
                        bus_id, filters_expr
                    ));
                    can_task_spawns.push_str(&format!(
                        "            s.spawn(can_bus_{0}_task(can_bus_{0})).unwrap();\n",
                        bus_id
                    ));
                }
                "mcp2515" => {
                    let cs   = bus.get("cs_pin")  .and_then(|v| v.as_integer()).unwrap_or_else(|| { eprintln!("\u{274c} MCP2515 bus {} missing 'cs_pin'",   bus_id); exit(1); });
                    let clk  = bus.get("clk_pin") .and_then(|v| v.as_integer()).unwrap_or_else(|| { eprintln!("\u{274c} MCP2515 bus {} missing 'clk_pin'",  bus_id); exit(1); });
                    let mosi = bus.get("mosi_pin").and_then(|v| v.as_integer()).unwrap_or_else(|| { eprintln!("\u{274c} MCP2515 bus {} missing 'mosi_pin'", bus_id); exit(1); });
                    let miso = bus.get("miso_pin").and_then(|v| v.as_integer()).unwrap_or_else(|| { eprintln!("\u{274c} MCP2515 bus {} missing 'miso_pin'", bus_id); exit(1); });
                    let int  = bus.get("int_pin") .and_then(|v| v.as_integer()).unwrap_or_else(|| { eprintln!("\u{274c} MCP2515 bus {} missing 'int_pin'",  bus_id); exit(1); });
                    let can_speed_str = bus.get("can_speed").and_then(|v| v.as_str()).unwrap_or("500kbps");
                    let mcp_speed_str = bus.get("mcp_speed").and_then(|v| v.as_str()).unwrap_or("16mhz");
                    let can_speed_expr = match can_speed_str {
                        "100kbps"  => "board_esp::CanSpeed::Kbps100",
                        "125kbps"  => "board_esp::CanSpeed::Kbps125",
                        "250kbps"  => "board_esp::CanSpeed::Kbps250",
                        "500kbps"  => "board_esp::CanSpeed::Kbps500",
                        "1000kbps" => "board_esp::CanSpeed::Kbps1000",
                        other => { eprintln!("\u{274c} Unknown can_speed '{}' for bus {}. Use: 100kbps, 125kbps, 250kbps, 500kbps, 1000kbps.", other, bus_id); exit(1); }
                    };
                    let mcp_speed_expr = match mcp_speed_str {
                        "8mhz"  => "board_esp::McpSpeed::MHz8",
                        "16mhz" => "board_esp::McpSpeed::MHz16",
                        other => { eprintln!("\u{274c} Unknown mcp_speed '{}' for bus {}. Use: 8mhz, 16mhz.", other, bus_id); exit(1); }
                    };
                    let spi  = format!("SPI{}", mcp_spi_idx);
                    mcp_spi_idx += 1;
                    can_hardware_init.push_str(&format!(
                        "    let can_bus_{0}_cs = esp_hal::gpio::Output::new(peripherals.GPIO{2}, esp_hal::gpio::Level::High, esp_hal::gpio::OutputConfig::default());\n\
                             let can_bus_{0}_int = esp_hal::gpio::Input::new(peripherals.GPIO{7}, esp_hal::gpio::InputConfig::default().with_pull(esp_hal::gpio::Pull::Up));\n\
                         let (can_bus_{0}, can_bus_{0}_int) = board_esp::init_mcp2515(\n\
                             peripherals.{1},\n\
                             peripherals.GPIO{3},\n\
                             peripherals.GPIO{4},\n\
                             peripherals.GPIO{5},\n\
                             can_bus_{0}_cs,\n\
                             {6},\n\
                             {0},\n\
                             {8},\n\
                             {9},\n\
                             can_bus_{0}_int,\n\
                         );\n",
                        bus_id, spi, cs, clk, mosi, miso, filters_expr, int, can_speed_expr, mcp_speed_expr
                    ));
                    can_task_defs.push_str(&format!(
                        "    #[embassy_executor::task]\n    async fn can_bus_{0}_task(driver: board_esp::Mcp2515Driver, int_pin: board_esp::CanIntPin) {{\n        board_esp::run_mcp2515_loop(driver, int_pin, {0}, {1}).await;\n    }}\n",
                        bus_id, filters_expr
                    ));
                    can_task_spawns.push_str(&format!(
                        "            s.spawn(can_bus_{0}_task(can_bus_{0}, can_bus_{0}_int)).unwrap();\n",
                        bus_id
                    ));
                }
                other => {
                    eprintln!("\u{274c} Unknown CAN interface '{}' for bus {}. Supported: 'twai', 'mcp2515'.", other, bus_id);
                    exit(1);
                }
            }
        }

        // Build .app_build/Cargo.toml
        let mut cargo_toml = String::new();
        cargo_toml.push_str("[package]\nname = \"app-build\"\nversion = \"0.1.0\"\nedition = \"2024\"\n\n");
        cargo_toml.push_str("[dependencies]\n");
        cargo_toml.push_str("core-interface = { path = \"../core-interface\" }\n");
        cargo_toml.push_str("board-esp = { path = \"../boards/esp\", features = [\"hardware\"] }\n");
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
        cargo_toml.push_str("static_cell = \"2\"\n");

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

        let template = fs::read_to_string("boards/esp/main.template.rs")
            .expect("\u{274c} Could not read boards/esp/main.template.rs");
        let main_rs = template
            .replace("{PLATFORM_ID}", &format!("0x{:08X}", platform_id))
            .replace("{VEHICLE_CRATE_IDENT}", &vehicle_crate_ident)
            .replace("{CAN_HARDWARE_INIT}", &can_hardware_init)
            .replace("{CORE1_CAN_TASK_DEFS}", &can_task_defs)
            .replace("{CORE1_TASK_SPAWNS}", &can_task_spawns)
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

    fn generate_app_test_build(&self, config: &Config) {
        let esp_hw = Self::get_esp_config(config);
        let mcu = &esp_hw.mcu;

        // Read platform meta for PLATFORM_ID
        let platform_underscore = config.target.platform.replace('-', "_");
        let meta_path = format!("contracts/opencar/cars/{}/v1/meta.toml", platform_underscore);
        let meta_str = fs::read_to_string(&meta_path)
            .unwrap_or_else(|_| panic!("\u{274c} Could not read platform meta: {}", meta_path));
        let meta: PlatformMeta = toml::from_str(&meta_str)
            .expect("\u{274c} Invalid platform meta.toml format");
        let hex = meta.platform_id.trim_start_matches("0x").trim_start_matches("0X");
        let platform_id: u32 = u32::from_str_radix(hex, 16)
            .expect("\u{274c} Invalid platform_id hex in meta.toml");

        // Read any user-defined on-hardware tests from boards/esp/tests/hardware.rs
        let extra_tests = fs::read_to_string("boards/esp/tests/hardware.rs").unwrap_or_default();

        // Build .app_test_build/Cargo.toml
        let mut cargo_toml = String::new();
        cargo_toml.push_str("[package]\nname = \"app-test-build\"\nversion = \"0.1.0\"\nedition = \"2024\"\n\n");
        cargo_toml.push_str("[dependencies]\n");
        cargo_toml.push_str("core-interface = { path = \"../core-interface\" }\n");
        cargo_toml.push_str("board-esp = { path = \"../boards/esp\", features = [\"hardware\"] }\n");
        cargo_toml.push_str("esp-hal = { version = \"1.0.0\", features = [\"unstable\"] }\n");
        cargo_toml.push_str("esp-rtos = { version = \"0.2.0\", features = [\"embassy\"] }\n");
        cargo_toml.push_str("esp-alloc = \"0.7.0\"\n");
        cargo_toml.push_str("esp-println = { version = \"0.16.1\", features = [\"log-04\"] }\n");
        cargo_toml.push_str("embassy-executor = \"0.9.1\"\n");
        cargo_toml.push_str("embassy-time = \"0.5.1\"\n");
        cargo_toml.push_str("static_cell = \"2\"\n");
        // embedded-test with defmt RTT for reporting results over probe-rs
        cargo_toml.push_str("embedded-test = { version = \"0.5\", features = [\"embassy\"] }\n");
        cargo_toml.push_str("defmt = \"0.3\"\n");
        cargo_toml.push_str("defmt-rtt = \"0.4\"\n");
        cargo_toml.push_str("panic-probe = { version = \"0.3\", features = [\"print-defmt\"] }\n");

        // Chip-specific feature string for esp-hal/esp-rtos
        let features = format!("esp-hal/{mcu},esp-rtos/{mcu},esp-println/{mcu}");
        cargo_toml.push_str(&format!("\n[features]\ndefault = [\"{}\"]", features));

        // Generate main.rs from template
        let template = fs::read_to_string("boards/esp/tests.template.rs")
            .expect("\u{274c} Could not read boards/esp/tests.template.rs");
        let main_rs = template
            .replace("{PLATFORM_ID}", &format!("0x{:08X}", platform_id))
            .replace("{ON_HARDWARE_TESTS}", &extra_tests);

        fs::create_dir_all(".app_test_build/src").expect("Failed to create .app_test_build/src");
        fs::write(".app_test_build/Cargo.toml", cargo_toml)
            .expect("Failed to write .app_test_build/Cargo.toml");
        fs::write(".app_test_build/src/main.rs", main_rs)
            .expect("Failed to write .app_test_build/src/main.rs");
        println!("✅ .app_test_build/ generated for {} ({})", mcu, config.target.platform);
    }

    fn test_hardware(&self, config: &Config) {
        let esp_hw = Self::get_esp_config(config);
        let mcu = &esp_hw.mcu;

        let target_arch = match mcu.as_str() {
            "esp32"   => "xtensa-esp32-none-elf",
            "esp32s2" => "xtensa-esp32s2-none-elf",
            "esp32s3" => "xtensa-esp32s3-none-elf",
            "esp32c3" => "riscv32imc-unknown-none-elf",
            "esp32c6" => "riscv32imac-unknown-none-elf",
            _ => unreachable!(),
        };

        println!("⚙️  Compiling on-hardware tests for {} ({})...", mcu, target_arch);

        let mut build_cmd = Command::new("cargo");
        if target_arch.starts_with("xtensa") {
            build_cmd.arg("+esp").arg("-Zbuild-std=core,alloc");
        }
        build_cmd
            .arg("test")
            .arg("--no-run")
            .arg("--manifest-path").arg(".app_test_build/Cargo.toml")
            .arg("--target").arg(target_arch);
        build_cmd.env("RUSTFLAGS", "-C link-arg=-Tlinkall.x");

        let status = build_cmd.status().expect("Failed to compile test binary");
        if !status.success() { exit(status.code().unwrap_or(1)); }

        // Locate the compiled test binary under .app_test_build/target/
        let test_bin_dir = format!(".app_test_build/target/{}/debug/deps", target_arch);
        let test_binary = fs::read_dir(&test_bin_dir)
            .unwrap_or_else(|_| panic!("\u{274c} Could not read {}", test_bin_dir))
            .filter_map(|e| e.ok())
            .find(|e| {
                let name = e.file_name();
                let s = name.to_string_lossy();
                s.starts_with("app_test_build") && !s.contains('.')
            })
            .unwrap_or_else(|| panic!("\u{274c} Could not find test binary in {}", test_bin_dir))
            .path();

        println!("🔌 Flashing and running tests via probe-rs: {}", test_binary.display());
        let run_status = Command::new("probe-rs")
            .arg("test")
            .arg(&test_binary)
            .arg("--chip").arg(mcu)
            .status()
            .expect("probe-rs not found — install it with: cargo install probe-rs-tools");
        if !run_status.success() { exit(run_status.code().unwrap_or(1)); }
    }
}

