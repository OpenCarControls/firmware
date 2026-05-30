use crate::{
    Config, TargetBuilder, escape_rust_string, load_transport_contract, parse_broker_url,
    render_topic_from_template, validate_ble_transport_contract,
};
use serde::Deserialize;
use std::fs;
use std::process::{Command, exit};

/// Load `boards/esp/Cargo.toml` as a raw TOML value for version lookups.
fn load_esp_board_toml() -> toml::Value {
    let toml_str = fs::read_to_string("boards/esp/Cargo.toml")
        .expect("❌ Could not read boards/esp/Cargo.toml");
    toml::from_str(&toml_str).expect("❌ Failed to parse boards/esp/Cargo.toml")
}

/// Look up a crate version from `boards/esp/Cargo.toml`.
/// Checks `[dependencies]` first, then `[package.metadata.xtask-only-deps]`.
fn esp_dep_version(board_toml: &toml::Value, name: &str) -> String {
    if let Some(dep) = board_toml.get("dependencies").and_then(|d| d.get(name)) {
        if let Some(s) = dep.as_str() {
            return s.to_string();
        }
        if let Some(v) = dep.get("version").and_then(|v| v.as_str()) {
            return v.to_string();
        }
    }
    if let Some(v) = board_toml
        .get("package")
        .and_then(|p| p.get("metadata"))
        .and_then(|m| m.get("xtask-only-deps"))
        .and_then(|x| x.get(name))
        .and_then(|v| v.as_str())
    {
        return v.to_string();
    }
    panic!(
        "❌ Missing ESP dependency '{}'. Add it to [dependencies] or \
         [package.metadata.xtask-only-deps] in boards/esp/Cargo.toml.",
        name
    );
}

#[derive(Deserialize)]
struct EspBleConfig {
    #[serde(default)]
    device_name: Option<String>,
    #[serde(default = "default_pairing_button_pin")]
    pairing_button_pin: u8,
    #[serde(default = "default_pairing_button_hold_s")]
    pairing_button_hold_s: u32,
    #[serde(default = "default_max_bonded_phones")]
    max_bonded_phones: u8,
    #[serde(default = "default_controller_lease_ttl_s")]
    controller_lease_ttl_s: u32,
}

impl Default for EspBleConfig {
    fn default() -> Self {
        Self {
            device_name: None,
            pairing_button_pin: default_pairing_button_pin(),
            pairing_button_hold_s: default_pairing_button_hold_s(),
            max_bonded_phones: default_max_bonded_phones(),
            controller_lease_ttl_s: default_controller_lease_ttl_s(),
        }
    }
}

fn default_pairing_button_pin() -> u8 {
    0
}

fn default_pairing_button_hold_s() -> u32 {
    3
}

fn default_max_bonded_phones() -> u8 {
    8
}

fn default_controller_lease_ttl_s() -> u32 {
    15
}

/// Maps an MCU name from `config.toml` to its Rust no_std target triple.
fn mcu_to_target_arch(mcu: &str) -> &'static str {
    match mcu {
        "esp32"   => "xtensa-esp32-none-elf",
        "esp32s3" => "xtensa-esp32s3-none-elf",
        "esp32c3" => "riscv32imc-unknown-none-elf",
        "esp32c6" => "riscv32imac-unknown-none-elf",
        _ => unreachable!("Unsupported MCU: {}", mcu),
    }
}

#[derive(Deserialize)]
struct EspConfig {
    mcu: String,
    #[allow(dead_code)]
    modem_tx_pin: u8,
    #[allow(dead_code)]
    modem_rx_pin: u8,
    #[serde(default)]
    ble: EspBleConfig,
    #[serde(default)]
    can_buses: Vec<toml::Value>,
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

    fn execute_cargo_command(&self, config: &Config, cargo_cmd: &str, release: bool) {
        let esp_hw = Self::get_esp_config(config);
        let mcu = &esp_hw.mcu;

        let target_arch = mcu_to_target_arch(mcu);

        let mut cmd = Command::new("cargo");

        if target_arch.starts_with("xtensa") {
            cmd.arg("+esp");
        }

        cmd.arg(cargo_cmd)
            .current_dir(".app_build");

        if release {
            cmd.arg("--release");
        }

        let features = format!("esp-hal/{mcu},esp-rtos/{mcu},esp-backtrace/{mcu},esp-println/{mcu},esp-radio/{mcu},esp-storage/{mcu},esp-bootloader-esp-idf/{mcu}");
        cmd.arg("--features").arg(features);

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
        if esp.ble.pairing_button_pin > 48 {
            eprintln!(
                "❌ Error: [hardware.esp.ble].pairing_button_pin={} is out of range.",
                esp.ble.pairing_button_pin
            );
            exit(1);
        }
        if esp.ble.pairing_button_hold_s == 0
            || esp.ble.max_bonded_phones == 0
            || esp.ble.controller_lease_ttl_s == 0
        {
            eprintln!(
                "❌ Error: BLE lifecycle values must be > 0 (pairing_button_hold_s, max_bonded_phones, controller_lease_ttl_s)."
            );
            exit(1);
        }
    }

    fn generate_app_build(&self, config: &Config) {
        let transport = load_transport_contract();
        validate_ble_transport_contract(&transport);

        let esp_hw = Self::get_esp_config(config);
        let vehicle_platform = &config.target.platform;

        let (platform_id, can_bus_count) = crate::load_platform_meta(vehicle_platform);
        let (vehicle_crate_name, vehicle_crate_ident) = crate::load_vehicle_crate_info(vehicle_platform);

        // Validate CAN bus count
        if esp_hw.can_buses.len() < can_bus_count {
            eprintln!(
                "❌ Vehicle '{}' requires {} CAN bus(es) but [hardware.esp] only defines {} [[hardware.esp.can_buses]] entries.",
                vehicle_platform, can_bus_count, esp_hw.can_buses.len()
            );
            exit(1);
        }

        // Generate per-bus CAN hardware init expressions and task definitions.
        // FIXME: Only the first `can_bus_count` buses are wired up; extras in config are ignored.
        let mut can_hardware_init = String::new();
        let mut can_task_defs = String::new();
        let mut can_task_spawns = String::new();
        let mut mcp_spi_idx: u8 = 2; // SPI0/SPI1 reserved for flash; MCP2515 buses start at SPI2
        let filters_expr = format!("{}::CAN_FILTERS", vehicle_crate_ident);

        for (bus_id, bus) in esp_hw.can_buses.iter().take(can_bus_count).enumerate() {
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
                        "    let can_bus_{0} = board_esp::init_twai(peripherals.TWAI0, peripherals.GPIO{2}, peripherals.GPIO{1}, {3});\n",
                        bus_id, tx, rx, filters_expr
                    ));
                    can_task_defs.push_str(&format!(
                        "#[embassy_executor::task]\nasync fn can_bus_{0}_task(driver: board_esp::TwaiDriver) {{\n    board_esp::run_twai_loop(driver, {0}, {1}).await;\n}}\n",
                        bus_id, filters_expr
                    ));
                    can_task_spawns.push_str(&format!(
                        "                s.spawn(can_bus_{0}_task(can_bus_{0}).unwrap());\n",
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
                        "#[embassy_executor::task]\nasync fn can_bus_{0}_task(driver: board_esp::Mcp2515Driver, int_pin: board_esp::CanIntPin) {{\n    board_esp::run_mcp2515_loop(driver, int_pin, {0}, {1}).await;\n}}\n",
                        bus_id, filters_expr
                    ));
                    can_task_spawns.push_str(&format!(
                        "                s.spawn(can_bus_{0}_task(can_bus_{0}, can_bus_{0}_int).unwrap());\n",
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
            "{} = {{ path = \"../cars/{}\" }}\n",
            vehicle_crate_name, vehicle_platform
        ));
        let v = |name: &str| crate::ws_dep_version(&config.workspace_deps, name);
        let board_toml = load_esp_board_toml();
        let ev = |name: &str| esp_dep_version(&board_toml, name);
        cargo_toml.push_str(&format!("esp-hal = {{ version = \"{}\", features = [\"unstable\"] }}\n", ev("esp-hal")));
        cargo_toml.push_str(&format!("esp-rtos = {{ version = \"{}\", features = [\"embassy\", \"esp-radio\", \"esp-alloc\"] }}\n", ev("esp-rtos")));
        cargo_toml.push_str(&format!("esp-backtrace = {{ version = \"{}\", features = [\"panic-handler\", \"println\"] }}\n", ev("esp-backtrace")));
        cargo_toml.push_str(&format!("esp-println = {{ version = \"{}\", features = [\"log-04\"] }}\n", ev("esp-println")));
        cargo_toml.push_str(&format!("esp-alloc = \"{}\"\n", ev("esp-alloc")));
        cargo_toml.push_str(&format!("embassy-executor = \"{}\"\n", v("embassy-executor")));
        cargo_toml.push_str(&format!("embassy-time = \"{}\"\n", v("embassy-time")));
        cargo_toml.push_str(&format!("static_cell = \"{}\"\n", ev("static_cell")));
        // Direct dep so we can forward the MCU chip feature via --features esp-radio/<mcu>
        cargo_toml.push_str(&format!("esp-radio = {{ version = \"{}\", features = [\"wifi\", \"ble\", \"unstable\"] }}\n", ev("esp-radio")));
        // Direct dep so we can forward the MCU chip feature via --features esp-storage/<mcu>
        cargo_toml.push_str(&format!("esp-storage = {{ version = \"{}\", features = [\"critical-section\"] }}\n", ev("esp-storage")));
        // App descriptor for the ESP-IDF 2nd stage bootloader (sets min_efuse_blk_rev_full to 0)
        cargo_toml.push_str(&format!("esp-bootloader-esp-idf = {{ version = \"{}\" }}\n", ev("esp-bootloader-esp-idf")));

        // Debug-profile: optimize for size while keeping debug symbols so probe-rs can still
        // attach.  Without this, opt-level=0 leaves BLE/rand_chacha stack frames so large that
        // the default Embassy task stack overflows, corrupting return addresses and producing the
        // misleading infinite-recursion trace seen in the backtrace.
        cargo_toml.push_str("\n[profile.dev]\nopt-level = \"s\"\ndebug = true\n");

        // Build .app_build/src/main.rs from template
        let mtls_certs = crate::generate_mtls_certs(config);

        // Generate network constants and WiFi/MQTT driver spawn
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

        let ble_device_name = esp_hw
            .ble
            .device_name
            .as_deref()
            .unwrap_or(transport.ble.service.name.as_str());
        let ble_device_name = escape_rust_string(ble_device_name);

        let wifi_enabled = config.network.wifi.enabled;
        let wifi_ssid = if wifi_enabled {
            config.network.wifi.ssid.as_deref().unwrap_or_else(|| {
                eprintln!("\u{274c} [network.wifi].ssid is required for the ESP board when [network.wifi].enabled=true");
                exit(1);
            })
        } else {
            ""
        };
        let wifi_ssid = escape_rust_string(wifi_ssid);
        let wifi_password = escape_rust_string(config.network.wifi.password.as_deref().unwrap_or(""));
        let broker_host = escape_rust_string(&broker_host);
        let client_id = escape_rust_string(client_id);

        let ble_constants = format!(
            "const BLE_DEVICE_NAME_BASE: &str = \"{ble_name}\";\n\
             const BLE_PAIRING_BUTTON_PIN: u8 = {pair_btn};\n\
             const BLE_PAIRING_BUTTON_HOLD_S: u32 = {pair_hold};\n\
             const BLE_PAIRING_WINDOW_S: u32 = {pair_window};\n\
             const BLE_MAX_BONDED_PHONES: u8 = {max_bonds};\n\
             const BLE_CONTROLLER_LEASE_TTL_S: u32 = {lease_ttl};",
            ble_name = ble_device_name,
            pair_btn = esp_hw.ble.pairing_button_pin,
            pair_hold = esp_hw.ble.pairing_button_hold_s,
            pair_window = transport.ble.pairing.pairing_window_seconds,
            max_bonds = esp_hw.ble.max_bonded_phones,
            lease_ttl = esp_hw.ble.controller_lease_ttl_s,
        );

        let network_constants = if wifi_enabled {
            format!(
                "const WIFI_SSID: &str = \"{ssid}\";\n\
                 const WIFI_PASSWORD: &str = \"{wifi_pw}\";\n\
                 const MQTT_BROKER_HOST: &str = \"{host}\";\n\
                 const MQTT_BROKER_PORT: u16 = {port};\n\
                 const MQTT_CLIENT_ID: &str = \"{cid}\";\n\
                 const MQTT_CMD_TOPIC: &str = \"{cmd_topic}\";\n\
                 const MQTT_DATA_TOPIC: &str = \"{data_topic}\";\n\
                 const MQTT_USERNAME: &str = \"{user}\";\n\
                 const MQTT_PASSWORD: &str = \"{pass}\";\n\
                 {ble}",
                ssid = wifi_ssid,
                wifi_pw = wifi_password,
                host = broker_host,
                port = broker_port,
                cid = client_id,
                cmd_topic = mqtt_cmd_topic,
                data_topic = mqtt_data_topic,
                user = mqtt_username,
                pass = mqtt_password,
                ble = ble_constants,
            )
        } else {
            ble_constants
        };

        let network_hardware_init = if wifi_enabled {
            concat!(
                "    let wifi_stack = board_esp::init_wifi(\n",
                "        &spawner,\n",
                "        peripherals.WIFI,\n",
                "        WIFI_SSID,\n",
                "        WIFI_PASSWORD,\n",
                "    );\n",
            ).to_string()
        } else {
            String::new()
        };

        let ble_driver_spawn = concat!(
            "    spawner\n",
            "        .spawn(board_esp::ble_transport_task(\n",
            "            peripherals.BT,\n",
            "            peripherals.FLASH,\n",
            "            BLE_DEVICE_NAME_BASE,\n",
            "        ).unwrap());\n",
        ).to_string();

        let mqtt_driver_spawn = if wifi_enabled {
            concat!(
                "    spawner\n",
                "        .spawn(board_esp::mqtt_driver_task(\n",
                "            wifi_stack,\n",
                "            MQTT_BROKER_HOST,\n",
                "            MQTT_BROKER_PORT,\n",
                "            MQTT_CLIENT_ID,\n",
                "            MQTT_CMD_TOPIC,\n",
                "            MQTT_DATA_TOPIC,\n",
                "            MQTT_USERNAME,\n",
                "            MQTT_PASSWORD,\n",
                "        ).unwrap());\n",
            ).to_string()
        } else {
            "    // WiFi disabled: MQTT driver not spawned.\n".to_string()
        };

        let template = fs::read_to_string("boards/esp/main.template.rs")
            .expect("\u{274c} Could not read boards/esp/main.template.rs");
        let main_rs = template
            .replace("{PLATFORM_ID}", &format!("0x{:08X}", platform_id))
            .replace("{VEHICLE_CRATE_IDENT}", &vehicle_crate_ident)
            .replace("{CAN_HARDWARE_INIT}", &can_hardware_init)
            .replace("{CORE1_CAN_TASK_DEFS}", &can_task_defs)
            .replace("{CORE1_TASK_SPAWNS}", &can_task_spawns)
            .replace("{MTLS_CERTS}", &mtls_certs)
            .replace("{NETWORK_CONSTANTS}", &network_constants)
            .replace("{NETWORK_HARDWARE_INIT}", &network_hardware_init)
            .replace("{BLE_DRIVER_SPAWN}", &ble_driver_spawn)
            .replace("{MQTT_DRIVER_SPAWN}", &mqtt_driver_spawn);

        let target_arch = mcu_to_target_arch(&esp_hw.mcu);
        let cargo_config = if target_arch.starts_with("xtensa") {
            format!(
                "[unstable]\nbuild-std = [\"core\", \"alloc\"]\n\n\
                 [build]\ntarget = \"{target}\"\n\n\
                 [target.{target}]\nrustflags = [\"-C\", \"link-arg=-Tlinkall.x\"]\n",
                target = target_arch
            )
        } else {
            format!(
                "[build]\ntarget = \"{target}\"\n\n\
                 [target.{target}]\nrustflags = [\"-C\", \"link-arg=-Tlinkall.x\"]\n",
                target = target_arch
            )
        };

        fs::create_dir_all(".app_build/src").expect("Failed to create .app_build/src");
        fs::create_dir_all(".app_build/.cargo").expect("Failed to create .app_build/.cargo");
        fs::write(".app_build/Cargo.toml", cargo_toml).expect("Failed to write .app_build/Cargo.toml");
        fs::write(".app_build/.cargo/config.toml", cargo_config).expect("Failed to write .app_build/.cargo/config.toml");
        fs::write(".app_build/src/main.rs", main_rs).expect("Failed to write .app_build/src/main.rs");
    }

    fn compile(&self, config: &Config, release: bool) {
        let profile = if release { "release" } else { "debug (unoptimized)" };
        println!("⚙️  Compiling the bare-metal firmware ({profile})...");
        self.execute_cargo_command(config, "build", release);
    }

    fn flash(&self, config: &Config, port: Option<&str>, monitor: bool, release: bool) {
        let esp_hw = Self::get_esp_config(config);
        let mcu = &esp_hw.mcu;

        let target_arch = mcu_to_target_arch(mcu);

        let profile_dir = if release { "release" } else { "debug" };
        let elf_path = format!(".app_build/target/{}/{}/app-build", target_arch, profile_dir);

        println!(
            "⚡ Flashing {} ({}) via {}...",
            mcu,
            elf_path,
            port.unwrap_or("auto-detected port"),
        );

        let mut cmd = Command::new("espflash");
        cmd.arg("flash");

        if let Some(p) = port {
            cmd.arg("--port").arg(p);
        }

        if monitor {
            cmd.arg("--monitor");
        }

        cmd.arg(&elf_path);

        let status = cmd.status().expect("❌ Failed to execute espflash — is it installed?");
        if !status.success() {
            exit(status.code().unwrap_or(1));
        }

        if !monitor {
            println!("✅ Flash complete.");
        }
    }

    fn run(&self, config: &Config) {
        println!("🚀 Running the firmware build pipeline...");
        self.execute_cargo_command(config, "run", true);
    }

    fn clippy(&self, config: &Config) {
        println!("🔍 Running clippy on the firmware build pipeline...");
        self.execute_cargo_command(config, "clippy", true);
    }

    fn generate_app_test_build(&self, config: &Config) {
        let esp_hw = Self::get_esp_config(config);
        let mcu = &esp_hw.mcu;

        // Read platform meta for PLATFORM_ID
        let (platform_id, _) = crate::load_platform_meta(&config.target.platform);

        // Read any user-defined on-hardware tests from boards/esp/tests/hardware.rs
        let extra_tests = fs::read_to_string("boards/esp/tests/hardware.rs").unwrap_or_default();

        // Build .app_test_build/Cargo.toml
        let mut cargo_toml = String::new();
        cargo_toml.push_str("[workspace]\n\n[package]\nname = \"app-test-build\"\nversion = \"0.1.0\"\nedition = \"2024\"\n\n");
        cargo_toml.push_str("[dependencies]\n");
        cargo_toml.push_str("core-interface = { path = \"../core-interface\" }\n");
        cargo_toml.push_str("board-esp = { path = \"../boards/esp\", features = [\"hardware\"] }\n");
        let v = |name: &str| crate::ws_dep_version(&config.workspace_deps, name);
        let board_toml = load_esp_board_toml();
        let ev = |name: &str| esp_dep_version(&board_toml, name);
        cargo_toml.push_str(&format!("esp-hal = {{ version = \"{}\", features = [\"unstable\"] }}\n", ev("esp-hal")));
        cargo_toml.push_str(&format!("esp-rtos = {{ version = \"{}\", features = [\"embassy\", \"esp-alloc\"] }}\n", ev("esp-rtos")));
        cargo_toml.push_str(&format!("esp-radio = {{ version = \"{}\", features = [\"unstable\"] }}\n", ev("esp-radio")));
        cargo_toml.push_str(&format!("esp-storage = {{ version = \"{}\", features = [\"critical-section\"] }}\n", ev("esp-storage")));
        cargo_toml.push_str(&format!("esp-alloc = \"{}\"\n", ev("esp-alloc")));
        cargo_toml.push_str(&format!("esp-println = {{ version = \"{}\", features = [\"log-04\"] }}\n", ev("esp-println")));
        cargo_toml.push_str(&format!("embassy-executor = \"{}\"\n", v("embassy-executor")));
        cargo_toml.push_str(&format!("embassy-time = \"{}\"\n", v("embassy-time")));
        cargo_toml.push_str(&format!("static_cell = \"{}\"\n", ev("static_cell")));
        // external-executor: pass esp_rtos::embassy::Executor to the tests macro.
        // embassy-09: also required by the macro even with external-executor.
        // xtensa-semihosting: required for Xtensa targets (openocd-semihosting interface).
        // semihosting + panic-handler: re-enable defaults (workspace has default-features = false).
        // Do NOT include esp-backtrace here — embedded-test's panic-handler feature covers it,
        // and the chip feature for esp-backtrace is forwarded via board-esp's hardware feature.
        let is_xtensa = matches!(mcu.as_str(), "esp32" | "esp32s3");
        let embedded_test_features = if is_xtensa {
            "\"external-executor\", \"embassy-09\", \"semihosting\", \"xtensa-semihosting\", \"panic-handler\""
        } else {
            "\"external-executor\", \"embassy-09\", \"semihosting\", \"panic-handler\""
        };
        cargo_toml.push_str(&format!("embedded-test = {{ version = \"{}\", features = [{embedded_test_features}] }}\n", ev("embedded-test")));
        // No defmt in test build: _defmt_timestamp and _defmt_panic are not wired for Xtensa
        // without panic-probe. Test pass/fail is communicated via semihosting, not defmt RTT.
        cargo_toml.push_str(&format!("embedded-can = \"{}\"\n", v("embedded-can")));
        cargo_toml.push_str(&format!("esp-bootloader-esp-idf = {{ version = \"{}\" }}\n", ev("esp-bootloader-esp-idf")));

        // Chip-specific feature string — must include esp-radio/{mcu}, esp-storage/{mcu},
        // and esp-backtrace/{mcu} to satisfy build scripts pulled in transitively by board-esp's hardware feature.
        let features = format!(
            "\"esp-hal/{mcu}\", \"esp-rtos/{mcu}\", \"esp-println/{mcu}\", \"esp-radio/{mcu}\", \"esp-storage/{mcu}\", \"esp-bootloader-esp-idf/{mcu}\""
        );
        cargo_toml.push_str(&format!("\n[features]\ndefault = [{}]\n", features));

        // embedded-test requires harness = false for the binary target.
        cargo_toml.push_str("\n[[bin]]\nname = \"app-test-build\"\ntest = true\nharness = false\n");

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

        let target_arch = mcu_to_target_arch(mcu);

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
        build_cmd.env("RUSTFLAGS", "-C link-arg=-Tlinkall.x -C link-arg=-Tembedded-test.x");

        let status = build_cmd.status().expect("Failed to compile test binary");
        if !status.success() { exit(status.code().unwrap_or(1)); }

        // Locate the compiled test binary under .app_test_build/target/
        // Pick the most recently modified match to avoid stale hashes from prior builds.
        let test_bin_dir = format!(".app_test_build/target/{}/debug/deps", target_arch);
        let test_binary = fs::read_dir(&test_bin_dir)
            .unwrap_or_else(|_| panic!("\u{274c} Could not read {}", test_bin_dir))
            .filter_map(|e| e.ok())
            .filter(|e| {
                let name = e.file_name();
                let s = name.to_string_lossy();
                s.starts_with("app_test_build") && !s.contains('.')
            })
            .max_by_key(|e| e.metadata().and_then(|m| m.modified()).ok())
            .unwrap_or_else(|| panic!("\u{274c} Could not find test binary in {}", test_bin_dir))
            .path();

        println!("🔌 Flashing and running tests via probe-rs: {}", test_binary.display());
        // probe-rs needs raw USB access; in the dev container the USB device is
        // owned by root:root so we use `sudo` (passwordless in devcontainer) when
        // the direct open fails due to permissions.  Resolve the full path so sudo
        // can find the binary even if /usr/local/cargo/bin isn't in root's PATH.
        let probe_rs_bin = std::process::Command::new("sh")
            .args(["-c", "which probe-rs"])
            .output()
            .ok()
            .and_then(|o| if o.status.success() { Some(o.stdout) } else { None })
            .map(|b| String::from_utf8_lossy(&b).trim().to_string())
            .unwrap_or_else(|| "probe-rs".to_string());
        let run_status = Command::new("sudo")
            .arg("--preserve-env")
            .arg(&probe_rs_bin)
            .arg("run")
            .arg(&test_binary)
            .arg("--chip").arg(mcu)
            .status()
            .expect("probe-rs not found — install it with: cargo install probe-rs-tools");
        if !run_status.success() { exit(run_status.code().unwrap_or(1)); }
    }
}

