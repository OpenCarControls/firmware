//! Board support crate for the ESP32 family.
//!
//! Wires the `core-interface` task set to ESP32-specific hardware drivers: BLE
//! (trouble-host), WiFi/MQTT (esp-radio + embassy-net), and CAN buses (TWAI and/or
//! MCP2515). The public API is kept minimal: [`start`] spawns the core protocol
//! tasks, and hardware-init functions are re-exported for use by the generated
//! `main.rs` entry point produced by `xtask`.

#![cfg_attr(not(test), no_std)]
#[cfg(not(test))]
extern crate alloc;

mod ble;
mod can;
mod network;

#[cfg(feature = "hardware")]
mod hardware {
    use super::{ble, can, network};

    pub use ble::{ble_lifecycle_task, ble_transport_task};

    pub use can::{
        CanIntPin, CanSpeed, Mcp2515Driver, McpSpeed, TwaiDriver, init_mcp2515, init_twai,
        run_mcp2515_loop, run_twai_loop,
    };

    pub use network::{WifiStack, init_wifi, mqtt_driver_task};

    use core_interface::SYSTEM_COMMAND_CHANNEL;

    fn pairing_window_open_or_warn(op_name: &str) -> bool {
        let open = core_interface::is_pairing_window_open();
        if !open {
            log::warn!("SYSTEM: {} denied, pairing window is closed", op_name);
        }
        open
    }

    /// Spawns all `core-interface` protocol tasks on the provided Embassy executor.
    /// Called once from the generated `main.rs` entry point running on Core 0.
    /// Hardware-specific tasks (BLE transport, CAN loops, MQTT driver) are spawned
    /// separately by `main.rs` after hardware peripherals have been initialised.
    pub fn start(spawner: &embassy_executor::Spawner) {
        spawner.spawn(core_interface::process_ble_commands_task().unwrap());
        spawner.spawn(core_interface::process_mqtt_commands_task().unwrap());
        spawner.spawn(core_interface::route_responses_task().unwrap());
        spawner.spawn(core_interface::publish_state_task().unwrap());
        spawner.spawn(core_interface::publish_can_debug_task().unwrap());
        spawner.spawn(system_command_task().unwrap());
    }

    #[embassy_executor::task]
    pub async fn system_command_task() {
        // TODO: This should be moved to core-interface and the ESP should just implement it as a trait.
        loop {
            let cmd = SYSTEM_COMMAND_CHANNEL.receiver().receive().await;
            match cmd.action {
                Some(core_interface::proto::system_command::Action::RestartCommand(_)) => {
                    log::warn!("SYSTEM: restart requested (ESP handler not wired yet)");
                }
                Some(core_interface::proto::system_command::Action::ListPairedPhones(_)) => {
                    let phones = core_interface::list_paired_phones().await;
                    log::debug!(
                        "SYSTEM: list paired phones requested, count={} (ESP runtime)",
                        phones.len()
                    );
                }
                Some(core_interface::proto::system_command::Action::RemovePairedPhone(req)) => {
                    if !pairing_window_open_or_warn("remove paired phone") {
                        continue;
                    }
                    let removed = core_interface::remove_paired_phone(&req.device_id).await;
                    log::debug!(
                        "SYSTEM: remove paired phone requested, removed={}, device_id_len={}",
                        removed,
                        req.device_id.len(),
                    );
                }
                Some(core_interface::proto::system_command::Action::ClearPairedPhones(_)) => {
                    if !pairing_window_open_or_warn("clear paired phones") {
                        continue;
                    }
                    let removed = core_interface::clear_paired_phones().await;
                    log::info!(
                        "SYSTEM: clear paired phones requested, removed={} (ESP runtime)",
                        removed
                    );
                }
                Some(core_interface::proto::system_command::Action::UpsertPairedPhone(req)) => {
                    if !pairing_window_open_or_warn("add paired phone") {
                        continue;
                    }
                    let added = core_interface::add_paired_phone(&req.device_id).await;
                    log::debug!(
                        "SYSTEM: upsert paired phone requested, accepted={}, device_id_len={}",
                        added,
                        req.device_id.len(),
                    );
                }
                Some(core_interface::proto::system_command::Action::SetCanDebugEnabled(_))
                | Some(core_interface::proto::system_command::Action::UpdateCanDebugFilters(_))
                | None => {}
            }
        }
    }
}

#[cfg(feature = "hardware")]
pub use hardware::*;
