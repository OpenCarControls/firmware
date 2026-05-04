#![cfg_attr(not(test), no_std)]
#[cfg(not(test))]
extern crate alloc;

mod ble;
mod can;
mod network;

#[cfg(feature = "hardware")]
pub use ble::{ble_lifecycle_task, ble_transport_task};

#[cfg(feature = "hardware")]
pub use can::{
    CanIntPin, CanSpeed, Mcp2515Driver, McpSpeed, TwaiDriver, init_mcp2515, init_twai,
    run_mcp2515_loop, run_twai_loop,
};

#[cfg(feature = "hardware")]
pub use network::{SharedRadioController, WifiStack, init_radio, init_wifi, mqtt_driver_task};

#[cfg(feature = "hardware")]
use core_interface::SYSTEM_COMMAND_CHANNEL;

#[cfg(feature = "hardware")]
fn pairing_window_open_or_warn(op_name: &str) -> bool {
    if core_interface::is_pairing_window_open() {
        true
    } else {
        log::warn!(
            "SYSTEM: {} denied, pairing window is closed",
            op_name
        );
        false
    }
}

#[cfg(feature = "hardware")]
async fn persist_pairs_if(changed: bool) {
    // TODO: This should be with BLE, not in the general board lib.
    if changed {
        ble::persist_paired_phones_to_store().await;
    }
}

#[cfg(feature = "hardware")]
pub fn start(spawner: &embassy_executor::Spawner) {
    spawner
        .spawn(core_interface::process_ble_commands_task())
        .unwrap();
    spawner
        .spawn(core_interface::process_mqtt_commands_task())
        .unwrap();
    spawner
        .spawn(core_interface::route_responses_task())
        .unwrap();
    spawner
        .spawn(core_interface::publish_state_task())
        .unwrap();
    spawner
        .spawn(core_interface::publish_can_debug_task())
        .unwrap();
    spawner
        .spawn(system_command_task())
        .unwrap();
}

#[cfg(feature = "hardware")]
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
                log::info!(
                    "SYSTEM: list paired phones requested, count={} (ESP runtime)",
                    phones.len()
                );
            }
            Some(core_interface::proto::system_command::Action::RemovePairedPhone(req)) => {
                if !pairing_window_open_or_warn("remove paired phone") {
                    continue;
                }
                let removed = core_interface::remove_paired_phone(&req.device_id).await;
                persist_pairs_if(removed).await;
                log::info!(
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
                persist_pairs_if(removed > 0).await;
                log::warn!(
                    "SYSTEM: clear paired phones requested, removed={} (ESP runtime)",
                    removed
                );
            }
            Some(core_interface::proto::system_command::Action::UpsertPairedPhone(req)) => {
                if !pairing_window_open_or_warn("add paired phone") {
                    continue;
                }
                let added = core_interface::add_paired_phone(&req.device_id).await;
                persist_pairs_if(added).await;
                log::info!(
                    "SYSTEM: upsert paired phone requested, accepted={}, device_id_len={}",
                    added,
                    req.device_id.len(),
                );
            }
            Some(core_interface::proto::system_command::Action::SetCanDebugEnabled(_)) => {}
            Some(core_interface::proto::system_command::Action::UpdateCanDebugFilters(_)) => {}
            None => {}
        }
    }
}
