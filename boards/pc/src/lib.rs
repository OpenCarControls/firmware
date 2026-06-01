#![cfg_attr(not(feature = "hardware"), no_std)]

mod ble;
mod can;
mod network;

#[cfg(feature = "hardware")]
mod hardware {
    use super::{ble, can, network};

    use core_interface::SYSTEM_COMMAND_CHANNEL;

    pub use ble::{ble_http_task, set_ble_paired_store_path};
    pub use can::socket_can_task;
    pub use network::mqtt_driver_task;

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
        // Restore persisted bonded-phone registry on startup.
        let path = ble::ble_paired_store_path();
        let persisted = ble::load_paired_phones_from_file(path);
        for id in &persisted {
            let _ = core_interface::add_paired_phone(id).await;
        }
        if !persisted.is_empty() {
            log::info!(
                "BLE store: restored {} paired phone(s) from {}",
                persisted.len(),
                path
            );
        }

        loop {
            let cmd = SYSTEM_COMMAND_CHANNEL.receiver().receive().await;
            match cmd.action {
                Some(core_interface::proto::system_command::Action::RestartCommand(_)) => {
                    log::warn!("SYSTEM: restart requested (PC board stub)");
                }
                Some(core_interface::proto::system_command::Action::ListPairedPhones(_)) => {
                    let phones = core_interface::list_paired_phones().await;
                    log::debug!(
                        "SYSTEM: list paired phones requested, count={} (PC runtime)",
                        phones.len()
                    );
                }
                Some(core_interface::proto::system_command::Action::RemovePairedPhone(req)) => {
                    if !core_interface::is_pairing_window_open() {
                        log::warn!("SYSTEM: remove paired phone denied, pairing window is closed");
                        continue;
                    }
                    let removed = core_interface::remove_paired_phone(&req.device_id).await;
                    if removed {
                        ble::persist_paired_phones().await;
                    }
                    log::debug!(
                        "SYSTEM: remove paired phone requested, removed={}, device_id_len={}",
                        removed,
                        req.device_id.len(),
                    );
                }
                Some(core_interface::proto::system_command::Action::ClearPairedPhones(_)) => {
                    if !core_interface::is_pairing_window_open() {
                        log::warn!("SYSTEM: clear paired phones denied, pairing window is closed");
                        continue;
                    }
                    let removed = core_interface::clear_paired_phones().await;
                    if removed > 0 {
                        ble::persist_paired_phones().await;
                    }
                    log::info!(
                        "SYSTEM: clear paired phones requested, removed={} (PC runtime)",
                        removed
                    );
                }
                Some(core_interface::proto::system_command::Action::UpsertPairedPhone(req)) => {
                    if !core_interface::is_pairing_window_open() {
                        log::warn!("SYSTEM: add paired phone denied, pairing window is closed");
                        continue;
                    }
                    let added = core_interface::add_paired_phone(&req.device_id).await;
                    if added {
                        ble::persist_paired_phones().await;
                    }
                    log::debug!(
                        "SYSTEM: upsert paired phone requested, accepted={}, device_id_len={}",
                        added,
                        req.device_id.len()
                    );
                }
                Some(core_interface::proto::system_command::Action::SetCanDebugEnabled(_)) => {
                    // Consumed by core-interface and not forwarded here.
                }
                Some(core_interface::proto::system_command::Action::UpdateCanDebugFilters(_)) => {
                    // Consumed by core-interface and not forwarded here.
                }
                None => {}
            }
        }
    }
}

#[cfg(feature = "hardware")]
pub use hardware::*;
