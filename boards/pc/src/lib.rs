mod ble;
mod can;
mod can_approval;
mod can_logger;
mod network;
mod phone_validations;

use core_interface::SYSTEM_COMMAND_CHANNEL;

pub use ble::{ble_http_task, set_ble_paired_store_path};
pub use can::socket_can_task;
pub use can_approval::{can_rw_approval_store_task, restore_can_rw_approval};
pub use can_logger::can_logger_task;
pub use network::mqtt_driver_task;
pub use phone_validations::{phone_validations_store_task, restore_phone_validations};

pub fn start(spawner: &embassy_executor::Spawner) {
    spawner.spawn(core_interface::process_ble_commands_task().unwrap());
    spawner.spawn(core_interface::process_mqtt_commands_task().unwrap());
    spawner.spawn(core_interface::route_responses_task().unwrap());
    spawner.spawn(core_interface::publish_state_task().unwrap());
    spawner.spawn(core_interface::publish_can_access_task().unwrap());
    spawner.spawn(core_interface::publish_command_availability_task().unwrap());
    spawner.spawn(can_rw_approval_store_task().unwrap());
    spawner.spawn(phone_validations_store_task().unwrap());
    core_interface::set_random_source(fill_random);
    spawner.spawn(system_command_task().unwrap());
}

/// The validation codes' random source: the kernel's CSPRNG.
fn fill_random(buf: &mut [u8]) {
    use std::io::Read as _;
    if let Err(e) = std::fs::File::open("/dev/urandom").and_then(|mut f| f.read_exact(buf)) {
        // Never fall back to a guessable code.
        panic!("no random source for the validation code: {e}");
    }
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
    // A validation never outlives its phone's bond.
    core_interface::prune_phone_validations().await;

    loop {
        let cmd = SYSTEM_COMMAND_CHANNEL.receiver().receive().await;
        match cmd.action {
            Some(core_interface::proto::system_command::Action::RestartCommand(_)) => {
                log::warn!("SYSTEM: restart requested (PC board stub)");
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
            #[allow(deprecated)]
            Some(core_interface::proto::system_command::Action::SetCanDebugEnabled(_)) => {
                // Deprecated: dropped by core-interface, never forwarded here.
            }
            #[allow(deprecated)]
            Some(core_interface::proto::system_command::Action::UpdateCanDebugFilters(_)) => {
                // Deprecated: dropped by core-interface, never forwarded here.
            }
            // Handled by core-interface's BLE dispatcher, never forwarded here.
            Some(core_interface::proto::system_command::Action::ListPairedPhones(_))
            | Some(core_interface::proto::system_command::Action::SetCanReadWriteApproval(_))
            | Some(core_interface::proto::system_command::Action::SetCanDevWriteUnlock(_))
            | Some(core_interface::proto::system_command::Action::ListPhoneValidationMethods(_))
            | Some(core_interface::proto::system_command::Action::StartPhoneValidation(_))
            | Some(core_interface::proto::system_command::Action::SubmitPhoneValidationCode(_))
            | Some(core_interface::proto::system_command::Action::CancelPhoneValidation(_))
            | None => {}
        }
    }
}
