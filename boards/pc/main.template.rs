use embassy_executor::Spawner;

const PLATFORM_ID: u32 = {PLATFORM_ID};
{MTLS_CERTS}
{NETWORK_CONSTANTS}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    log::info!("Booting firmware for vehicle platform: {}", stringify!({VEHICLE_CRATE_IDENT}));
    core_interface::init(PLATFORM_ID);
{BUILD_KIND_INIT}    board_pc::restore_can_rw_approval(CAN_RW_APPROVAL_FILE);
    board_pc::set_ble_paired_store_path(BLE_PAIRED_PHONES_FILE);
    core_interface::set_ble_max_bonded_phones(BLE_MAX_BONDED_PHONES);
    board_pc::start(&spawner);
{CAN_SPAWNS}{BLE_HTTP_SPAWN}{MQTT_DRIVER_SPAWN}{CAN_LOGGER_SPAWN}    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_basic_commands_task().unwrap());
    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_advanced_commands_task().unwrap());
    spawner.spawn({VEHICLE_CRATE_IDENT}::state_update_task().unwrap());
    spawner.spawn({VEHICLE_CRATE_IDENT}::can_rx_task().unwrap());
}
