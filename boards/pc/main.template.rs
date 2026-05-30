use embassy_executor::Spawner;

const PLATFORM_ID: u32 = {PLATFORM_ID};
{MTLS_CERTS}
{NETWORK_CONSTANTS}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    core_interface::init(PLATFORM_ID);
    board_pc::set_ble_paired_store_path(BLE_PAIRED_PHONES_FILE);
    core_interface::set_ble_max_bonded_phones(BLE_MAX_BONDED_PHONES);
    board_pc::start(&spawner);
{CAN_SPAWNS}{BLE_HTTP_SPAWN}{MQTT_DRIVER_SPAWN}    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_basic_commands_task().unwrap());
    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_advanced_commands_task().unwrap());
    spawner.spawn({VEHICLE_CRATE_IDENT}::state_update_task().unwrap());
    spawner.spawn({VEHICLE_CRATE_IDENT}::can_rx_task().unwrap());
}
