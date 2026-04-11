use embassy_executor::Spawner;

const PLATFORM_ID: u32 = {PLATFORM_ID};
{MTLS_CERTS}
{NETWORK_CONSTANTS}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    core_interface::init(PLATFORM_ID);
    board_pc::start(&spawner);
{CAN_SPAWNS}{MQTT_DRIVER_SPAWN}    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_basic_commands_task()).unwrap();
    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_advanced_commands_task()).unwrap();
    spawner.spawn({VEHICLE_CRATE_IDENT}::state_update_task()).unwrap();
    spawner.spawn({VEHICLE_CRATE_IDENT}::can_rx_task()).unwrap();
}
