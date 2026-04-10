use embassy_executor::Spawner;

const PLATFORM_ID: u32 = {PLATFORM_ID};
#[allow(dead_code)]
const CAN_INTERFACE: &str = "{CAN_INTERFACE}";
{MTLS_CERTS}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    core_interface::init(PLATFORM_ID);
    board_pc::start(&spawner);
    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_basic_commands_task()).unwrap();
    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_advanced_commands_task()).unwrap();
    spawner.spawn({VEHICLE_CRATE_IDENT}::state_update_task()).unwrap();
}
