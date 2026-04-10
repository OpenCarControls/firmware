#![no_std]

pub fn start(spawner: &embassy_executor::Spawner) {
    spawner.spawn(core_interface::process_ble_commands_task()).unwrap();
    spawner.spawn(core_interface::process_mqtt_commands_task()).unwrap();
    spawner.spawn(core_interface::route_responses_task()).unwrap();
    spawner.spawn(core_interface::publish_state_task()).unwrap();
}
