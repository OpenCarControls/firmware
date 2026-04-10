use embassy_executor::Spawner;

#[embassy_executor::task]
async fn led_driver_task() {
    let receiver = core_interface::LED_CHANNEL.receiver();
    loop {
        match receiver.receive().await {
            core_interface::LedCommand::On  => println!("PC SIM: LED turned ON"),
            core_interface::LedCommand::Off => println!("PC SIM: LED turned OFF"),
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    println!("Starting PC Simulator...");
    spawner.spawn(core_interface::blinky_task(500)).unwrap();
    spawner.spawn(led_driver_task()).unwrap();
}
