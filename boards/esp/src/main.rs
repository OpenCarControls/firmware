#![no_std]
#![no_main]

extern crate alloc;

use esp_alloc as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    timer::timg::TimerGroup,
};

include!(concat!(env!("OUT_DIR"), "/generated_config.rs"));

#[embassy_executor::task]
async fn led_driver_task(mut led: Output<'static>) {
    let receiver = core_interface::LED_CHANNEL.receiver();
    loop {
        match receiver.receive().await {
            core_interface::LedCommand::On  => { let _ = led.set_high(); }
            core_interface::LedCommand::Off => { let _ = led.set_low(); }
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let led_pin = get_led_pin!(peripherals);
    let led = Output::new(led_pin, Level::High, OutputConfig::default());

    spawner.spawn(core_interface::blinky_task(BLINK_INTERVAL_MS)).unwrap();
    spawner.spawn(led_driver_task(led)).unwrap();

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
