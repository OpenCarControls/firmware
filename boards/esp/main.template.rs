#![no_std]
#![no_main]
extern crate alloc;

use esp_alloc as _;
use esp_backtrace as _;
use embassy_executor::Spawner;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};

const PLATFORM_ID: u32 = {PLATFORM_ID};

macro_rules! get_can_tx_pin { ($p:expr) => { $p.GPIO{CAN_TX_PIN} } }
pub(crate) use get_can_tx_pin;

macro_rules! get_can_rx_pin { ($p:expr) => { $p.GPIO{CAN_RX_PIN} } }
pub(crate) use get_can_rx_pin;
{MTLS_CERTS}
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    core_interface::init(PLATFORM_ID);
    board_esp::start(&spawner);
    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_basic_commands_task()).unwrap();
    spawner.spawn({VEHICLE_CRATE_IDENT}::handle_advanced_commands_task()).unwrap();
    spawner.spawn({VEHICLE_CRATE_IDENT}::state_update_task()).unwrap();

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}
