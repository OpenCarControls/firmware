#![no_std]

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.core.v1.rs"));
}

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

pub enum LedCommand {
    On,
    Off,
}

pub static LED_CHANNEL: Channel<CriticalSectionRawMutex, LedCommand, 1> = Channel::new();

#[embassy_executor::task]
pub async fn blinky_task(interval_ms: u64) {
    let delay = Duration::from_millis(interval_ms);
    let sender = LED_CHANNEL.sender();
    loop {
        sender.send(LedCommand::On).await;
        Timer::after(delay).await;
        sender.send(LedCommand::Off).await;
        Timer::after(delay).await;
    }
}

pub trait CarController {}
