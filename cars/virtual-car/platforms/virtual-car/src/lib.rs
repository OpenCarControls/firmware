#![no_std]
extern crate alloc;

use alloc::string::String;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use prost::Message;

use core_interface::{
    VehicleStatePayload,
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, CMD_RESP_CHANNEL, VEHICLE_STATE_CHANNEL,
};

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.cars.virtual_car.v1.rs"));
}

// ── Vehicle State ─────────────────────────────────────────────────────────────

struct VirtualCarState {
    odometer: Option<u32>,
    is_driving: Option<bool>,
    are_doors_locked: Option<bool>,
    speed: Option<i32>,
    gear: Option<i32>,
}

impl VirtualCarState {
    const fn new() -> Self {
        Self {
            odometer: None,
            is_driving: None,
            are_doors_locked: None,
            speed: None,
            gear: None,
        }
    }
}

static CAR_STATE: Mutex<CriticalSectionRawMutex, VirtualCarState> =
    Mutex::new(VirtualCarState::new());

fn encode_state(state: &VirtualCarState) -> VehicleStatePayload {
    let basic = proto::BasicState {
        odometer: state.odometer,
        is_driving: state.is_driving,
        are_doors_locked: state.are_doors_locked,
    };
    let advanced = proto::AdvancedState {
        speed: state.speed,
        gear: state.gear,
    };
    VehicleStatePayload {
        basic: basic.encode_to_vec(),
        advanced: advanced.encode_to_vec(),
    }
}

// ── Vehicle Tasks ─────────────────────────────────────────────────────────────

/// Receives basic commands from `BASIC_CMD_CHANNEL`, processes them, pushes a
/// state update to `VEHICLE_STATE_CHANNEL`, and sends a `CommandResponse` to
/// `CMD_RESP_CHANNEL`. Both BLE and MQTT may send basic commands.
#[embassy_executor::task]
pub async fn handle_basic_commands_task() {
    loop {
        let inbound = BASIC_CMD_CHANNEL.receiver().receive().await;
        let (success, error_message) =
            match process_basic_command(inbound.bytes.as_slice()).await {
                Ok(()) => (true, String::new()),
                Err(e) => (false, String::from(e)),
            };
        let response = core_interface::proto::CommandResponse {
            message_id: inbound.message_id,
            success,
            error_message,
            response_data: None,
        };
        CMD_RESP_CHANNEL
            .sender()
            .send((inbound.transport, response))
            .await;
    }
}

async fn process_basic_command(bytes: &[u8]) -> Result<(), &'static str> {
    let cmd =
        proto::BasicCommand::decode(bytes).map_err(|_| "Failed to decode BasicCommand")?;
    match cmd.action {
        Some(proto::basic_command::Action::DoorLock(door_lock)) => {
            let payload = {
                let mut state = CAR_STATE.lock().await;
                state.are_doors_locked = Some(door_lock.lock);
                encode_state(&state)
                // MutexGuard is dropped here, before the channel send below
            };
            VEHICLE_STATE_CHANNEL.sender().send(payload).await;
            Ok(())
        }
        None => Err("No action in BasicCommand"),
    }
}

/// Receives advanced commands from `ADVANCED_CMD_CHANNEL` (BLE only) and sends
/// a `CommandResponse`. `AdvancedCommand` is currently empty so it always succeeds.
#[embassy_executor::task]
pub async fn handle_advanced_commands_task() {
    loop {
        let inbound = ADVANCED_CMD_CHANNEL.receiver().receive().await;
        let (success, error_message) =
            match proto::AdvancedCommand::decode(inbound.bytes.as_slice()) {
                Ok(_) => (true, String::new()),
                Err(_) => (false, String::from("Failed to decode AdvancedCommand")),
            };
        let response = core_interface::proto::CommandResponse {
            message_id: inbound.message_id,
            success,
            error_message,
            response_data: None,
        };
        CMD_RESP_CHANNEL
            .sender()
            .send((inbound.transport, response))
            .await;
    }
}

/// Periodically encodes and pushes the full vehicle state to `VEHICLE_STATE_CHANNEL`
/// without waiting for a command — the spontaneous state push mechanism.
///
/// The 5-second interval is a placeholder; it will become event-driven when
/// CAN bus integration is added.
#[embassy_executor::task]
pub async fn state_update_task() {
    loop {
        Timer::after(Duration::from_secs(5)).await;
        let payload = {
            let state = CAR_STATE.lock().await;
            encode_state(&state)
        };
        VEHICLE_STATE_CHANNEL.sender().send(payload).await;
    }
}
