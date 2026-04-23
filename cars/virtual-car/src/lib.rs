#![cfg_attr(not(test), no_std)]
extern crate alloc;

use alloc::string::String;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_can::{Id, StandardId};
use prost::Message;

use core_interface::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, CAN_RX_CHANNEL, CAN_TX_CHANNEL, CMD_RESP_CHANNEL,
    CanFilter, CanFrame, VEHICLE_STATE_CHANNEL, VehicleStatePayload,
};

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.cars.virtual_car.v1.rs"));
}

// ── Vehicle State ─────────────────────────────────────────────────────────────

pub(crate) struct VirtualCarState {
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

pub(crate) fn encode_state(state: &VirtualCarState) -> VehicleStatePayload {
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

// ── CAN Configuration ─────────────────────────────────────────────────────────

/// CAN acceptance filters for the virtual car. All filters operate on bus 0.
///
/// Frame IDs are synthetic — injected via vcan0 for testing:
///   0x100  speed frame: bytes [0..1] = speed as big-endian i16 (kph)
///   0x200  gear  frame: byte  [0]   = gear enum value (0–4)
///   0x300  doors frame: byte  [0]   = 0x01 if locked, 0x00 if unlocked
///
/// Mask 0x7FF = exact standard-ID match (all 11 bits must match).
pub const CAN_FILTERS: &[CanFilter] = &[
    CanFilter {
        bus_id: 0,
        id: Id::Standard(unsafe { StandardId::new_unchecked(0x100) }),
        mask: 0x7FF,
    },
    CanFilter {
        bus_id: 0,
        id: Id::Standard(unsafe { StandardId::new_unchecked(0x200) }),
        mask: 0x7FF,
    },
    CanFilter {
        bus_id: 0,
        id: Id::Standard(unsafe { StandardId::new_unchecked(0x300) }),
        mask: 0x7FF,
    },
];

// ── Vehicle Tasks ─────────────────────────────────────────────────────────────

/// Processes a single CAN frame: updates the shared vehicle state and pushes
/// a `VehicleStatePayload` to `VEHICLE_STATE_CHANNEL` if the frame triggered a change.
///
/// Frame interpretation (virtual-car synthetic protocol, bus 0):
///   0x100 — speed:  bytes [0..1] big-endian i16 (kph)
///   0x200 — gear:   byte  [0]   gear enum 0–4
///   0x300 — doors:  byte  [0]   0x01 = locked, 0x00 = unlocked
pub async fn handle_can_frame(frame: CanFrame) {
    let id_raw = match frame.id {
        Id::Standard(sid) => sid.as_raw() as u32,
        Id::Extended(eid) => eid.as_raw(),
    };
    let changed = match id_raw {
        0x100 if frame.dlc >= 2 => {
            let speed = i16::from_be_bytes([frame.data[0], frame.data[1]]) as i32;
            let mut state = CAR_STATE.lock().await;
            state.speed = Some(speed);
            true
        }
        0x200 if frame.dlc >= 1 => {
            let mut state = CAR_STATE.lock().await;
            state.gear = Some(frame.data[0] as i32);
            true
        }
        0x300 if frame.dlc >= 1 => {
            let mut state = CAR_STATE.lock().await;
            state.are_doors_locked = Some(frame.data[0] != 0);
            true
        }
        _ => false,
    };
    if changed {
        let payload = {
            let state = CAR_STATE.lock().await;
            encode_state(&state)
        };
        VEHICLE_STATE_CHANNEL.sender().send(payload).await;
    }
}

/// Receives CAN frames from `CAN_RX_CHANNEL`, updates internal vehicle state,
/// and pushes a `VehicleStatePayload` to `VEHICLE_STATE_CHANNEL` on each change.
#[embassy_executor::task]
pub async fn can_rx_task() {
    let receiver = CAN_RX_CHANNEL.receiver();
    loop {
        handle_can_frame(receiver.receive().await).await;
    }
}

/// Sends a CAN frame on bus 0. Vehicle tasks use this to request state from ECUs.
///
/// Command handlers that need to await a CAN response should:
///   1. Call `send_can_request(id, &data)` to transmit the request frame.
///   2. Await a vehicle-internal `Signal<CriticalSectionRawMutex, CanFrame>` with a
///      timeout via `embassy_time::with_timeout`.
///   3. `can_rx_task` signals it when the matching response frame arrives.
#[allow(dead_code)]
async fn send_can_request(id: StandardId, data: &[u8]) {
    let mut buf = [0u8; 8];
    let dlc = data.len().min(8) as u8;
    buf[..dlc as usize].copy_from_slice(&data[..dlc as usize]);
    CAN_TX_CHANNEL
        .sender()
        .send(CanFrame {
            bus_id: 0,
            id: Id::Standard(id),
            data: buf,
            dlc,
        })
        .await;
}

/// state update to `VEHICLE_STATE_CHANNEL`, and sends a `CommandResponse` to
/// `CMD_RESP_CHANNEL`. Both BLE and MQTT may send basic commands.
#[embassy_executor::task]
pub async fn handle_basic_commands_task() {
    loop {
        let inbound = BASIC_CMD_CHANNEL.receiver().receive().await;
        let (success, error_message) = match process_basic_command(inbound.bytes.as_slice()).await {
            Ok(()) => (true, String::new()),
            Err(e) => (false, String::from(e)),
        };
        let response = core_interface::proto::CommandResponse {
            message_id: inbound.message_id,
            success,
            error_message,
            response_data: None,
            status_code: if success { 1 } else { 2 },
        };
        CMD_RESP_CHANNEL
            .sender()
            .send((inbound.transport, response))
            .await;
    }
}

pub async fn process_basic_command(bytes: &[u8]) -> Result<(), &'static str> {
    let cmd = proto::BasicCommand::decode(bytes).map_err(|_| "Failed to decode BasicCommand")?;
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
            status_code: if success { 1 } else { 2 },
        };
        CMD_RESP_CHANNEL
            .sender()
            .send((inbound.transport, response))
            .await;
    }
}

/// Periodically encodes and pushes the full vehicle state to `VEHICLE_STATE_CHANNEL`
/// as a fallback heartbeat. Primary state updates are driven by `can_rx_task` on
/// each received CAN frame; this periodic push ensures the app sees a fresh snapshot
/// even if no frames arrive for a while.
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

// ── Unit Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use prost::Message;

    fn state_with(f: impl FnOnce(&mut VirtualCarState)) -> VirtualCarState {
        let mut s = VirtualCarState::new();
        f(&mut s);
        s
    }

    #[test]
    fn encode_state_odometer_appears_in_basic_not_advanced() {
        let state = state_with(|s| s.odometer = Some(12345));
        let payload = encode_state(&state);
        let basic = proto::BasicState::decode(payload.basic.as_slice()).unwrap();
        let advanced = proto::AdvancedState::decode(payload.advanced.as_slice()).unwrap();
        assert_eq!(basic.odometer, Some(12345));
        assert_eq!(advanced.speed, None);
        assert_eq!(advanced.gear, None);
    }

    #[test]
    fn encode_state_speed_appears_in_advanced_not_basic() {
        let state = state_with(|s| s.speed = Some(120));
        let payload = encode_state(&state);
        let basic = proto::BasicState::decode(payload.basic.as_slice()).unwrap();
        let advanced = proto::AdvancedState::decode(payload.advanced.as_slice()).unwrap();
        assert_eq!(advanced.speed, Some(120));
        assert_eq!(basic.odometer, None);
        assert_eq!(basic.is_driving, None);
        assert_eq!(basic.are_doors_locked, None);
    }

    #[test]
    fn encode_state_gear_appears_in_advanced() {
        let state = state_with(|s| s.gear = Some(1));
        let payload = encode_state(&state);
        let advanced = proto::AdvancedState::decode(payload.advanced.as_slice()).unwrap();
        assert_eq!(advanced.gear, Some(1));
    }

    #[test]
    fn encode_state_doors_locked_appears_in_basic() {
        let state = state_with(|s| s.are_doors_locked = Some(true));
        let payload = encode_state(&state);
        let basic = proto::BasicState::decode(payload.basic.as_slice()).unwrap();
        assert_eq!(basic.are_doors_locked, Some(true));
    }

    #[test]
    fn encode_state_all_none_produces_empty_encodings() {
        let state = VirtualCarState::new();
        let payload = encode_state(&state);
        let basic = proto::BasicState::decode(payload.basic.as_slice()).unwrap();
        let advanced = proto::AdvancedState::decode(payload.advanced.as_slice()).unwrap();
        assert_eq!(basic.odometer, None);
        assert_eq!(basic.is_driving, None);
        assert_eq!(basic.are_doors_locked, None);
        assert_eq!(advanced.speed, None);
        assert_eq!(advanced.gear, None);
    }
}
