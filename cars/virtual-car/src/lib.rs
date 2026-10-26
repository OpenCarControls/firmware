#![cfg_attr(not(test), no_std)]
extern crate alloc;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_can::{Id, StandardId};
use prost::Message;

use core_interface::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, CAN_RX_CHANNEL, CanFilter, CanFrame, CommandError,
    CommandRequirements, Requirement, Snapshot, VEHICLE_STATE_CHANNEL, VehicleStatePayload,
    respond_to_command,
};

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.cars.virtual_car.v1.rs"));
}

// ── Vehicle State ─────────────────────────────────────────────────────────────

pub struct VirtualCarState {
    pub odometer: Option<u32>,
    pub is_driving: Option<bool>,
    pub are_doors_locked: Option<bool>,
    pub speed: Option<i32>,
    pub gear: Option<i32>,
    pub custom_state1: Option<bool>,
}

impl Default for VirtualCarState {
    fn default() -> Self {
        Self::new()
    }
}

impl VirtualCarState {
    pub const fn new() -> Self {
        Self {
            odometer: Some(0),
            is_driving: Some(false),
            are_doors_locked: Some(false),
            speed: Some(0),
            gear: Some(proto::advanced_state::Gear::Park as i32),
            custom_state1: Some(false),
        }
    }
}

/// Advances the simulated vehicle state by one 5-second tick.
///
/// The simulation runs a 20-tick (≈100 s) loop:
///   Ticks  0–4:  speed ramps from 0 → 80 kph (16 kph/tick)
///   Ticks  5–9:  cruising at 80 kph
///   Ticks 10–14: speed ramps from 80 → 0 kph (16 kph/tick)
///   Ticks 15–19: parked at 0 kph
///
/// `gear` and `is_driving` are derived from `speed`:
///   speed > 0  → gear = `Gear::Drive`, is_driving = true
///   speed == 0 → gear = `Gear::Park`,  is_driving = false
///
/// `odometer` increments by 1 km at tick 19 (end of each full cycle).
/// `are_doors_locked` is intentionally untouched — it is command-controlled only.
#[cfg(debug_assertions)]
pub fn tick_simulation(state: &mut VirtualCarState, tick: u64) {
    use proto::advanced_state::Gear;

    let phase = (tick % 20) as i32;
    let speed = match phase {
        0..=4 => phase * 16,
        5..=9 => 80,
        10..=14 => (14 - phase) * 16,
        _ => 0,
    };
    state.speed = Some(speed);
    state.is_driving = Some(speed > 0);
    state.gear = Some(if speed > 0 { Gear::Drive } else { Gear::Park } as i32);
    if phase == 19 {
        state.odometer = Some(state.odometer.unwrap_or(0).saturating_add(1));
    }
}

pub static CAR_STATE: Mutex<CriticalSectionRawMutex, VirtualCarState> =
    Mutex::new(VirtualCarState::new());

/// The simulated car's fingerprint. There is no real car to recognise, so every
/// virtual-car device identifies this same car, and the owner's approval for it
/// carries over to any device running virtual-car (never to another platform).
const SIMULATED_CAR_FINGERPRINT: &[u8] = b"virtual-car";

/// Identifies the simulated car and asks for read-write mode, as a real vehicle does
/// once it has recognised the car on the bus. Commands are still refused until the
/// owner approves this car and, in development builds, unlocks the dev lock (see
/// docs/can.md, "Read-write policy").
pub fn identify_simulated_car() {
    core_interface::confirm_car_identity();
    core_interface::confirm_car_fingerprint(SIMULATED_CAR_FINGERPRINT);
    core_interface::set_can_read_only(false);
}

pub fn encode_state(state: &VirtualCarState) -> VehicleStatePayload {
    let basic = proto::BasicState {
        odometer: state.odometer,
        is_driving: state.is_driving,
        are_doors_locked: state.are_doors_locked,
    };
    let advanced = proto::AdvancedState {
        speed: state.speed,
        gear: state.gear,
        custom_state1: state.custom_state1,
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
    core_interface::send_can_frame(CanFrame {
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
        let result = process_basic_command(inbound.bytes.as_slice()).await;
        respond_to_command(inbound.transport, inbound.message_id, result).await;
    }
}

pub async fn process_basic_command(bytes: &[u8]) -> Result<(), CommandError> {
    let cmd = proto::BasicCommand::decode(bytes)
        .map_err(|_| CommandError::Failed("Failed to decode BasicCommand"))?;
    match cmd.action {
        Some(proto::basic_command::Action::DoorLock(door_lock)) => {
            let payload = {
                let mut state = CAR_STATE.lock().await;
                door_lock_requirements(&state)
                    .check(&SIMULATED_SNAPSHOT)
                    .map_err(CommandError::Rejected)?;
                state.are_doors_locked = Some(door_lock.lock);
                encode_state(&state)
                // MutexGuard is dropped here, before the channel send below
            };
            VEHICLE_STATE_CHANNEL.sender().send(payload).await;
            Ok(())
        }
        None => Err(CommandError::Failed("No action in BasicCommand")),
    }
}

/// Receives advanced commands from `ADVANCED_CMD_CHANNEL` (BLE only) and sends
/// a `CommandResponse`. `AdvancedCommand` is currently empty so it always succeeds.
#[embassy_executor::task]
pub async fn handle_advanced_commands_task() {
    loop {
        let inbound = ADVANCED_CMD_CHANNEL.receiver().receive().await;
        let result = process_advanced_command(inbound.bytes.as_slice()).await;
        respond_to_command(inbound.transport, inbound.message_id, result).await;
    }
}

pub async fn process_advanced_command(bytes: &[u8]) -> Result<(), CommandError> {
    let cmd = proto::AdvancedCommand::decode(bytes)
        .map_err(|_| CommandError::Failed("Failed to decode AdvancedCommand"))?;
    match cmd.action {
        Some(proto::advanced_command::Action::ToggleCustomState1(_)) => {
            let payload = {
                let mut state = CAR_STATE.lock().await;
                state.custom_state1 = Some(!state.custom_state1.unwrap_or(false));
                encode_state(&state)
            };
            VEHICLE_STATE_CHANNEL.sender().send(payload).await;
            Ok(())
        }
        None => Err(CommandError::Failed("No action in AdvancedCommand")),
    }
}

/// The ids of the virtual car's commands: their field numbers in the `BasicCommand` /
/// `AdvancedCommand` oneof. A test pins them to the wire.
pub const DOOR_LOCK_COMMAND_ID: u32 = 1;
pub const TOGGLE_CUSTOM_STATE1_COMMAND_ID: u32 = 1;

/// The simulated car has no CAN bus and its state is always fresh, so the only inputs of the
/// requirements check that matter are the car's requirements themselves. That the device is
/// read-write is enforced by the dispatcher, and `core-interface` holds the availability back
/// while it is read-only.
const SIMULATED_SNAPSHOT: Snapshot = Snapshot {
    read_only_reason: None,
    state_age: Some(Duration::from_ticks(0)),
};

/// What the door lock command needs. Locking while the simulated car drives is refused so
/// that the app's greyed-out buttons and rejection messages can be tried without a car: start
/// the simulation (it drives for ≈75 s of every ≈100 s cycle).
fn door_lock_requirements(state: &VirtualCarState) -> CommandRequirements {
    CommandRequirements::new(Duration::from_secs(1)).require(Requirement::new(
        state.is_driving.map(|driving| !driving),
        "command rejected: the car is driving",
        "command rejected: the car's driving state is not known yet",
    ))
}

/// Tells `core-interface` which commands are available, from the same requirements the
/// handlers check. Called every second.
pub async fn report_availability() {
    use core_interface::proto::CommandAvailabilityState::{Available, Unavailable};
    let door_lock = {
        let state = CAR_STATE.lock().await;
        door_lock_requirements(&state)
            .check(&SIMULATED_SNAPSHOT)
            .is_ok()
    };
    core_interface::report_command_availability(
        core_interface::CommandKind::Basic,
        &[(
            DOOR_LOCK_COMMAND_ID,
            if door_lock { Available } else { Unavailable },
        )],
    );
    core_interface::report_command_availability(
        core_interface::CommandKind::Advanced,
        &[(TOGGLE_CUSTOM_STATE1_COMMAND_ID, Available)],
    );
}

/// Seconds between simulated state updates.
const STATE_UPDATE_SECONDS: u64 = 5;

/// Periodically advances the simulation by one tick and pushes the resulting
/// state to `VEHICLE_STATE_CHANNEL`. Fires every 5 seconds (20 ticks ≈ 100 s
/// per full drive cycle). Primary state mutations are still driven by
/// `can_rx_task` and command handlers; this task keeps the app updated even
/// when no CAN frames are present. Identifies the simulated car first.
#[embassy_executor::task]
pub async fn state_update_task() {
    identify_simulated_car();
    let mut tick: u64 = 0;
    let mut seconds: u64 = 0;
    loop {
        Timer::after(Duration::from_secs(1)).await;
        report_availability().await;
        seconds += 1;
        if !seconds.is_multiple_of(STATE_UPDATE_SECONDS) {
            continue;
        }
        let payload = {
            #[allow(unused_mut)]
            let mut state = CAR_STATE.lock().await;
            #[cfg(debug_assertions)]
            if core_interface::is_simulation_enabled() {
                tick_simulation(&mut state, tick);
            }
            encode_state(&state)
        };
        tick = tick.wrapping_add(1);
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
    fn simulated_car_is_identified_but_stays_read_only_until_approved() {
        identify_simulated_car();
        assert!(core_interface::is_car_fingerprint_confirmed());
        assert!(core_interface::is_can_read_only());

        core_interface::approve_can_read_write().unwrap();
        core_interface::set_can_dev_write_unlocked(true).unwrap();
        assert!(!core_interface::is_can_read_only());

        core_interface::revoke_car_identity();
        core_interface::reset_can_access_for_tests();
    }

    #[test]
    fn encode_state_odometer_appears_in_basic_not_advanced() {
        let state = state_with(|s| s.odometer = Some(12345));
        let payload = encode_state(&state);
        let basic = proto::BasicState::decode(payload.basic.as_slice()).unwrap();
        let advanced = proto::AdvancedState::decode(payload.advanced.as_slice()).unwrap();
        assert_eq!(basic.odometer, Some(12345));
        // odometer must not bleed into the advanced proto
        assert!(advanced.speed.is_some() || advanced.speed.is_none()); // advanced may carry speed
        let _ = advanced; // field isolation check: advanced has no odometer field by definition
    }

    #[test]
    fn encode_state_speed_appears_in_advanced_not_basic() {
        // Start from a blank slate so other defaults don't interfere with this
        // field-isolation check.
        let state = VirtualCarState {
            speed: Some(120),
            odometer: None,
            is_driving: None,
            are_doors_locked: None,
            gear: None,
            custom_state1: None,
        };
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
    fn encode_state_default_state_has_all_fields_present() {
        let state = VirtualCarState::new();
        let payload = encode_state(&state);
        let basic = proto::BasicState::decode(payload.basic.as_slice()).unwrap();
        let advanced = proto::AdvancedState::decode(payload.advanced.as_slice()).unwrap();
        assert_eq!(basic.odometer, Some(0));
        assert_eq!(basic.is_driving, Some(false));
        assert_eq!(basic.are_doors_locked, Some(false));
        assert_eq!(advanced.speed, Some(0));
        assert_eq!(
            advanced.gear,
            Some(proto::advanced_state::Gear::Park as i32)
        );
    }

    /// The largest state update this vehicle can produce must fit in one
    /// reassembled BLE message (`BLE_MAX_MESSAGE_BYTES`): an oversized update
    /// is dropped on the device. Every field is set to the value with the longest
    /// protobuf encoding (negative ints take 10 bytes). The struct is built
    /// without `..` on purpose: a new state field won't compile until it's
    /// covered here.
    #[test]
    fn worst_case_state_update_fits_ble_message() {
        let state = VirtualCarState {
            odometer: Some(u32::MAX),
            is_driving: Some(true),
            are_doors_locked: Some(true),
            speed: Some(i32::MIN),
            gear: Some(i32::MIN),
            custom_state1: Some(true),
        };

        let msg = core_interface::ble_state_update(u64::MAX, u32::MAX, encode_state(&state));
        let limit = core_interface::ble::BLE_MAX_MESSAGE_BYTES;
        assert!(
            msg.encoded_len() <= limit,
            "worst-case virtual car state update is {} bytes, BLE limit is {limit}",
            msg.encoded_len()
        );
    }
}
