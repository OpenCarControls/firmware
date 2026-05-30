use prost::Message;
use virtual_car_controller::{
    VirtualCarState, encode_state,
    proto::{AdvancedState, BasicState},
    tick_simulation,
};

fn decode_basic(payload: &core_interface::VehicleStatePayload) -> BasicState {
    BasicState::decode(payload.basic.as_slice()).unwrap()
}

fn decode_advanced(payload: &core_interface::VehicleStatePayload) -> AdvancedState {
    AdvancedState::decode(payload.advanced.as_slice()).unwrap()
}

// ── Initial state ─────────────────────────────────────────────────────────────

#[test]
fn initial_state_all_fields_present() {
    let state = VirtualCarState::new();
    let payload = encode_state(&state);
    let basic = decode_basic(&payload);
    let advanced = decode_advanced(&payload);

    assert!(basic.odometer.is_some(), "odometer should have a value");
    assert!(basic.is_driving.is_some(), "is_driving should have a value");
    assert!(
        basic.are_doors_locked.is_some(),
        "are_doors_locked should have a value"
    );
    assert!(advanced.speed.is_some(), "speed should have a value");
    assert!(advanced.gear.is_some(), "gear should have a value");
}

// ── Tick simulation ───────────────────────────────────────────────────────────

#[test]
fn tick_simulation_starts_driving_on_tick_1() {
    let mut state = VirtualCarState::new();
    tick_simulation(&mut state, 1);
    assert_eq!(state.speed, Some(16));
    assert_eq!(state.is_driving, Some(true));
    assert_eq!(state.gear, Some(2));
}

#[test]
fn tick_simulation_parks_after_rampdown() {
    let mut state = VirtualCarState::new();
    tick_simulation(&mut state, 15);
    assert_eq!(state.speed, Some(0));
    assert_eq!(state.is_driving, Some(false));
    assert_eq!(state.gear, Some(0));
}

#[test]
fn tick_simulation_increments_odometer_at_end_of_cycle() {
    let mut state = VirtualCarState::new();
    // Tick 18: still in parked phase, no increment yet
    tick_simulation(&mut state, 18);
    assert_eq!(state.odometer, Some(0));
    // Tick 19: last tick of the cycle, odometer increments
    tick_simulation(&mut state, 19);
    assert_eq!(state.odometer, Some(1));
}

#[test]
fn door_lock_not_overwritten_by_simulation() {
    let mut state = VirtualCarState::new();
    state.are_doors_locked = Some(true);
    tick_simulation(&mut state, 5); // cruising tick
    assert_eq!(
        state.are_doors_locked,
        Some(true),
        "simulation must not touch are_doors_locked"
    );
}

#[test]
fn tick_simulation_wraps_correctly_on_second_cycle() {
    let mut state = VirtualCarState::new();
    // Tick 20 == same as tick 0 (phase 0): speed should be 0
    tick_simulation(&mut state, 20);
    assert_eq!(state.speed, Some(0));
    // Tick 21 == same as tick 1: speed = 16
    tick_simulation(&mut state, 21);
    assert_eq!(state.speed, Some(16));
}
