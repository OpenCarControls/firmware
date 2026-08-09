#![cfg_attr(not(test), no_std)]
#![allow(unexpected_cfgs)]
#![allow(unused_imports)]
extern crate alloc;

pub mod can;
pub use can::{CAN_FILTERS, can_rx_task};
use alloc::string::String;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use core_interface::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, CMD_RESP_CHANNEL, VEHICLE_STATE_CHANNEL, VehicleStatePayload,
};
use prost::Message;

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/opencar.cars.egmp.v1.rs"));
}

// ── Vehicle State ─────────────────────────────────────────────────────────────

pub struct EgmpCarState {
    pub odometer: Option<u32>,
    pub is_driving: Option<bool>,
    pub speed: Option<i32>,
    pub outdoor_temperature: Option<f32>,
    pub indoor_temperature: Option<f32>,
    pub is_climate_on: Option<bool>,
    pub target_climate_temperature: Option<f32>,
    pub is_windshield_defrost_on: Option<bool>,
    pub is_rear_window_defrost_on: Option<bool>,
    pub is_steering_wheel_heater_on: Option<bool>,
    pub battery_soc: Option<u32>,
    pub battery_range_km: Option<u32>,
    pub charge_port_state: Option<proto::basic_state::ChargePortState>,
    pub power_flow_watt: Option<i32>,
    pub time_remaining_minutes: Option<u32>,
    pub are_doors_locked: Option<bool>,
    pub is_driver_door_open: Option<bool>,
    pub is_driver_window_open: Option<bool>,
    pub is_passenger_door_open: Option<bool>,
    pub is_passenger_window_open: Option<bool>,
    pub is_rear_left_door_open: Option<bool>,
    pub is_rear_left_window_open: Option<bool>,
    pub is_rear_right_door_open: Option<bool>,
    pub is_rear_right_window_open: Option<bool>,
    pub is_frunk_open: Option<bool>,
    pub is_trunk_open: Option<bool>,
    pub are_lights_on: Option<bool>,
    pub are_hazard_lights_on: Option<bool>,
    pub lights_flash_timestamp: Option<u64>,

    pub battery_voltage: Option<u32>,
    pub instantaneous_power_watt: Option<i32>,
    pub battery_minimum_temperature: Option<f32>,
    pub battery_maximum_temperature: Option<f32>,
    pub is_preconditioning_enabled: Option<bool>,
    pub gear: Option<proto::advanced_state::Gear>,
}

impl EgmpCarState {
    pub const fn new() -> Self {
        Self {
            odometer: Some(0),
            is_driving: Some(false),
            speed: Some(0),
            outdoor_temperature: Some(20.0),
            indoor_temperature: Some(22.0),
            is_climate_on: Some(false),
            target_climate_temperature: Some(22.0),
            is_windshield_defrost_on: Some(false),
            is_rear_window_defrost_on: Some(false),
            is_steering_wheel_heater_on: Some(false),
            battery_soc: Some(100),
            battery_range_km: Some(400),
            charge_port_state: Some(proto::basic_state::ChargePortState::Unspecified),
            power_flow_watt: Some(0),
            time_remaining_minutes: Some(0),
            are_doors_locked: Some(false),
            is_driver_door_open: Some(false),
            is_driver_window_open: Some(false),
            is_passenger_door_open: Some(false),
            is_passenger_window_open: Some(false),
            is_rear_left_door_open: Some(false),
            is_rear_left_window_open: Some(false),
            is_rear_right_door_open: Some(false),
            is_rear_right_window_open: Some(false),
            is_frunk_open: Some(false),
            is_trunk_open: Some(false),
            are_lights_on: Some(false),
            are_hazard_lights_on: Some(false),
            lights_flash_timestamp: Some(0),

            battery_voltage: Some(400),
            instantaneous_power_watt: Some(0),
            battery_minimum_temperature: Some(25.0),
            battery_maximum_temperature: Some(26.0),
            is_preconditioning_enabled: Some(false),
            gear: Some(proto::advanced_state::Gear::Unspecified),
        }
    }
}

pub(crate) static CAR_STATE: Mutex<CriticalSectionRawMutex, EgmpCarState> = Mutex::new(EgmpCarState::new());

pub fn encode_state(state: &EgmpCarState) -> VehicleStatePayload {
    let basic = proto::BasicState {
        odometer: state.odometer,
        is_driving: state.is_driving,
        speed: state.speed,
        outdoor_temperature: state.outdoor_temperature,
        indoor_temperature: state.indoor_temperature,
        is_climate_on: state.is_climate_on,
        target_climate_temperature: state.target_climate_temperature,
        is_windshield_defrost_on: state.is_windshield_defrost_on,
        is_rear_window_defrost_on: state.is_rear_window_defrost_on,
        is_steering_wheel_heater_on: state.is_steering_wheel_heater_on,
        battery_soc: state.battery_soc,
        battery_range_km: state.battery_range_km,
        charge_port_state: state.charge_port_state.map(|s| s as i32),
        power_flow_watt: state.power_flow_watt,
        time_remaining_minutes: state.time_remaining_minutes,
        are_doors_locked: state.are_doors_locked,
        is_driver_door_open: state.is_driver_door_open,
        is_driver_window_open: state.is_driver_window_open,
        is_passenger_door_open: state.is_passenger_door_open,
        is_passenger_window_open: state.is_passenger_window_open,
        is_rear_left_door_open: state.is_rear_left_door_open,
        is_rear_left_window_open: state.is_rear_left_window_open,
        is_rear_right_door_open: state.is_rear_right_door_open,
        is_rear_right_window_open: state.is_rear_right_window_open,
        is_frunk_open: state.is_frunk_open,
        is_trunk_open: state.is_trunk_open,
        are_lights_on: state.are_lights_on,
        are_hazard_lights_on: state.are_hazard_lights_on,
        lights_flash_timestamp: state.lights_flash_timestamp,
    };
    let advanced = proto::AdvancedState {
        battery_voltage: state.battery_voltage,
        instantaneous_power_watt: state.instantaneous_power_watt,
        battery_minimum_temperature: state.battery_minimum_temperature,
        battery_maximum_temperature: state.battery_maximum_temperature,
        is_preconditioning_enabled: state.is_preconditioning_enabled,
        gear: state.gear.map(|g| g as i32),
    };
    VehicleStatePayload {
        basic: basic.encode_to_vec(),
        advanced: advanced.encode_to_vec(),
    }
}



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
        Some(proto::basic_command::Action::DoorLockCommand(_)) => {
            Err("Not implemented")
        }
        Some(proto::basic_command::Action::ChargePortCommand(_)) => {
            Err("Not implemented")
        }
        Some(proto::basic_command::Action::FlashLightsCommand(_)) => {
            Err("Not implemented")
        }
        Some(proto::basic_command::Action::ClimateControlCommand(_)) => {
            Err("Not implemented")
        }
        Some(proto::basic_command::Action::WindowCommand(_)) => {
            Err("Not implemented")
        }
        None => Err("No action in BasicCommand"),
    }
}

#[embassy_executor::task]
pub async fn handle_advanced_commands_task() {
    loop {
        let inbound = ADVANCED_CMD_CHANNEL.receiver().receive().await;
        let (success, error_message) =
            match process_advanced_command(inbound.bytes.as_slice()).await {
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

pub async fn process_advanced_command(bytes: &[u8]) -> Result<(), &'static str> {
    let cmd =
        proto::AdvancedCommand::decode(bytes).map_err(|_| "Failed to decode AdvancedCommand")?;
    match cmd.action {
        Some(proto::advanced_command::Action::BatteryPreconditioningCommand(_)) => {
            Err("Not implemented")
        }
        None => Err("No action in AdvancedCommand"),
    }
}

/// Advances the simulated vehicle state by one 5-second tick.
#[cfg(debug_assertions)]
pub fn tick_simulation(state: &mut EgmpCarState, tick: u64) {
    let phase = (tick % 100) as i32;

    // Base state for each tick
    state.speed = Some(0);
    state.is_driving = Some(false);
    state.gear = Some(proto::advanced_state::Gear::Unspecified);
    state.charge_port_state = Some(proto::basic_state::ChargePortState::Unspecified);
    state.power_flow_watt = Some(0);
    state.is_driver_door_open = Some(false);
    state.is_passenger_door_open = Some(false);
    state.is_rear_left_door_open = Some(false);
    state.is_rear_right_door_open = Some(false);

    if phase < 15 {
        // 0-14: Parked, doors sequence
        if phase > 1 && phase < 13 { state.is_driver_door_open = Some(true); }
        if phase > 3 && phase < 11 { state.is_passenger_door_open = Some(true); }
        if phase > 5 && phase < 9 { 
            state.is_rear_left_door_open = Some(true); 
            state.is_rear_right_door_open = Some(true);
        }
    } else if phase < 50 {
        // 15-49: Driving
        let drive_phase = phase - 15;
        let speed = match drive_phase {
            0..=4 => drive_phase * 16,
            5..=25 => 80,
            26..=30 => (30 - drive_phase) * 16,
            _ => 0,
        };
        state.speed = Some(speed);
        state.is_driving = Some(speed > 0);
        state.gear = Some(if speed > 0 { proto::advanced_state::Gear::Drive } else { proto::advanced_state::Gear::Unspecified });
        
        if speed > 0 {
            state.power_flow_watt = Some(-25000); // Discharging 25kW
            if drive_phase % 4 == 0 {
                let soc = state.battery_soc.unwrap_or(100);
                if soc > 0 {
                    state.battery_soc = Some(soc - 1);
                }
            }
        }
        
        if drive_phase == 34 {
            state.odometer = Some(state.odometer.unwrap_or(0).saturating_add(2));
        }
    } else if phase < 80 {
        // 50-79: Charging
        let charge_phase = phase - 50;
        if charge_phase < 0 {
            state.charge_port_state = Some(proto::basic_state::ChargePortState::Closed);
        } else if charge_phase < 1 {
            state.charge_port_state = Some(proto::basic_state::ChargePortState::Open);
        } else if charge_phase < 2 {
            state.charge_port_state = Some(proto::basic_state::ChargePortState::DcConnected);
        } else if charge_phase < 28 {
            state.charge_port_state = Some(proto::basic_state::ChargePortState::DcCharging);
            state.power_flow_watt = Some(50000); // Charging at 50kW
            if charge_phase % 2 == 0 {
                let soc = state.battery_soc.unwrap_or(0);
                if soc < 100 {
                    state.battery_soc = Some(soc + 1);
                }
            }
        } else if charge_phase < 29 {
            state.charge_port_state = Some(proto::basic_state::ChargePortState::DcConnected);
        } else {
            state.charge_port_state = Some(proto::basic_state::ChargePortState::Open)
        }
    } else {
        // 80-99: Idle
        state.charge_port_state = Some(proto::basic_state::ChargePortState::Closed);
    }
}

#[embassy_executor::task]
pub async fn state_update_task() {
    let mut tick: u64 = 0;
    loop {
        Timer::after(Duration::from_secs(5)).await;
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

    #[test]
    fn encode_state_default_state_has_fields_present() {
        let state = EgmpCarState::new();
        let payload = encode_state(&state);
        let basic = proto::BasicState::decode(payload.basic.as_slice()).unwrap();
        let advanced = proto::AdvancedState::decode(payload.advanced.as_slice()).unwrap();
        
        assert_eq!(basic.odometer, Some(0));
        assert_eq!(basic.is_driving, Some(false));
        assert_eq!(basic.speed, Some(0));
        assert_eq!(basic.battery_soc, Some(100));
        assert_eq!(basic.are_doors_locked, Some(false));

        assert_eq!(advanced.battery_voltage, Some(400));
        assert_eq!(advanced.gear, Some(0));
    }
}
