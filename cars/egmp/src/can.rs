#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(clippy::all)]
#![allow(unexpected_cfgs)]
#![allow(unused_imports)]

include!(concat!(env!("OUT_DIR"), "/messages.rs"));

use embedded_can::{Id, StandardId};
use core_interface::{CAN_RX_CHANNEL, CAN_TX_CHANNEL, CanFilter, CanFrame};

// ── CAN Configuration ─────────────────────────────────────────────────────────

pub const CAN_FILTERS: &[CanFilter] = &[
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1041).unwrap()), mask: 0x7FF }, // BODY_STATUS
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(428).unwrap()), mask: 0x7FF },  // CLUSTER_INFO428
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(938).unwrap()), mask: 0x7FF },  // CHARGE_PORT_DOOR
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1044).unwrap()), mask: 0x7FF }, // TRUNK_STATUS
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(764).unwrap()), mask: 0x7FF },  // BATTERY_INFO764
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(550).unwrap()), mask: 0x7FF },  // AMBIENT_TEMPERATURE
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(551).unwrap()), mask: 0x7FF },  // ODOMETER
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(795).unwrap()), mask: 0x7FF },  // CLIMATE_STATUS
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(685).unwrap()), mask: 0x7FF },  // BMS_PRECOND_STATUS
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1077).unwrap()), mask: 0x7FF }, // POWER_CAN_STATUS
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(896).unwrap()), mask: 0x7FF },  // CLIMATE_TEMPERATURE
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(782).unwrap()), mask: 0x7FF },  // CHARGING_POWER
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1090).unwrap()), mask: 0x7FF }, // BCM12200ms
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1050).unwrap()), mask: 0x7FF }, // BCM07200ms
];

// ── Vehicle Tasks ─────────────────────────────────────────────────────────────

use crate::{encode_state, CAR_STATE};
use core_interface::VEHICLE_STATE_CHANNEL;

pub async fn handle_can_frame(frame: CanFrame) {
    let id = match frame.id {
        Id::Standard(id) => id.as_raw() as u32,
        Id::Extended(id) => id.as_raw(),
    };

    if let Ok(message) = m_can::Messages::from_can_message(id, &frame.data[..frame.dlc as usize]) {
        let mut state = CAR_STATE.lock().await;
        let mut updated = false;

        match message {
            m_can::Messages::BodyStatus(m) => {
                state.are_doors_locked = Some(m.doors_unlocked() == 0);
                state.is_driver_door_open = Some(m.driver_door_open());
                state.is_passenger_door_open = Some(m.passenger_door_open());
                state.is_rear_left_door_open = Some(m.rear_left_door_open());
                state.is_rear_right_door_open = Some(m.rear_right_door_open());
                state.is_frunk_open = Some(m.hood_open() != 0);
                updated = true;
            }
            m_can::Messages::ClusterInfo428(m) => {
                state.speed = Some(m.speed_kph() as i32);
                updated = true;
            }
            m_can::Messages::ChargePortDoor(m) => {
                state.charge_port_state = Some(if m.port_open() != 0 { crate::proto::basic_state::ChargePortState::Open } else { crate::proto::basic_state::ChargePortState::Closed });
                updated = true;
            }
            m_can::Messages::TrunkStatus(m) => {
                state.is_trunk_open = Some(m.trunk_open() != 0);
                updated = true;
            }
            m_can::Messages::BatteryInfo764(m) => {
                state.battery_soc = Some(m.state_of_charge() as u32);
                updated = true;
            }
            m_can::Messages::AmbientTemperature(m) => {
                state.outdoor_temperature = Some(m.outdoor_temperature() as f32);
                updated = true;
            }
            m_can::Messages::Odometer(m) => {
                state.odometer = Some(m.odometer() as u32);
                updated = true;
            }
            m_can::Messages::ClimateStatus(m) => {
                state.is_steering_wheel_heater_on = Some(m.steering_wheel_heating_state());
                state.is_climate_on = Some(m.climate_ac_state());
                updated = true;
            }
            m_can::Messages::BmsPrecondStatus(m) => {
                state.is_preconditioning_enabled = Some(
                    m.battery_precond_state() == m_can::BmsPrecondStatusBatteryPrecondState::On ||
                    m.battery_precond_state() == m_can::BmsPrecondStatusBatteryPrecondState::Preparing
                );
                updated = true;
            }
            m_can::Messages::PowerCanStatus(m) => {
                state.is_driving = Some(m.power_state() == m_can::PowerCanStatusPowerState::CarOn);
                updated = true;
            }
            m_can::Messages::ClimateTemperature(m) => {
                // UNTESTED MAPPING:
                state.target_climate_temperature = Some(m.driver_temp_c_raw() as f32);
                updated = true;
            }
            m_can::Messages::ChargingPower(m) => {
                state.power_flow_watt = Some((m.charging_power_k_w() as f32 * 1000.0) as i32);
                updated = true;
            }
            m_can::Messages::Bcm12200ms(m) => {
                // UNTESTED MAPPING:
                state.is_driver_window_open = Some(m.window_drvr_sd_wdw_sta_bcm() != 0);
                state.is_passenger_window_open = Some(m.window_asst_sd_wdw_sta_bcm() != 0);
                state.is_rear_left_window_open = Some(m.window_rr_lft_wdw_sta_bcm() != 0);
                state.is_rear_right_window_open = Some(m.window_rr_rt_wdw_sta_bcm() != 0);
                updated = true;
            }
            m_can::Messages::Bcm07200ms(m) => {
                // UNTESTED MAPPING:
                state.are_lights_on = Some(m.lamp_hd_lmp_lo_on_req() != 0);
                updated = true;
            }
            // TODO: Unmapped BasicState fields:
            // - indoor_temperature
            // - is_windshield_defrost_on
            // - is_rear_window_defrost_on (There is a Rear_Defrost_State in BO 593 that could be used)
            // - battery_range_km
            // - time_remaining_minutes
            // - are_hazard_lights_on

            // TODO: Unmapped AdvancedState fields:
            // - battery_voltage
            // - instantaneous_power_watt
            // - battery_minimum_temperature
            // - battery_maximum_temperature
            // - gear (BCM_GearPosPSta in BO 1058 is unverified and might just be the parking pawl)
            _ => {}
        }

        if updated {
            let payload = encode_state(&state);
            VEHICLE_STATE_CHANNEL.sender().send(payload).await;
        }
    }
}

#[embassy_executor::task]
pub async fn can_rx_task() {
    let receiver = CAN_RX_CHANNEL.receiver();
    loop {
        handle_can_frame(receiver.receive().await).await;
    }
}

#[allow(dead_code)]
pub(crate) async fn send_can_request(id: StandardId, data: &[u8]) {
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
