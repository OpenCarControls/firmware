#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(clippy::all)]
#![allow(unexpected_cfgs)]
#![allow(unused_imports)]

include!(concat!(env!("OUT_DIR"), "/messages.rs"));

use embedded_can::{Id, StandardId};
use core_interface::{CAN_RX_CHANNEL, CanFilter, CanFrame};

// ── CAN Configuration ─────────────────────────────────────────────────────────

// Signals whose DBC mapping has not been verified on a real car are only decoded when the
// `untested-signals` feature is on; otherwise their state fields stay `None` and are not
// published to the app.
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
    #[cfg(feature = "untested-signals")]
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(896).unwrap()), mask: 0x7FF },  // CLIMATE_TEMPERATURE
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(782).unwrap()), mask: 0x7FF },  // CHARGING_POWER
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1124).unwrap()), mask: 0x7FF }, // BCM15200ms
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1050).unwrap()), mask: 0x7FF }, // BCM07200ms
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(1043).unwrap()), mask: 0x7FF }, // BLINKERS
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(780).unwrap()), mask: 0x7FF },  // CHARGE_INLET
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(806).unwrap()), mask: 0x7FF },  // CHARGING_STATUS
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(413).unwrap()), mask: 0x7FF },  // CHARGER_SESSION
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(567).unwrap()), mask: 0x7FF },  // BATTERY_INFO567
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(695).unwrap()), mask: 0x7FF },  // CLUSTER_RANGE
    CanFilter { bus_id: 0, id: Id::Standard(StandardId::new(818).unwrap()), mask: 0x7FF },  // CHARGE_TIME_ESTIMATE
];

// ── Vehicle Tasks ─────────────────────────────────────────────────────────────

use crate::{CAR_STATE, STATE_CHANGED};
use crate::proto::basic_state::ChargePortState;

/// `None` for values the DBC doesn't define, so an unknown value never overwrites a known
/// state. CHARGE_PORT_DOOR only reports the door; `ChargeTracker` adds the cable.
fn door_state(port: m_can::ChargePortDoorPortOpen) -> Option<ChargePortState> {
    match port {
        m_can::ChargePortDoorPortOpen::Open => Some(ChargePortState::Open),
        m_can::ChargePortDoorPortOpen::Closed => Some(ChargePortState::Closed),
        m_can::ChargePortDoorPortOpen::_Other(_) => None,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Charger {
    Ac,
    Dc,
}

/// Combines the door, inlet, charging and charger-session frames into one charge port state.
///
/// E-GMP cars try DC first at every plug-in, so CHARGER_SESSION reports DC for the first
/// seconds on an AC charger too. The charger type is only taken once it is certain (AC
/// reported, or a DC sequence stage that an AC charger never reaches) and kept until unplug,
/// because the stage moves on once a DC session ends. Until then the state is `Connected`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ChargeTracker {
    door: Option<ChargePortState>,
    plugged: Option<bool>,
    charging: bool,
    charger: Option<Charger>,
}

impl ChargeTracker {
    const fn new() -> Self {
        Self { door: None, plugged: None, charging: false, charger: None }
    }

    fn on_door(&mut self, port: m_can::ChargePortDoorPortOpen) {
        if let Some(door) = door_state(port) {
            self.door = Some(door);
        }
    }

    fn on_inlet(&mut self, plugged: bool) {
        // A new plug-in starts a new session; an unplug ends it.
        if self.plugged != Some(plugged) {
            self.charger = None;
            self.charging = false;
        }
        self.plugged = Some(plugged);
    }

    fn on_charging(&mut self, charging: bool) {
        self.charging = charging;
    }

    fn on_session(&mut self, charger_type: m_can::ChargerSessionChargerType, dc_stage: u8) {
        if self.plugged == Some(false) {
            return;
        }
        match charger_type {
            m_can::ChargerSessionChargerType::Ac => self.charger = Some(Charger::Ac),
            m_can::ChargerSessionChargerType::Dc if (2..=6).contains(&dc_stage) => self.charger = Some(Charger::Dc),
            _ => {}
        }
    }

    fn state(&self) -> Option<ChargePortState> {
        if self.plugged != Some(true) {
            return self.door;
        }
        Some(match (self.charger, self.charging) {
            (None, _) => ChargePortState::Connected,
            (Some(Charger::Ac), false) => ChargePortState::AcConnected,
            (Some(Charger::Ac), true) => ChargePortState::AcCharging,
            (Some(Charger::Dc), false) => ChargePortState::DcConnected,
            (Some(Charger::Dc), true) => ChargePortState::DcCharging,
        })
    }
}

static CHARGE_TRACKER: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<ChargeTracker>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(ChargeTracker::new()));

/// Battery power in watts from the voltage and current, positive when charging as the proto
/// defines it. The car reports the current positive when discharging.
fn battery_power_watt(voltage: f32, current: f32) -> i32 {
    -(voltage * current) as i32
}

/// `CHARGING_POWER` is only set on DC, so while AC charging the charge port power comes from
/// the battery power instead. That is what reaches the battery, a little under what the
/// charger supplies. Clamped at 0, since a negative `power_flow_watt` means V2L.
fn is_ac_charging(state: &crate::EgmpCarState) -> bool {
    state.charge_port_state == Some(ChargePortState::AcCharging)
}

/// CHARGE_TIME_ESTIMATE is sent all the time as the estimate for a DC fast charge, and its
/// units are unclear on AC, so it is only taken while a DC charger is plugged in.
fn is_dc_plugged_in(state: &crate::EgmpCarState) -> bool {
    matches!(state.charge_port_state, Some(ChargePortState::DcConnected | ChargePortState::DcCharging))
}

/// Last battery voltage and current, which come in separate messages.
static BATTERY: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<(Option<f32>, Option<f32>)>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new((None, None)));

/// Records a new voltage or current and returns the battery power once both are known.
fn track_battery(voltage: Option<f32>, current: Option<f32>) -> Option<i32> {
    BATTERY.lock(|cell| {
        let (mut v, mut i) = cell.get();
        v = voltage.or(v);
        i = current.or(i);
        cell.set((v, i));
        Some(battery_power_watt(v?, i?))
    })
}

/// Applies `update` to the shared tracker and returns the resulting charge port state.
fn track_charge(update: impl FnOnce(&mut ChargeTracker)) -> Option<ChargePortState> {
    CHARGE_TRACKER.lock(|cell| {
        let mut tracker = cell.get();
        update(&mut tracker);
        cell.set(tracker);
        tracker.state()
    })
}

/// Maps a `Window_*DetailOpnSta` raw value from BCM_15_200ms: 0 is closed, 1 to 3 are
/// open by steps. `None` for Not_Available (7), which the EV6 reports for its rear windows.
fn window_open(position: u8) -> Option<bool> {
    match position {
        0 => Some(false),
        1..=3 => Some(true),
        _ => None,
    }
}

pub async fn handle_can_frame(frame: CanFrame) {
    let id = match frame.id {
        Id::Standard(id) => id.as_raw() as u32,
        Id::Extended(id) => id.as_raw(),
    };

    if let Ok(message) = m_can::Messages::from_can_message(id, &frame.data[..frame.dlc as usize]) {
        let mut state = CAR_STATE.lock().await;
        let old_state = state.clone();

        match message {
            m_can::Messages::BodyStatus(m) => {
                state.are_doors_locked = Some(m.doors_unlocked() == 0);
                state.is_driver_door_open = Some(m.driver_door_open());
                state.is_passenger_door_open = Some(m.passenger_door_open());
                state.is_rear_left_door_open = Some(m.rear_left_door_open());
                state.is_rear_right_door_open = Some(m.rear_right_door_open());
                state.is_frunk_open = Some(m.hood_open() != 0);
            }
            m_can::Messages::ClusterInfo428(m) => {
                let speed = m.speed_kph() as i32;
                state.speed = Some(speed);
                state.is_driving = Some(speed > 0);
            }
            m_can::Messages::ChargePortDoor(m) => {
                if let Some(port) = track_charge(|t| t.on_door(m.port_open())) {
                    state.charge_port_state = Some(port);
                }
            }
            m_can::Messages::ChargeInlet(m) => {
                if let Some(port) = track_charge(|t| t.on_inlet(m.plug_connected())) {
                    state.charge_port_state = Some(port);
                }
            }
            m_can::Messages::ChargingStatus(m) => {
                if let Some(port) = track_charge(|t| t.on_charging(m.charging())) {
                    state.charge_port_state = Some(port);
                }
            }
            m_can::Messages::ChargerSession(m) => {
                if let Some(port) = track_charge(|t| t.on_session(m.charger_type(), m.dc_sequence_stage())) {
                    state.charge_port_state = Some(port);
                }
            }
            m_can::Messages::TrunkStatus(m) => {
                state.is_trunk_open = Some(m.trunk_open() != 0);
            }
            m_can::Messages::BatteryInfo764(m) => {
                state.battery_soc = Some(m.state_of_charge() as u32);
                if let Some(power) = track_battery(None, Some(m.battery_current())) {
                    state.instantaneous_power_watt = Some(power);
                    if is_ac_charging(&state) {
                        state.power_flow_watt = Some(power.max(0));
                    }
                }
            }
            m_can::Messages::BatteryInfo567(m) => {
                let voltage = m.battery_voltage();
                state.battery_voltage = Some(voltage as u32);
                if let Some(power) = track_battery(Some(voltage), None) {
                    state.instantaneous_power_watt = Some(power);
                    if is_ac_charging(&state) {
                        state.power_flow_watt = Some(power.max(0));
                    }
                }
            }
            m_can::Messages::AmbientTemperature(m) => {
                state.outdoor_temperature = Some(m.outdoor_temperature() as f32);
            }
            m_can::Messages::Odometer(m) => {
                state.odometer = Some(m.odometer() as u32);
            }
            m_can::Messages::ClimateStatus(m) => {
                state.is_steering_wheel_heater_on = Some(m.steering_wheel_heating_state());
                state.is_climate_on = Some(m.climate_ac_state());
            }
            m_can::Messages::BmsPrecondStatus(m) => {
                state.is_preconditioning_enabled = Some(
                    m.battery_precond_state() == m_can::BmsPrecondStatusBatteryPrecondState::On ||
                    m.battery_precond_state() == m_can::BmsPrecondStatusBatteryPrecondState::Preparing
                );
            }
            #[cfg(feature = "untested-signals")]
            m_can::Messages::ClimateTemperature(m) => {
                // UNTESTED MAPPING:
                state.target_climate_temperature = Some(m.driver_temp_c_raw() as f32);
            }
            m_can::Messages::ClusterRange(m) => {
                // The range for the active drive mode, as the cluster shows it.
                state.battery_range_km = Some(m.range() as u32);
            }
            m_can::Messages::ChargeTimeEstimate(m) => {
                if is_dc_plugged_in(&state) {
                    state.time_remaining_minutes = Some(m.charge_time_to_limit() as u32 / 60);
                }
            }
            m_can::Messages::ChargingPower(m) => {
                if !is_ac_charging(&state) {
                    state.power_flow_watt = Some((m.charging_power_k_w() as f32 * 1000.0) as i32);
                }
            }
            m_can::Messages::Bcm15200ms(m) => {
                state.is_driver_window_open = window_open(m.window_drvr_sd_wdw_detail_opn_sta_raw());
                state.is_passenger_window_open = window_open(m.window_asst_sd_wdw_detail_opn_sta_raw());
                state.is_rear_left_window_open = window_open(m.window_rr_lft_wdw_detail_opn_sta_raw());
                state.is_rear_right_window_open = window_open(m.window_rr_rt_wdw_detail_opn_sta_raw());
            }
            m_can::Messages::Bcm07200ms(m) => {
                state.are_lights_on = Some(m.lamp_hd_lmp_lo_on_req() != 0);
            }
            m_can::Messages::Blinkers(m) => {
                state.are_hazard_lights_on = Some(m.hazard_on());
            }
            // TODO: Unmapped BasicState fields:
            // - is_driving is derived from speed; a power-state signal (POWER_CAN_STATUS,
            //   BO 1077) tells "car on" apart from "moving" if that distinction is needed
            // - indoor_temperature
            // - is_windshield_defrost_on
            // - is_rear_window_defrost_on (There is a Rear_Defrost_State in BO 593 that could be used)
            // - time_remaining_minutes on AC (CHARGE_TIME_ESTIMATE units unclear there) and
            //   when driving (time to empty)
            // - charge_port_state errors, DC negotiating and V2L

            // TODO: Unmapped AdvancedState fields:
            // - battery_minimum_temperature
            // - battery_maximum_temperature
            // - gear (BCM_GearPosPSta in BO 1058 is unverified and might just be the parking pawl)
            _ => {}
        }

        // Publishing is left to `state_update_task`, which rate-limits it; the CAN path
        // must never block on the outbound state channel.
        if *state != old_state {
            STATE_CHANGED.signal(());
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
    core_interface::send_can_frame(CanFrame {
        bus_id: 0,
        id: Id::Standard(id),
        data: buf,
        dlc,
    })
    .await;
}

// ── Unit Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Payloads are real frames from EV6 CAN logs.

    #[test]
    fn charge_port_door_maps_open_and_closed() {
        let open = m_can::ChargePortDoor::try_from(&[0xD3, 0xE0, 0x00, 0x02, 0x52, 0x00, 0x00, 0x00][..]).ok().unwrap();
        let closed = m_can::ChargePortDoor::try_from(&[0xB2, 0x90, 0x00, 0x02, 0x54, 0x00, 0x00, 0x00][..]).ok().unwrap();
        assert_eq!(door_state(open.port_open()), Some(ChargePortState::Open));
        assert_eq!(door_state(closed.port_open()), Some(ChargePortState::Closed));
    }

    #[test]
    fn charge_port_door_ignores_unknown_values() {
        assert_eq!(door_state(m_can::ChargePortDoorPortOpen::_Other(0)), None);
    }

    fn inlet(data: [u8; 8]) -> bool {
        m_can::ChargeInlet::try_from(&data[..]).ok().unwrap().plug_connected()
    }

    fn session(t: &mut ChargeTracker, data: [u8; 8]) {
        let m = m_can::ChargerSession::try_from(&data[..]).ok().unwrap();
        t.on_session(m.charger_type(), m.dc_sequence_stage());
    }

    fn charging(data: [u8; 8]) -> bool {
        m_can::ChargingStatus::try_from(&data[..]).ok().unwrap().charging()
    }

    /// The L2 start capture: the car reports DC first, gives up on it, then reports AC.
    #[test]
    fn ac_charger_shows_connected_until_the_car_reports_ac() {
        let mut t = ChargeTracker::new();
        t.on_door(m_can::ChargePortDoorPortOpen::Open);
        t.on_inlet(inlet([0x00, 0x00, 0x0E, 0x00, 0x2C, 0x00, 0xA0, 0x0F]));
        assert_eq!(t.state(), Some(ChargePortState::Open));

        t.on_inlet(inlet([0x80, 0x1C, 0xB4, 0x00, 0x0F, 0x01, 0xA0, 0x0F]));
        assert_eq!(t.state(), Some(ChargePortState::Connected));
        for frame in [
            [0x00, 0x12, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00],
            [0x00, 0x12, 0x02, 0x14, 0x00, 0x00, 0x00, 0x00],
            [0x00, 0x12, 0x02, 0x94, 0x00, 0x00, 0x00, 0x30],
            [0x00, 0x12, 0x02, 0xB5, 0x00, 0x00, 0x00, 0x50],
            [0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00],
        ] {
            session(&mut t, frame);
            assert_eq!(t.state(), Some(ChargePortState::Connected));
        }

        session(&mut t, [0x00, 0x11, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(t.state(), Some(ChargePortState::AcConnected));
        t.on_charging(charging([0x29, 0x00, 0x00, 0x14, 0x01, 0x00, 0x72, 0x01]));
        assert_eq!(t.state(), Some(ChargePortState::AcCharging));
    }

    /// log_21: a DC session from plug-in to unplug.
    #[test]
    fn dc_charger_is_kept_until_unplug() {
        let mut t = ChargeTracker::new();
        t.on_door(m_can::ChargePortDoorPortOpen::Open);
        t.on_inlet(true);
        session(&mut t, [0x00, 0x12, 0x02, 0x14, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(t.state(), Some(ChargePortState::Connected));
        session(&mut t, [0x00, 0x12, 0x02, 0x24, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(t.state(), Some(ChargePortState::DcConnected));

        t.on_charging(charging([0xB6, 0x10, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00]));
        session(&mut t, [0x00, 0x12, 0x02, 0x64, 0x00, 0x00, 0x00, 0x08]);
        assert_eq!(t.state(), Some(ChargePortState::DcCharging));

        // Charging stops; the ending stages must not drop the charger type.
        t.on_charging(charging([0x92, 0x10, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00]));
        session(&mut t, [0x00, 0x12, 0x02, 0xA4, 0x00, 0x00, 0x00, 0x58]);
        assert_eq!(t.state(), Some(ChargePortState::DcConnected));

        t.on_inlet(false);
        assert_eq!(t.state(), Some(ChargePortState::Open));
        t.on_door(m_can::ChargePortDoorPortOpen::Closed);
        assert_eq!(t.state(), Some(ChargePortState::Closed));
    }

    #[test]
    fn a_new_plug_in_forgets_the_last_charger() {
        let mut t = ChargeTracker::new();
        t.on_inlet(true);
        session(&mut t, [0x00, 0x11, 0x01, 0x04, 0x02, 0x00, 0x00, 0x00]);
        assert_eq!(t.state(), Some(ChargePortState::AcConnected));

        t.on_inlet(false);
        // A late session frame after unplug must not carry over to the next plug-in.
        session(&mut t, [0x00, 0x11, 0x01, 0x04, 0x02, 0x00, 0x00, 0x00]);
        t.on_inlet(true);
        assert_eq!(t.state(), Some(ChargePortState::Connected));
    }

    #[test]
    fn unknown_inlet_state_falls_back_to_the_door() {
        let mut t = ChargeTracker::new();
        assert_eq!(t.state(), None);
        t.on_door(m_can::ChargePortDoorPortOpen::Closed);
        assert_eq!(t.state(), Some(ChargePortState::Closed));
    }

    #[test]
    fn front_windows_decode_and_rear_windows_are_not_available() {
        let closed = m_can::Bcm15200ms::try_from(&[0xF5, 0xE0, 0x00, 0xFF, 0x00, 0x3F, 0x05, 0x00][..]).ok().unwrap();
        assert_eq!(window_open(closed.window_drvr_sd_wdw_detail_opn_sta_raw()), Some(false));
        assert_eq!(window_open(closed.window_asst_sd_wdw_detail_opn_sta_raw()), Some(false));
        assert_eq!(window_open(closed.window_rr_lft_wdw_detail_opn_sta_raw()), None);
        assert_eq!(window_open(closed.window_rr_rt_wdw_detail_opn_sta_raw()), None);

        let driver_open = m_can::Bcm15200ms::try_from(&[0x76, 0x20, 0x01, 0xFF, 0x03, 0x3F, 0x05, 0x00][..]).ok().unwrap();
        assert_eq!(window_open(driver_open.window_drvr_sd_wdw_detail_opn_sta_raw()), Some(true));
        assert_eq!(window_open(driver_open.window_asst_sd_wdw_detail_opn_sta_raw()), Some(false));

        let passenger_open = m_can::Bcm15200ms::try_from(&[0x37, 0xC0, 0x10, 0xFF, 0x18, 0x3F, 0x05, 0x00][..]).ok().unwrap();
        assert_eq!(window_open(passenger_open.window_drvr_sd_wdw_detail_opn_sta_raw()), Some(false));
        assert_eq!(window_open(passenger_open.window_asst_sd_wdw_detail_opn_sta_raw()), Some(true));
    }

    #[test]
    fn battery_voltage_current_and_power() {
        // log_15 at 1:43.6, a hard launch: discharging.
        let v = m_can::BatteryInfo567::try_from(&[0x00, 0x15, 0x65, 0x00, 0x00, 0x07, 0x1B, 0x00][..]).ok().unwrap();
        let i = m_can::BatteryInfo764::try_from(&[0xBC, 0x07, 0x18, 0x1A, 0x00, 0x00, 0x8D, 0x63][..]).ok().unwrap();
        assert!((v.battery_voltage() - 691.9).abs() < 0.05);
        assert!((i.battery_current() - 198.0).abs() < 0.05);
        assert!((battery_power_watt(v.battery_voltage(), i.battery_current()) + 136_996).abs() <= 1);

        // log_21 at 3:30, DC charging at 177-178 kW: negative current, positive power.
        let v = m_can::BatteryInfo567::try_from(&[0x00, 0x25, 0x63, 0x00, 0x00, 0x75, 0x1D, 0x00][..]).ok().unwrap();
        let i = m_can::BatteryInfo764::try_from(&[0xC9, 0xF6, 0x1B, 0x1D, 0x00, 0x00, 0x8C, 0x61][..]).ok().unwrap();
        assert!((i.battery_current() + 235.9).abs() < 0.05);
        assert!((battery_power_watt(v.battery_voltage(), i.battery_current()) - 177_892).abs() <= 1);
    }

    fn feed(id: u16, data: [u8; 8]) {
        let frame = CanFrame { bus_id: 0, id: Id::Standard(StandardId::new(id).unwrap()), data, dlc: 8 };
        embassy_futures::block_on(handle_can_frame(frame));
    }

    /// Frames from the L2 running capture, through the real handler: CHARGING_POWER reads 0
    /// on AC, so the charge port power must come from the battery and not be overwritten.
    #[test]
    fn ac_charging_power_flow_comes_from_the_battery() {
        feed(938, [0x0D, 0xA0, 0x00, 0x02, 0x52, 0x00, 0x00, 0x00]);
        feed(780, [0x80, 0x63, 0x76, 0x00, 0x0F, 0x01, 0xA0, 0x0F]);
        feed(413, [0x00, 0x11, 0x01, 0x04, 0x04, 0x00, 0x00, 0x00]);
        feed(806, [0xCD, 0x31, 0x91, 0x14, 0x01, 0x00, 0x72, 0x01]);
        feed(567, [0x00, 0x05, 0x57, 0x00, 0x00, 0x6E, 0x1B, 0x00]);
        feed(764, [0xE1, 0xFF, 0x10, 0x11, 0x00, 0x00, 0x8D, 0x54]);
        feed(782, [0x00, 0x00, 0x18, 0x05, 0x21, 0x00, 0x00, 0x10]);

        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.charge_port_state, Some(ChargePortState::AcCharging));
        assert_eq!(state.battery_voltage, Some(702));
        // 702.2 V * 3.1 A into the battery.
        let power = state.instantaneous_power_watt.unwrap();
        assert!((power - 2_176).abs() <= 1, "{power}");
        assert_eq!(state.power_flow_watt, Some(power));
    }

    #[test]
    fn cluster_range() {
        // log_03 at 18:13, in Sport: the cluster showed 263 km.
        let m = m_can::ClusterRange::try_from(&[0x07, 0x01, 0x86, 0x24, 0x00, 0x04, 0x00, 0x00][..]).ok().unwrap();
        assert_eq!(m.range(), 263);
    }

    /// log_23 at 1:51: 33 min to 100% on DC. The same frame is sent while driving or on AC,
    /// where it must not be taken.
    #[test]
    fn charge_time_is_only_taken_on_dc() {
        const ESTIMATE: [u8; 8] = [0xBC, 0x07, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00];
        feed(780, [0x00, 0x00, 0x0E, 0x00, 0x2C, 0x00, 0xA0, 0x0F]);
        feed(780, [0x80, 0x1C, 0xB4, 0x00, 0x0F, 0x01, 0xA0, 0x0F]);
        feed(413, [0x00, 0x11, 0x01, 0x04, 0x04, 0x00, 0x00, 0x00]);
        let before = embassy_futures::block_on(CAR_STATE.lock()).time_remaining_minutes;
        feed(818, ESTIMATE);
        assert_eq!(embassy_futures::block_on(CAR_STATE.lock()).time_remaining_minutes, before);

        feed(780, [0x00, 0x00, 0x0E, 0x00, 0x2C, 0x00, 0xA0, 0x0F]);
        feed(780, [0x80, 0x1C, 0xB4, 0x00, 0x0F, 0x01, 0xA0, 0x0F]);
        feed(413, [0x00, 0x12, 0x02, 0x64, 0x00, 0x00, 0x00, 0x08]);
        feed(806, [0xB6, 0x10, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00]);
        feed(818, ESTIMATE);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.charge_port_state, Some(ChargePortState::DcCharging));
        assert_eq!(state.time_remaining_minutes, Some(33));
    }

    #[test]
    fn blinkers_hazard_bit() {
        let on = m_can::Blinkers::try_from(&[0xCD, 0x65, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00][..]).ok().unwrap();
        let left_only = m_can::Blinkers::try_from(&[0xF0, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00][..]).ok().unwrap();
        assert!(on.hazard_on());
        assert!(!left_only.hazard_on());
        assert!(left_only.left_stalk());
    }
}
