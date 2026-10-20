#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(clippy::all)]
#![allow(unexpected_cfgs)]
#![allow(unused_imports)]

include!(concat!(env!("OUT_DIR"), "/messages.rs"));

use core_interface::{CAN_RX_CHANNEL, CanFilter, CanFrame};
use embedded_can::{Id, StandardId};

// ── CAN Configuration ─────────────────────────────────────────────────────────

// Only signals checked on a real car are decoded (the DBC comments say how each was checked),
// so the app never gets a guessed value.
pub const CAN_FILTERS: &[CanFilter] = &[
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1041).unwrap()),
        mask: 0x7FF,
    }, // BODY_STATUS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(428).unwrap()),
        mask: 0x7FF,
    }, // CLUSTER_INFO428
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(938).unwrap()),
        mask: 0x7FF,
    }, // CHARGE_PORT_DOOR
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1044).unwrap()),
        mask: 0x7FF,
    }, // TRUNK_STATUS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(764).unwrap()),
        mask: 0x7FF,
    }, // BATTERY_INFO764
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(550).unwrap()),
        mask: 0x7FF,
    }, // AMBIENT_TEMPERATURE
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(551).unwrap()),
        mask: 0x7FF,
    }, // ODOMETER
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(685).unwrap()),
        mask: 0x7FF,
    }, // BMS_PRECOND_STATUS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(896).unwrap()),
        mask: 0x7FF,
    }, // CLIMATE_TEMPERATURE
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(899).unwrap()),
        mask: 0x7FF,
    }, // DATC_06_200ms
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1048).unwrap()),
        mask: 0x7FF,
    }, // ICU_08_200ms
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(782).unwrap()),
        mask: 0x7FF,
    }, // CHARGING_POWER
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1124).unwrap()),
        mask: 0x7FF,
    }, // BCM15200ms
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1050).unwrap()),
        mask: 0x7FF,
    }, // BCM07200ms
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1043).unwrap()),
        mask: 0x7FF,
    }, // BLINKERS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(780).unwrap()),
        mask: 0x7FF,
    }, // CHARGE_INLET
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(806).unwrap()),
        mask: 0x7FF,
    }, // CHARGING_STATUS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(413).unwrap()),
        mask: 0x7FF,
    }, // CHARGER_SESSION
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(567).unwrap()),
        mask: 0x7FF,
    }, // BATTERY_INFO567
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(695).unwrap()),
        mask: 0x7FF,
    }, // CLUSTER_RANGE
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(818).unwrap()),
        mask: 0x7FF,
    }, // CHARGE_TIME_ESTIMATE
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(504).unwrap()),
        mask: 0x7FF,
    }, // CHARGING_TIME
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(337).unwrap()),
        mask: 0x7FF,
    }, // CHARGE_LIMITS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(435).unwrap()),
        mask: 0x7FF,
    }, // CABIN_TEMPERATURE
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(898).unwrap()),
        mask: 0x7FF,
    }, // DATC_05_200ms
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(54).unwrap()),
        mask: 0x7FF,
    }, // DRIVE_MODE_STATUS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(992).unwrap()),
        mask: 0x7FF,
    }, // BCM_02_200ms
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1174).unwrap()),
        mask: 0x7FF,
    }, // DRIVER_SEAT_HEATING
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(1141).unwrap()),
        mask: 0x7FF,
    }, // PASSENGER_SEAT_HEATING
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(427).unwrap()),
        mask: 0x7FF,
    }, // CLUSTER_DISPLAY_UNITS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(738).unwrap()),
        mask: 0x7FF,
    }, // REGEN_STATUS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(739).unwrap()),
        mask: 0x7FF,
    }, // I_PEDAL_STATUS
    CanFilter {
        bus_id: 0,
        id: Id::Standard(StandardId::new(929).unwrap()),
        mask: 0x7FF,
    }, // TIRE_PRESSURES
];

// ── Vehicle Tasks ─────────────────────────────────────────────────────────────

use crate::proto::basic_state::ChargePortState;
use crate::{CAR_STATE, STATE_CHANGED};

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
/// On DC, stages 2 to 6 without energy flowing are the handshake with the charger.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ChargeTracker {
    door: Option<ChargePortState>,
    plugged: Option<bool>,
    charging: bool,
    charger: Option<Charger>,
    dc_stage: u8,
}

impl ChargeTracker {
    const fn new() -> Self {
        Self {
            door: None,
            plugged: None,
            charging: false,
            charger: None,
            dc_stage: 0,
        }
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
            self.dc_stage = 0;
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
        self.dc_stage = dc_stage;
        match charger_type {
            m_can::ChargerSessionChargerType::Ac => self.charger = Some(Charger::Ac),
            m_can::ChargerSessionChargerType::Dc if (2..=6).contains(&dc_stage) => {
                self.charger = Some(Charger::Dc)
            }
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
            (Some(Charger::Dc), false) if (2..=6).contains(&self.dc_stage) => {
                ChargePortState::DcNegotiating
            }
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

/// CHARGE_TIME_ESTIMATE is the car's estimate for a DC fast charge and is sent all the time,
/// also on AC, so its time to 80% is only taken while a DC charger is plugged in.
fn is_dc_plugged_in(state: &crate::EgmpCarState) -> bool {
    matches!(
        state.charge_port_state,
        Some(ChargePortState::DcConnected | ChargePortState::DcCharging)
    )
}

/// "On" is when the battery heater switches on (and the cluster shows the battery icon), about
/// a minute after the request; the delay varies. Anything else, like the 2 sent while the car
/// is off, asleep or charging, is unknown rather than off.
fn preconditioning_state(
    raw: m_can::BmsPrecondStatusBatteryPrecondState,
) -> crate::proto::advanced_state::PreconditioningState {
    use crate::proto::advanced_state::PreconditioningState;
    match raw {
        m_can::BmsPrecondStatusBatteryPrecondState::Off => PreconditioningState::Off,
        m_can::BmsPrecondStatusBatteryPrecondState::Preparing => PreconditioningState::Preparing,
        m_can::BmsPrecondStatusBatteryPrecondState::On => PreconditioningState::Heating,
        m_can::BmsPrecondStatusBatteryPrecondState::_Other(_) => PreconditioningState::Unspecified,
    }
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

/// `None` for values not seen on a real car, so they never overwrite the last known gear.
fn gear(raw: m_can::DriveModeStatusGear) -> Option<crate::proto::advanced_state::Gear> {
    use crate::proto::advanced_state::Gear;
    match raw {
        m_can::DriveModeStatusGear::Park => Some(Gear::Park),
        m_can::DriveModeStatusGear::Reverse => Some(Gear::Reverse),
        m_can::DriveModeStatusGear::Neutral => Some(Gear::Neutral),
        m_can::DriveModeStatusGear::Drive => Some(Gear::Drive),
        m_can::DriveModeStatusGear::_Other(_) => None,
    }
}

/// The start button steps the BCM inputs through off, accessory and on; ready comes from
/// DRIVE_MODE_STATUS, since the ignition inputs read the same when on and ready.
fn power_state(
    accessory: bool,
    ignition: bool,
    ready: bool,
) -> crate::proto::advanced_state::PowerState {
    use crate::proto::advanced_state::PowerState;
    if ready {
        PowerState::Ready
    } else if ignition {
        PowerState::On
    } else if accessory {
        PowerState::Accessory
    } else {
        PowerState::Off
    }
}

/// Last (accessory, ignition) inputs and ready bit, which come in separate messages.
static POWER: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<(Option<(bool, bool)>, Option<bool>)>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new((None, None)));

/// Records new ignition inputs or a new ready bit and returns the power state once both are known.
fn track_power(
    inputs: Option<(bool, bool)>,
    ready: Option<bool>,
) -> Option<crate::proto::advanced_state::PowerState> {
    POWER.lock(|cell| {
        let (mut i, mut r) = cell.get();
        i = inputs.or(i);
        r = ready.or(r);
        cell.set((i, r));
        let (accessory, ignition) = i?;
        Some(power_state(accessory, ignition, r?))
    })
}

/// Auto takes precedence: the i-Pedal bit reads 0 in Auto, but checking Auto first keeps the
/// mode right if it ever lags behind.
fn regen_mode(auto: bool, i_pedal: bool) -> crate::RegenMode {
    use crate::RegenMode;
    if auto {
        RegenMode::Auto
    } else if i_pedal {
        RegenMode::IPedal
    } else {
        RegenMode::Manual
    }
}

/// Last Auto bit and i-Pedal bit, which come in separate messages.
static REGEN: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<(Option<bool>, Option<bool>)>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new((None, None)));

/// Records a new Auto or i-Pedal bit and returns the regen mode once both are known.
fn track_regen(auto: Option<bool>, i_pedal: Option<bool>) -> Option<crate::RegenMode> {
    REGEN.lock(|cell| {
        let (mut a, mut p) = cell.get();
        a = auto.or(a);
        p = i_pedal.or(p);
        cell.set((a, p));
        Some(regen_mode(a?, p?))
    })
}

/// The car's level is a 0-4 strength that moves smoothly between the steps in Auto, so it is
/// rounded to the nearest step. `None` for Not_Available (car off, smart cruise driving).
fn regen_level(raw: m_can::RegenStatusRegenLevel) -> Option<u32> {
    match raw {
        m_can::RegenStatusRegenLevel::_Other(level) if level <= 4.0 => Some((level + 0.5) as u32),
        _ => None,
    }
}

const KPA_PER_PSI: f32 = 6.894_757;

/// Maps a `Tire_Pressure_*` raw value in whole psi. `None` for Not_Available (255), sent until
/// the tire sensors report a minute or two into a drive, so the last known pressure is kept.
fn tire_pressure_kpa(psi: u8) -> Option<f32> {
    match psi {
        255 => None,
        psi => Some(psi as f32 * KPA_PER_PSI),
    }
}

/// Maps a `*_Seat_Heating_Oper` raw value: 2 is off, 6 to 8 are levels 1 to 3. `None` for
/// values not seen on a real car (3 to 5 may be ventilation), so they keep the last state.
fn seat_climate(raw: u8) -> Option<crate::proto::basic_state::SeatClimate> {
    use crate::proto::basic_state::SeatClimate;
    match raw {
        2 => Some(SeatClimate::Off),
        6 => Some(SeatClimate::HeatLevel1),
        7 => Some(SeatClimate::HeatLevel2),
        8 => Some(SeatClimate::HeatLevel3),
        _ => None,
    }
}

/// Maps a `Window_*DetailOpnSta` raw value from BCM_15_200ms: 0 is closed, 1 to 3 are
/// open by steps. `None` for Not_Available (7), which the EV6 reports for its rear windows.
fn window_position(raw: u8) -> Option<crate::proto::basic_state::WindowPosition> {
    use crate::proto::basic_state::WindowPosition;
    match raw {
        0 => Some(WindowPosition::Closed),
        1 => Some(WindowPosition::SlightlyOpen),
        2 => Some(WindowPosition::PartlyOpen),
        3 => Some(WindowPosition::FullyOpen),
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
                state.speed = Some(m.speed_kph() as i32);
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
                if let Some(port) =
                    track_charge(|t| t.on_session(m.charger_type(), m.dc_sequence_stage()))
                {
                    state.charge_port_state = Some(port);
                }
            }
            m_can::Messages::TrunkStatus(m) => {
                state.is_trunk_open = Some(m.trunk_open() != 0);
            }
            m_can::Messages::BatteryInfo764(m) => {
                state.battery_soc = Some(m.state_of_charge() as u32);
                state.battery_minimum_temperature = Some(m.battery_min_temperature() as f32);
                state.battery_maximum_temperature = Some(m.battery_max_temperature() as f32);
                // Not_Available for a frame while the car switches off.
                if let m_can::BatteryInfo764AuxBatteryVoltage::_Other(volts) =
                    m.aux_battery_voltage()
                {
                    state.auxiliary_battery_voltage = Some(volts);
                }
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
                // Not_Available while the car displays Fahrenheit; ODOMETER has it then.
                if let m_can::AmbientTemperatureOutdoorTemperature::_Other(celsius) =
                    m.outdoor_temperature()
                {
                    state.outdoor_temperature = Some(celsius);
                }
            }
            m_can::Messages::Odometer(m) => {
                state.odometer = Some(m.odometer() as u32);
                if let m_can::OdometerOutdoorTemperatureF::_Other(fahrenheit) =
                    m.outdoor_temperature_f()
                {
                    state.outdoor_temperature = Some((fahrenheit - 32.0) / 1.8);
                }
            }
            m_can::Messages::CabinTemperature(m) => {
                state.indoor_temperature = Some(m.cabin_temperature());
            }
            m_can::Messages::Datc05200ms(m) => {
                use crate::proto::basic_state::TemperatureUnit;
                match m.datc_temp_disp_unit_sta() {
                    m_can::Datc05200msDatcTempDispUnitSta::Celsius => {
                        state.temperature_unit = Some(TemperatureUnit::Celsius)
                    }
                    m_can::Datc05200msDatcTempDispUnitSta::Fahrenheit => {
                        state.temperature_unit = Some(TemperatureUnit::Fahrenheit)
                    }
                    _ => {}
                }
            }
            m_can::Messages::ChargeLimits(m) => {
                state.ac_charge_limit_percent = Some(m.ac_charge_limit() as u32);
                state.dc_charge_limit_percent = Some(m.dc_charge_limit() as u32);
            }
            m_can::Messages::Datc06200ms(m) => {
                // Not_Available with the car off: keep the last known state.
                match m.datc_op_sta() {
                    m_can::Datc06200msDatcOpSta::On => state.is_climate_on = Some(true),
                    m_can::Datc06200msDatcOpSta::Off => state.is_climate_on = Some(false),
                    _ => {}
                }
                match m.datc_fr_dfg_sta() {
                    m_can::Datc06200msDatcFrDfgSta::On => {
                        state.is_windshield_defrost_on = Some(true)
                    }
                    m_can::Datc06200msDatcFrDfgSta::Off => {
                        state.is_windshield_defrost_on = Some(false)
                    }
                    _ => {}
                }
            }
            m_can::Messages::Icu08200ms(m) => match m.hsws_sys_wrng_sta_icu() {
                m_can::Icu08200msHswsSysWrngStaIcu::On => {
                    state.is_steering_wheel_heater_on = Some(true)
                }
                m_can::Icu08200msHswsSysWrngStaIcu::Off => {
                    state.is_steering_wheel_heater_on = Some(false)
                }
                _ => {}
            },
            m_can::Messages::BmsPrecondStatus(m) => {
                state.preconditioning_state =
                    Some(preconditioning_state(m.battery_precond_state()));
            }
            m_can::Messages::ClimateTemperature(m) => {
                // OFF is also sent with the car off; keep the last set temperature then.
                if let m_can::ClimateTemperatureDriverTempC::_Other(celsius) = m.driver_temp_c() {
                    state.target_climate_temperature = Some(celsius);
                }
            }
            m_can::Messages::ClusterRange(m) => {
                // The range for the active drive mode, as the cluster shows it.
                state.battery_range_km = Some(m.range() as u32);
            }
            m_can::Messages::ChargingTime(m) => {
                // The charge in progress, AC or DC; 0 when not charging.
                state.time_remaining_minutes = Some(m.charging_time_to_limit() as u32);
            }
            m_can::Messages::ChargeTimeEstimate(m) => {
                if is_dc_plugged_in(&state) {
                    state.time_to_80_percent_minutes = Some(m.charge_time_to_80() as u32 / 60);
                }
            }
            m_can::Messages::ChargingPower(m) => {
                if !is_ac_charging(&state) {
                    state.power_flow_watt = Some((m.charging_power_k_w() as f32 * 1000.0) as i32);
                }
            }
            m_can::Messages::Bcm15200ms(m) => {
                state.driver_window_position =
                    window_position(m.window_drvr_sd_wdw_detail_opn_sta_raw());
                state.passenger_window_position =
                    window_position(m.window_asst_sd_wdw_detail_opn_sta_raw());
                state.rear_left_window_position =
                    window_position(m.window_rr_lft_wdw_detail_opn_sta_raw());
                state.rear_right_window_position =
                    window_position(m.window_rr_rt_wdw_detail_opn_sta_raw());
            }
            m_can::Messages::Bcm07200ms(m) => {
                state.are_lights_on = Some(m.lamp_hd_lmp_lo_on_req() != 0);
            }
            m_can::Messages::Blinkers(m) => {
                state.are_hazard_lights_on = Some(m.hazard_on());
            }
            m_can::Messages::DriveModeStatus(m) => {
                // Park with the car off, asleep or charging.
                if let Some(gear) = gear(m.gear()) {
                    state.gear = Some(gear);
                }
                // Driving includes stops in traffic, so it doesn't come from the speed.
                state.is_driving =
                    Some(m.ready() && state.gear != Some(crate::proto::advanced_state::Gear::Park));
                if let Some(power) = track_power(None, Some(m.ready())) {
                    state.power_state = Some(power);
                }
            }
            m_can::Messages::DriverSeatHeating(m) => {
                if let Some(seat) = seat_climate(m.driver_seat_heating_oper_raw()) {
                    state.driver_seat_climate = Some(seat);
                }
            }
            m_can::Messages::PassengerSeatHeating(m) => {
                if let Some(seat) = seat_climate(m.passenger_seat_heating_oper_raw()) {
                    state.passenger_seat_climate = Some(seat);
                }
            }
            m_can::Messages::Bcm02200ms(m) => {
                let inputs = (m.bcm_acc_in_sta() == 1, m.bcm_ign1_in_sta() == 1);
                if let Some(power) = track_power(Some(inputs), None) {
                    state.power_state = Some(power);
                }
            }
            m_can::Messages::ClusterDisplayUnits(m) => {
                use crate::proto::basic_state::DistanceUnit;
                state.distance_unit = Some(match m.distance_unit() {
                    m_can::ClusterDisplayUnitsDistanceUnit::Miles => DistanceUnit::Miles,
                    _ => DistanceUnit::Kilometers,
                });
            }
            m_can::Messages::RegenStatus(m) => {
                if let Some(level) = regen_level(m.regen_level()) {
                    state.regen_level = Some(level);
                }
                if let Some(mode) = track_regen(Some(m.regen_auto()), None) {
                    state.regen_mode = Some(mode);
                }
            }
            m_can::Messages::IPedalStatus(m) => {
                if let Some(mode) = track_regen(None, Some(m.i_pedal())) {
                    state.regen_mode = Some(mode);
                }
            }
            m_can::Messages::TirePressures(m) => {
                // Front right and rear right are assumed from the usual order (see the DBC).
                state.front_left_tire_pressure_kpa = tire_pressure_kpa(m.tire_pressure_fl_raw())
                    .or(state.front_left_tire_pressure_kpa);
                state.front_right_tire_pressure_kpa = tire_pressure_kpa(m.tire_pressure_fr_raw())
                    .or(state.front_right_tire_pressure_kpa);
                state.rear_left_tire_pressure_kpa = tire_pressure_kpa(m.tire_pressure_rl_raw())
                    .or(state.rear_left_tire_pressure_kpa);
                state.rear_right_tire_pressure_kpa = tire_pressure_kpa(m.tire_pressure_rr_raw())
                    .or(state.rear_right_tire_pressure_kpa);
            }
            // TODO: Unmapped BasicState fields:
            // - is_rear_window_defrost_on (not on this bus; Rear_Defrost_State in BO 593 is noise)
            // - charge_port_state errors and V2L
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
        let open =
            m_can::ChargePortDoor::try_from(&[0xD3, 0xE0, 0x00, 0x02, 0x52, 0x00, 0x00, 0x00][..])
                .ok()
                .unwrap();
        let closed =
            m_can::ChargePortDoor::try_from(&[0xB2, 0x90, 0x00, 0x02, 0x54, 0x00, 0x00, 0x00][..])
                .ok()
                .unwrap();
        assert_eq!(door_state(open.port_open()), Some(ChargePortState::Open));
        assert_eq!(
            door_state(closed.port_open()),
            Some(ChargePortState::Closed)
        );
    }

    #[test]
    fn charge_port_door_ignores_unknown_values() {
        assert_eq!(door_state(m_can::ChargePortDoorPortOpen::_Other(0)), None);
    }

    fn inlet(data: [u8; 8]) -> bool {
        m_can::ChargeInlet::try_from(&data[..])
            .ok()
            .unwrap()
            .plug_connected()
    }

    fn session(t: &mut ChargeTracker, data: [u8; 8]) {
        let m = m_can::ChargerSession::try_from(&data[..]).ok().unwrap();
        t.on_session(m.charger_type(), m.dc_sequence_stage());
    }

    fn charging(data: [u8; 8]) -> bool {
        m_can::ChargingStatus::try_from(&data[..])
            .ok()
            .unwrap()
            .charging()
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
        assert_eq!(t.state(), Some(ChargePortState::DcNegotiating));
        // log_23: ready at stage 4, then stage 6 arrives 0.2 s before energy flows.
        session(&mut t, [0x00, 0x12, 0x02, 0x44, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(t.state(), Some(ChargePortState::DcNegotiating));
        t.on_charging(charging([0x20, 0x91, 0xBE, 0x10, 0x00, 0x00, 0x00, 0x00]));
        session(&mut t, [0x00, 0x12, 0x02, 0x64, 0x00, 0x00, 0x00, 0x08]);
        assert_eq!(t.state(), Some(ChargePortState::DcNegotiating));

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
        use crate::proto::basic_state::WindowPosition;
        let closed =
            m_can::Bcm15200ms::try_from(&[0xF5, 0xE0, 0x00, 0xFF, 0x00, 0x3F, 0x05, 0x00][..])
                .ok()
                .unwrap();
        assert_eq!(
            window_position(closed.window_drvr_sd_wdw_detail_opn_sta_raw()),
            Some(WindowPosition::Closed)
        );
        assert_eq!(
            window_position(closed.window_asst_sd_wdw_detail_opn_sta_raw()),
            Some(WindowPosition::Closed)
        );
        assert_eq!(
            window_position(closed.window_rr_lft_wdw_detail_opn_sta_raw()),
            None
        );
        assert_eq!(
            window_position(closed.window_rr_rt_wdw_detail_opn_sta_raw()),
            None
        );

        // log_07 from 0:06.57: the driver window opening through each step.
        let driver_steps = [
            (
                [0x3E, 0x00, 0x01, 0xFF, 0x01, 0x3F, 0x05, 0x00],
                WindowPosition::SlightlyOpen,
            ),
            (
                [0x3D, 0x10, 0x01, 0xFF, 0x02, 0x3F, 0x05, 0x00],
                WindowPosition::PartlyOpen,
            ),
            (
                [0x76, 0x20, 0x01, 0xFF, 0x03, 0x3F, 0x05, 0x00],
                WindowPosition::FullyOpen,
            ),
        ];
        for (data, position) in driver_steps {
            let m = m_can::Bcm15200ms::try_from(&data[..]).ok().unwrap();
            assert_eq!(
                window_position(m.window_drvr_sd_wdw_detail_opn_sta_raw()),
                Some(position)
            );
            assert_eq!(
                window_position(m.window_asst_sd_wdw_detail_opn_sta_raw()),
                Some(WindowPosition::Closed)
            );
        }

        let passenger_open =
            m_can::Bcm15200ms::try_from(&[0x37, 0xC0, 0x10, 0xFF, 0x18, 0x3F, 0x05, 0x00][..])
                .ok()
                .unwrap();
        assert_eq!(
            window_position(passenger_open.window_drvr_sd_wdw_detail_opn_sta_raw()),
            Some(WindowPosition::Closed)
        );
        assert_eq!(
            window_position(passenger_open.window_asst_sd_wdw_detail_opn_sta_raw()),
            Some(WindowPosition::FullyOpen)
        );
    }

    #[test]
    fn battery_voltage_current_and_power() {
        // log_15 at 1:43.6, a hard launch: discharging.
        let v =
            m_can::BatteryInfo567::try_from(&[0x00, 0x15, 0x65, 0x00, 0x00, 0x07, 0x1B, 0x00][..])
                .ok()
                .unwrap();
        let i =
            m_can::BatteryInfo764::try_from(&[0xBC, 0x07, 0x18, 0x1A, 0x00, 0x00, 0x8D, 0x63][..])
                .ok()
                .unwrap();
        assert!((v.battery_voltage() - 691.9).abs() < 0.05);
        assert!((i.battery_current() - 198.0).abs() < 0.05);
        assert!(
            (battery_power_watt(v.battery_voltage(), i.battery_current()) + 136_996).abs() <= 1
        );

        // log_21 at 3:30, DC charging at 177-178 kW: negative current, positive power.
        let v =
            m_can::BatteryInfo567::try_from(&[0x00, 0x25, 0x63, 0x00, 0x00, 0x75, 0x1D, 0x00][..])
                .ok()
                .unwrap();
        let i =
            m_can::BatteryInfo764::try_from(&[0xC9, 0xF6, 0x1B, 0x1D, 0x00, 0x00, 0x8C, 0x61][..])
                .ok()
                .unwrap();
        assert!((i.battery_current() + 235.9).abs() < 0.05);
        assert!(
            (battery_power_watt(v.battery_voltage(), i.battery_current()) - 177_892).abs() <= 1
        );
    }

    fn feed(id: u16, data: [u8; 8]) {
        let frame = CanFrame {
            bus_id: 0,
            id: Id::Standard(StandardId::new(id).unwrap()),
            data,
            dlc: 8,
        };
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
        let m =
            m_can::ClusterRange::try_from(&[0x07, 0x01, 0x86, 0x24, 0x00, 0x04, 0x00, 0x00][..])
                .ok()
                .unwrap();
        assert_eq!(m.range(), 263);
    }

    /// The time to the limit comes from CHARGING_TIME for AC and DC; the time to 80% from the
    /// DC estimate, only while a DC charger is plugged in since that estimate is always sent.
    #[test]
    fn charging_times() {
        // log_23 at 1:51: 10 min to 80% in the DC estimate.
        const ESTIMATE: [u8; 8] = [0xBC, 0x07, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00];
        // On L2 (log_30 at 15:05): 135 min to the 80% AC limit, the cluster's "2 h 15".
        feed(780, [0x00, 0x00, 0x0E, 0x00, 0x2C, 0x00, 0xA0, 0x0F]);
        feed(780, [0x80, 0x1C, 0xB4, 0x00, 0x0F, 0x01, 0xA0, 0x0F]);
        feed(413, [0x00, 0x11, 0x01, 0x04, 0x04, 0x00, 0x00, 0x00]);
        feed(504, [0x00, 0x87, 0x00, 0x7D, 0x00, 0x87, 0x00, 0x00]);
        let before = embassy_futures::block_on(CAR_STATE.lock()).time_to_80_percent_minutes;
        feed(818, ESTIMATE);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.time_remaining_minutes, Some(135));
        assert_eq!(state.time_to_80_percent_minutes, before);

        // DC charging.
        feed(780, [0x00, 0x00, 0x0E, 0x00, 0x2C, 0x00, 0xA0, 0x0F]);
        feed(780, [0x80, 0x1C, 0xB4, 0x00, 0x0F, 0x01, 0xA0, 0x0F]);
        feed(413, [0x00, 0x12, 0x02, 0x64, 0x00, 0x00, 0x00, 0x08]);
        feed(806, [0xB6, 0x10, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00]);
        feed(818, ESTIMATE);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.charge_port_state, Some(ChargePortState::DcCharging));
        assert_eq!(state.time_to_80_percent_minutes, Some(10));
    }

    /// log_30: charge limits, cabin temperature, and the outside temperature in both units.
    #[test]
    fn limits_and_temperatures() {
        use crate::proto::basic_state::TemperatureUnit;
        // AC limit 50%, DC limit 100%.
        feed(337, [0xB9, 0xA3, 0x36, 0x64, 0xC8, 0x00, 0x28, 0x09]);
        // Cabin 20 C.
        feed(435, [0xA5, 0x1D, 0x00, 0x03, 0x52, 0x78, 0x9A, 0xAF]);
        // Celsius: 7 C outside.
        feed(898, [0xC5, 0xA8, 0xAA, 0x06, 0x00, 0x04, 0x08, 0x00]);
        feed(550, [0xCD, 0xDE, 0x57, 0x2F, 0x50, 0x51, 0x30, 0x41]);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.ac_charge_limit_percent, Some(50));
        assert_eq!(state.dc_charge_limit_percent, Some(100));
        assert_eq!(state.indoor_temperature, Some(20.0));
        assert_eq!(state.temperature_unit, Some(TemperatureUnit::Celsius));
        assert_eq!(state.outdoor_temperature, Some(7.0));

        // Fahrenheit: AMBIENT_TEMPERATURE reads Not_Available (must not become 215 C) and
        // ODOMETER carries 44.5 F.
        feed(898, [0x66, 0x64, 0xC5, 0x08, 0x00, 0x05, 0x08, 0x00]);
        feed(550, [0xF3, 0x44, 0xDD, 0xFF, 0x4F, 0xD1, 0x30, 0x41]);
        assert_eq!(
            embassy_futures::block_on(CAR_STATE.lock()).outdoor_temperature,
            Some(7.0)
        );
        feed(551, [0x14, 0x30, 0x7F, 0x03, 0x00, 0xF1, 0xA7, 0x02]);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.temperature_unit, Some(TemperatureUnit::Fahrenheit));
        assert!((state.outdoor_temperature.unwrap() - 6.94).abs() < 0.01);
    }

    /// log_10 and the idle captures.
    #[test]
    fn climate_state() {
        // Climate on, windshield defog on (log_10 at 0:39).
        feed(899, [0xF1, 0xF8, 0x03, 0x10, 0x09, 0x00, 0x40, 0x00]);
        // Steering wheel heater on (log_10 at 2:48).
        feed(1048, [0x07, 0xC4, 0x00, 0x00, 0x12, 0x0C, 0x00, 0x00]);
        // Target 19 C (log_10 at 0:30).
        feed(896, [0xCC, 0x62, 0x11, 0x0A, 0x0A, 0x0A, 0x0A, 0x01]);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.is_climate_on, Some(true));
        assert_eq!(state.is_windshield_defrost_on, Some(true));
        assert_eq!(state.is_steering_wheel_heater_on, Some(true));
        assert_eq!(state.target_climate_temperature, Some(19.0));

        // Parked with the climate off, then the car off while charging: the driver zone reads
        // OFF, which must not become 14 C.
        feed(899, [0x28, 0x0C, 0x03, 0x00, 0x02, 0x00, 0x40, 0x00]);
        feed(896, [0xE2, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.is_climate_on, Some(false));
        assert_eq!(state.is_windshield_defrost_on, Some(false));
        assert_eq!(state.target_climate_temperature, Some(19.0));
    }

    /// The manual preconditioning captures, and the car off.
    #[test]
    fn preconditioning_states() {
        use crate::proto::advanced_state::PreconditioningState;
        let state = |data: [u8; 8]| {
            preconditioning_state(
                m_can::BmsPrecondStatus::try_from(&data[..])
                    .ok()
                    .unwrap()
                    .battery_precond_state(),
            )
        };
        assert_eq!(
            state([0x00, 0x41, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F]),
            PreconditioningState::Off
        );
        assert_eq!(
            state([0x00, 0x45, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F]),
            PreconditioningState::Preparing
        );
        assert_eq!(
            state([0x00, 0x55, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F]),
            PreconditioningState::Heating
        );
        assert_eq!(
            state([0x00, 0x42, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F]),
            PreconditioningState::Unspecified
        );
    }

    /// log_09: shifts with the brake held, then the car switched off.
    #[test]
    fn gear_follows_the_shifter() {
        use crate::proto::advanced_state::Gear;
        let gear_of = |data: [u8; 8]| {
            feed(54, data);
            embassy_futures::block_on(CAR_STATE.lock()).gear
        };
        assert_eq!(
            gear_of([0xF1, 0x66, 0xDB, 0x51, 0x05, 0x00, 0x35, 0x05]),
            Some(Gear::Drive)
        );
        assert_eq!(
            gear_of([0x7D, 0xB9, 0x3E, 0x51, 0x07, 0x00, 0x35, 0x05]),
            Some(Gear::Reverse)
        );
        assert_eq!(
            gear_of([0xFE, 0xC8, 0xD1, 0x51, 0x06, 0x00, 0x35, 0x05]),
            Some(Gear::Neutral)
        );
        assert_eq!(
            gear_of([0x0B, 0xEF, 0x0D, 0x51, 0x00, 0x00, 0x35, 0x05]),
            Some(Gear::Park)
        );
        // Car off: bit 36 is set next to the gear, which still reads Park.
        assert_eq!(
            gear_of([0x3C, 0x3A, 0x8E, 0x01, 0x10, 0x00, 0x30, 0x05]),
            Some(Gear::Park)
        );
    }

    /// log_09 from 4:03: off, accessory, on without the brake, then ready with it.
    #[test]
    fn power_state_follows_the_start_button() {
        use crate::proto::advanced_state::{Gear, PowerState};
        let state = || embassy_futures::block_on(CAR_STATE.lock()).clone();
        // Not ready, in Park.
        feed(54, [0x96, 0x4D, 0x62, 0x01, 0x10, 0x00, 0x30, 0x05]);
        feed(992, [0xBC, 0xD0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(state().power_state, Some(PowerState::Off));
        feed(992, [0x27, 0xE0, 0x01, 0x00, 0x04, 0x02, 0x00, 0x00]);
        assert_eq!(state().power_state, Some(PowerState::Accessory));
        feed(992, [0xB0, 0x10, 0x01, 0x00, 0x54, 0x03, 0x00, 0x00]);
        assert_eq!(state().power_state, Some(PowerState::On));
        assert_eq!(state().is_driving, Some(false));

        // Ready in Park is not driving; ready in Drive is, even when stopped.
        feed(992, [0xCF, 0xD0, 0x01, 0x00, 0x54, 0x03, 0x00, 0x00]);
        feed(54, [0x0B, 0xEF, 0x0D, 0x51, 0x00, 0x00, 0x35, 0x05]);
        assert_eq!(state().power_state, Some(PowerState::Ready));
        assert_eq!(state().is_driving, Some(false));
        feed(54, [0xF1, 0x66, 0xDB, 0x51, 0x05, 0x00, 0x35, 0x05]);
        assert_eq!(state().gear, Some(Gear::Drive));
        assert_eq!(state().is_driving, Some(true));
    }

    /// log_10: each seat through levels 3, 2, 1 and off, as the button steps them.
    #[test]
    fn seat_heating_levels() {
        use crate::proto::basic_state::SeatClimate;
        let driver = |data: [u8; 8]| {
            feed(1174, data);
            embassy_futures::block_on(CAR_STATE.lock()).driver_seat_climate
        };
        assert_eq!(
            driver([0x42, 0, 0, 0, 0, 0, 0, 0]),
            Some(SeatClimate::HeatLevel3)
        );
        assert_eq!(
            driver([0x3A, 0, 0, 0, 0, 0, 0, 0]),
            Some(SeatClimate::HeatLevel2)
        );
        assert_eq!(
            driver([0x32, 0, 0, 0, 0, 0, 0, 0]),
            Some(SeatClimate::HeatLevel1)
        );
        assert_eq!(driver([0x12, 0, 0, 0, 0, 0, 0, 0]), Some(SeatClimate::Off));

        feed(1141, [0x3A, 0, 0, 0, 0, 0, 0, 0]);
        let state = embassy_futures::block_on(CAR_STATE.lock()).clone();
        assert_eq!(state.passenger_seat_climate, Some(SeatClimate::HeatLevel2));
        assert_eq!(state.driver_seat_climate, Some(SeatClimate::Off));

        // A value never seen on the car keeps the last level.
        assert_eq!(seat_climate(4), None);
    }

    /// log_31: Car Scanner showed 12 C and 13 C for the battery minimum and maximum, then the
    /// car was switched off.
    #[test]
    fn battery_temperatures_and_12v() {
        let state = || embassy_futures::block_on(CAR_STATE.lock()).clone();
        feed(764, [0x37, 0x00, 0x0C, 0x0D, 0x00, 0x00, 0x88, 0x93]);
        assert_eq!(state().battery_minimum_temperature, Some(12.0));
        assert_eq!(state().battery_maximum_temperature, Some(13.0));
        assert!((state().auxiliary_battery_voltage.unwrap() - 13.6).abs() < 0.01);

        // The single 0 frame while switching off keeps the last voltage.
        feed(764, [0x08, 0x00, 0x0C, 0x0D, 0x00, 0x00, 0x00, 0x93]);
        assert!((state().auxiliary_battery_voltage.unwrap() - 13.6).abs() < 0.01);
    }

    /// log_30: the display switched from km to miles and back.
    #[test]
    fn distance_unit_follows_the_display() {
        use crate::proto::basic_state::DistanceUnit;
        let unit = |data: [u8; 8]| {
            feed(427, data);
            embassy_futures::block_on(CAR_STATE.lock()).distance_unit
        };
        assert_eq!(
            unit([0x7F, 0x09, 0xD2, 0x40, 0x70, 0x60, 0x00, 0x00]),
            Some(DistanceUnit::Miles)
        );
        assert_eq!(
            unit([0xA8, 0x87, 0x97, 0x00, 0x70, 0x60, 0x00, 0x00]),
            Some(DistanceUnit::Kilometers)
        );
    }

    /// log_32: the paddles through levels, i-Pedal and Auto, with a video of the cluster.
    #[test]
    fn regen_mode_and_level() {
        use crate::RegenMode;
        let state = || embassy_futures::block_on(CAR_STATE.lock()).clone();
        const NOT_I_PEDAL: [u8; 8] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x01];
        const I_PEDAL: [u8; 8] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0x03];

        feed(739, NOT_I_PEDAL);
        feed(738, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x08]);
        assert_eq!(state().regen_mode, Some(RegenMode::Manual));
        assert_eq!(state().regen_level, Some(3));
        feed(738, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]);
        assert_eq!(state().regen_level, Some(0));

        feed(738, [0xFF, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x08]);
        feed(739, I_PEDAL);
        assert_eq!(state().regen_mode, Some(RegenMode::IPedal));
        assert_eq!(state().regen_level, Some(4));

        // Auto, at its lowest step (the cluster's Auto 1): 0.1 rounds to 0.
        feed(739, NOT_I_PEDAL);
        feed(738, [0xFF, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x08]);
        assert_eq!(state().regen_mode, Some(RegenMode::Auto));
        assert_eq!(state().regen_level, Some(0));
        // Between steps (1.9).
        feed(738, [0xFF, 0x00, 0x00, 0x04, 0x00, 0x26, 0x00, 0x08]);
        assert_eq!(state().regen_level, Some(2));

        // log_31, car off: Not_Available keeps the last level.
        feed(738, [0xDE, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x08]);
        assert_eq!(state().regen_level, Some(2));
        assert_eq!(state().regen_mode, Some(RegenMode::Manual));
    }

    /// log_33: only the front left tire deflated to 32 psi; log_03 before the sensors report.
    #[test]
    fn tire_pressures() {
        let state = || embassy_futures::block_on(CAR_STATE.lock()).clone();
        feed(929, [0x3A, 0x5F, 0x9F, 0x00, 0x23, 0x23, 0x23, 0x23]);
        feed(929, [0x99, 0xF8, 0x7C, 0x00, 0x20, 0x23, 0x23, 0x23]);
        let s = state();
        assert!((s.front_left_tire_pressure_kpa.unwrap() - 220.6).abs() < 0.1);
        assert!((s.front_right_tire_pressure_kpa.unwrap() - 241.3).abs() < 0.1);
        assert!((s.rear_left_tire_pressure_kpa.unwrap() - 241.3).abs() < 0.1);
        assert!((s.rear_right_tire_pressure_kpa.unwrap() - 241.3).abs() < 0.1);

        feed(929, [0x65, 0x2C, 0x05, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(
            state().front_left_tire_pressure_kpa,
            s.front_left_tire_pressure_kpa
        );
    }

    #[test]
    fn blinkers_hazard_bit() {
        let on = m_can::Blinkers::try_from(&[0xCD, 0x65, 0x50, 0x04, 0x00, 0x00, 0x00, 0x00][..])
            .ok()
            .unwrap();
        let left_only =
            m_can::Blinkers::try_from(&[0xF0, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00][..])
                .ok()
                .unwrap();
        assert!(on.hazard_on());
        assert!(!left_only.hazard_on());
        assert!(left_only.left_stalk());
    }
}
