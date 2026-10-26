//! What each E-GMP command needs from the car before it may run. The generic check lives in
//! `core_interface` (`Preconditions`); this module declares the E-GMP requirements and answers
//! the command. The table of preconditions per command is in `docs/commands.md`.

use alloc::string::String;
use alloc::vec::Vec;
use core_interface::proto::CommandAvailabilityState as Availability;
use core_interface::{
    ActivityClock, CMD_RESP_CHANNEL, CommandKind, Preconditions, Rejection, Requirement, Snapshot,
    Transport, rejection_response, report_command_availability, try_send_response,
};
use embassy_time::Duration;

use crate::proto::{self, basic_command::Action, basic_state::ChargePortState};
use crate::{CAR_STATE, EgmpCarState};

/// When a CAN frame that matched the vehicle filters last arrived (touched by `can_rx_task`).
/// Any matching frame counts, not only the ones that feed the fact a command needs; this is
/// the "is the bus alive" input, not a per-signal age (see firmware#11).
pub(crate) static CAR_ACTIVITY: ActivityClock = ActivityClock::new();

/// A command is refused when the car has been silent for longer than this. The car sends
/// several of the filtered messages every second while awake, so a gap this long means it is
/// asleep or the bus is silent. Provisional: confirm against a log of the car falling asleep.
pub const MAX_STATE_AGE: Duration = Duration::from_secs(10);

/// Why a command did not succeed.
pub enum CommandError {
    /// The preconditions did not hold. Nothing was built or sent to the car.
    Rejected(Rejection),
    /// The command was admitted but failed (decode error, not implemented, ...).
    Failed(&'static str),
}

// ── Requirements ──────────────────────────────────────────────────────────────

fn not_driving(state: &EgmpCarState) -> Requirement {
    Requirement::new(
        state.is_driving.map(|driving| !driving),
        "command rejected: the car is driving",
        "command rejected: the car's driving state is not known yet",
    )
}

fn doors_not_already(state: &EgmpCarState, lock: bool) -> Requirement {
    Requirement::new(
        state.are_doors_locked.map(|locked| locked != lock),
        if lock {
            "command rejected: the doors are already locked"
        } else {
            "command rejected: the doors are already unlocked"
        },
        "command rejected: the door lock state is not known yet",
    )
}

fn charge_port_not_already(state: &EgmpCarState, open: bool) -> Requirement {
    let wanted = if open {
        ChargePortState::Open
    } else {
        ChargePortState::Closed
    };
    Requirement::new(
        state
            .charge_port_state
            .filter(|s| *s != ChargePortState::Unspecified)
            .map(|current| current != wanted),
        if open {
            "command rejected: the charge port is already open"
        } else {
            "command rejected: the charge port is already closed"
        },
        "command rejected: the charge port state is not known yet",
    )
}

/// Preconditions of a basic command, evaluated against `state`. Provisional until each command
/// is confirmed on a real car, see `docs/commands.md`.
pub fn basic_preconditions(action: &Action, state: &EgmpCarState) -> Preconditions {
    let base = Preconditions::new(MAX_STATE_AGE);
    match action {
        Action::DoorLockCommand(c) => base.require(doors_not_already(state, c.lock)),
        Action::ChargePortCommand(c) => base
            .require(not_driving(state))
            .require(charge_port_not_already(state, c.open)),
        Action::WindowCommand(_) => base.require(not_driving(state)),
        Action::FlashLightsCommand(_) | Action::ClimateControlCommand(_) => base,
    }
}

/// Preconditions of an advanced command, see [`basic_preconditions`].
pub fn advanced_preconditions(action: &proto::advanced_command::Action) -> Preconditions {
    match action {
        proto::advanced_command::Action::BatteryPreconditioningCommand(_) => {
            Preconditions::new(MAX_STATE_AGE)
        }
    }
}

// ── Availability ──────────────────────────────────────────────────────────────

/// The commands as the app identifies them: the field number in the `BasicCommand` /
/// `AdvancedCommand` oneof of `egmp.proto`. A test pins them to the wire.
pub mod command_id {
    pub const DOOR_LOCK: u32 = 1;
    pub const CHARGE_PORT: u32 = 2;
    pub const FLASH_LIGHTS: u32 = 3;
    pub const CLIMATE_CONTROL: u32 = 4;
    pub const WINDOW: u32 = 5;
    pub const BATTERY_PRECONDITIONING: u32 = 1;
}

/// Each basic command with the requests the app can make of it. A command is available when
/// at least one of them would be accepted: the door lock command, for example, is available
/// while either locking or unlocking would change something.
fn basic_variants() -> Vec<(u32, Vec<Action>)> {
    use command_id::*;
    alloc::vec![
        (
            DOOR_LOCK,
            [true, false]
                .map(|lock| Action::DoorLockCommand(proto::DoorLockCommand { lock }))
                .into()
        ),
        (
            CHARGE_PORT,
            [true, false]
                .map(|open| Action::ChargePortCommand(proto::ChargePortCommand { open }))
                .into()
        ),
        (
            FLASH_LIGHTS,
            alloc::vec![Action::FlashLightsCommand(proto::FlashLightsCommand {})]
        ),
        (
            CLIMATE_CONTROL,
            alloc::vec![Action::ClimateControlCommand(
                proto::ClimateControlCommand::default()
            )]
        ),
        (
            WINDOW,
            alloc::vec![Action::WindowCommand(proto::WindowCommand::default())]
        ),
    ]
}

fn availability(accepted: bool) -> Availability {
    if accepted {
        Availability::Available
    } else {
        Availability::Unavailable
    }
}

/// What the app is told about each basic command: available exactly when
/// [`basic_preconditions`] would let the command through.
pub fn basic_availability(state: &EgmpCarState, snapshot: &Snapshot) -> Vec<(u32, Availability)> {
    basic_variants()
        .into_iter()
        .map(|(id, actions)| {
            let accepted = actions
                .iter()
                .any(|action| basic_preconditions(action, state).check(snapshot).is_ok());
            (id, availability(accepted))
        })
        .collect()
}

/// What the app is told about each advanced command, see [`basic_availability`].
pub fn advanced_availability(snapshot: &Snapshot) -> Vec<(u32, Availability)> {
    let action = proto::advanced_command::Action::BatteryPreconditioningCommand(
        proto::BatteryPreconditioningCommand { enable: true },
    );
    let accepted = advanced_preconditions(&action).check(snapshot).is_ok();
    alloc::vec![(command_id::BATTERY_PRECONDITIONING, availability(accepted))]
}

/// Evaluates every command against the live car state and tells `core-interface`, which
/// publishes it to the app. Called about once a second by `state_update_task`.
pub async fn report_availability() {
    let snapshot = Snapshot::capture(&CAR_ACTIVITY);
    let basic = basic_availability(&*CAR_STATE.lock().await, &snapshot);
    report_command_availability(CommandKind::Basic, &basic);
    report_command_availability(CommandKind::Advanced, &advanced_availability(&snapshot));
}

// ── Admission ─────────────────────────────────────────────────────────────────

/// Runs the check for a basic command against the live car state and gate.
pub async fn admit_basic(action: &Action) -> Result<(), Rejection> {
    // Build the preconditions under the lock, but check after releasing it.
    let preconditions = basic_preconditions(action, &*CAR_STATE.lock().await);
    preconditions.check(&Snapshot::capture(&CAR_ACTIVITY))
}

/// Runs the check for an advanced command against the live gate.
pub fn admit_advanced(action: &proto::advanced_command::Action) -> Result<(), Rejection> {
    advanced_preconditions(action).check(&Snapshot::capture(&CAR_ACTIVITY))
}

// ── Responses ─────────────────────────────────────────────────────────────────

/// Answers a command. A response is always produced, also for a rejected command.
///
/// A rejection is queued with `try_send`: nothing was done, so losing it only costs the app a
/// retry, and blocking on a full `CMD_RESP_CHANNEL` would stall this handler behind the router
/// and drivers. Commands that ran keep the blocking send so their result is not lost.
pub(crate) async fn respond(
    transport: Transport,
    message_id: u64,
    result: Result<(), CommandError>,
) {
    let response = match result {
        Err(CommandError::Rejected(rejection)) => {
            try_send_response(transport, rejection_response(message_id, &rejection));
            return;
        }
        Ok(()) => proto_response(message_id, true, String::new()),
        Err(CommandError::Failed(message)) => {
            proto_response(message_id, false, String::from(message))
        }
    };
    CMD_RESP_CHANNEL.sender().send((transport, response)).await;
}

fn proto_response(
    message_id: u64,
    success: bool,
    error_message: String,
) -> core_interface::proto::CommandResponse {
    core_interface::proto::CommandResponse {
        message_id,
        success,
        error_message,
        response_data: None,
        status_code: if success { 1 } else { 2 },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::basic_command::Action;
    use embassy_futures::block_on;
    use embassy_time::Instant;
    use prost::Message;

    const FRESH: Duration = Duration::from_secs(1);

    fn open_gate() -> Snapshot {
        Snapshot {
            read_only_reason: None,
            state_age: Some(FRESH),
        }
    }

    fn check(action: Action, state: &EgmpCarState) -> Result<(), Rejection> {
        basic_preconditions(&action, state).check(&open_gate())
    }

    fn lock(lock: bool) -> Action {
        Action::DoorLockCommand(proto::DoorLockCommand { lock })
    }

    fn charge_port(open: bool) -> Action {
        Action::ChargePortCommand(proto::ChargePortCommand { open })
    }

    fn parked() -> EgmpCarState {
        EgmpCarState {
            is_driving: Some(false),
            ..EgmpCarState::new()
        }
    }

    fn driving() -> EgmpCarState {
        EgmpCarState {
            is_driving: Some(true),
            ..EgmpCarState::new()
        }
    }

    #[test]
    fn charge_port_needs_a_parked_car() {
        let state = EgmpCarState {
            charge_port_state: Some(ChargePortState::Closed),
            ..driving()
        };
        let rejection = check(charge_port(true), &state).unwrap_err();
        assert_eq!(rejection.message(), "command rejected: the car is driving");
    }

    #[test]
    fn charge_port_with_unknown_driving_state_is_rejected() {
        let state = EgmpCarState {
            charge_port_state: Some(ChargePortState::Closed),
            ..EgmpCarState::new()
        };
        assert!(matches!(
            check(charge_port(true), &state),
            Err(Rejection::Unknown(_))
        ));
    }

    #[test]
    fn charge_port_open_when_parked_and_closed_passes() {
        let state = EgmpCarState {
            charge_port_state: Some(ChargePortState::Closed),
            ..parked()
        };
        assert_eq!(check(charge_port(true), &state), Ok(()));
    }

    #[test]
    fn charge_port_already_in_the_requested_state_is_rejected() {
        let open = EgmpCarState {
            charge_port_state: Some(ChargePortState::Open),
            ..parked()
        };
        let closed = EgmpCarState {
            charge_port_state: Some(ChargePortState::Closed),
            ..parked()
        };
        assert_eq!(
            check(charge_port(true), &open).unwrap_err().message(),
            "command rejected: the charge port is already open"
        );
        assert_eq!(
            check(charge_port(false), &closed).unwrap_err().message(),
            "command rejected: the charge port is already closed"
        );
        assert_eq!(check(charge_port(false), &open), Ok(()));
    }

    #[test]
    fn unspecified_charge_port_state_counts_as_unknown() {
        let state = EgmpCarState {
            charge_port_state: Some(ChargePortState::Unspecified),
            ..parked()
        };
        assert!(matches!(
            check(charge_port(true), &state),
            Err(Rejection::Unknown(_))
        ));
    }

    #[test]
    fn door_lock_that_changes_nothing_is_rejected() {
        let locked = EgmpCarState {
            are_doors_locked: Some(true),
            ..EgmpCarState::new()
        };
        assert_eq!(
            check(lock(true), &locked).unwrap_err().message(),
            "command rejected: the doors are already locked"
        );
        assert_eq!(check(lock(false), &locked), Ok(()));
    }

    #[test]
    fn door_lock_with_unknown_state_is_rejected() {
        assert!(matches!(
            check(lock(true), &EgmpCarState::new()),
            Err(Rejection::Unknown(_))
        ));
    }

    #[test]
    fn flash_lights_only_needs_the_gate() {
        let action = Action::FlashLightsCommand(proto::FlashLightsCommand {});
        assert_eq!(check(action, &EgmpCarState::new()), Ok(()));
    }

    #[test]
    fn gate_failures_come_before_requirements() {
        let action = Action::WindowCommand(proto::WindowCommand::default());
        let stale = Snapshot {
            state_age: Some(MAX_STATE_AGE + Duration::from_secs(1)),
            ..open_gate()
        };
        assert_eq!(
            basic_preconditions(&action, &parked()).check(&stale),
            Err(Rejection::StaleState)
        );
    }

    // ── Through the handlers (shares the process-global gate and CAR_STATE) ──

    /// Boot state: nothing identified, approved or unlocked, so the device is read-only.
    fn boot() {
        core_interface::reset_can_access_for_tests();
        core_interface::revoke_car_identity();
        core_interface::set_can_read_only(true);
        while CMD_RESP_CHANNEL.try_receive().is_ok() {}
    }

    /// Every condition of the read-write policy met, and the car heard just now.
    fn open_the_gate() {
        boot();
        core_interface::confirm_car_identity();
        assert!(core_interface::confirm_car_fingerprint(b"TEST-CAR"));
        core_interface::approve_can_read_write().unwrap();
        core_interface::set_can_dev_write_unlocked(true).unwrap();
        assert!(!core_interface::set_can_read_only(false));
        CAR_ACTIVITY.touch_at(Instant::now());
    }

    fn encode(action: Action) -> alloc::vec::Vec<u8> {
        proto::BasicCommand {
            action: Some(action),
        }
        .encode_to_vec()
    }

    fn error_message(result: Result<(), CommandError>) -> String {
        match result {
            Err(CommandError::Rejected(r)) => r.message(),
            Err(CommandError::Failed(m)) => String::from(m),
            Ok(()) => String::from("ok"),
        }
    }

    #[test]
    fn handler_rejects_before_not_implemented_while_read_only() {
        boot();
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(
            message,
            "command rejected: the device is read-only: the car platform is not recognised yet"
        );
    }

    #[test]
    fn handler_reaches_not_implemented_when_the_gate_is_open() {
        open_the_gate();
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(message, "Not implemented");
        boot();
    }

    #[test]
    fn handler_rejects_when_the_bus_was_never_heard() {
        open_the_gate();
        CAR_ACTIVITY.reset();
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(message, Rejection::NoCarData.message());
        boot();
    }

    #[test]
    fn rejected_command_is_always_answered() {
        boot();
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        block_on(respond(
            Transport::Ble,
            9,
            block_on(crate::process_basic_command(&bytes)),
        ));
        let (_, response) = CMD_RESP_CHANNEL.try_receive().expect("a response");
        assert_eq!(response.message_id, 9);
        assert!(!response.success);
        assert!(
            response
                .error_message
                .starts_with("command rejected: the device is read-only")
        );
    }

    #[test]
    fn rejections_do_not_block_when_the_response_channel_is_full() {
        while CMD_RESP_CHANNEL.try_receive().is_ok() {}
        for id in 0..4 {
            assert!(
                CMD_RESP_CHANNEL
                    .try_send((Transport::Ble, proto_response(id, true, String::new())))
                    .is_ok()
            );
        }
        // Would hang forever if `respond` awaited the full channel.
        block_on(respond(
            Transport::Ble,
            99,
            Err(CommandError::Rejected(Rejection::StaleState)),
        ));
        while CMD_RESP_CHANNEL.try_receive().is_ok() {}
    }
    // ── Availability ──

    fn avail(list: &[(u32, Availability)], id: u32) -> Availability {
        list.iter()
            .find(|(i, _)| *i == id)
            .expect("command listed")
            .1
    }

    #[test]
    fn command_ids_are_the_wire_field_numbers() {
        use command_id::*;
        let first_field = |bytes: alloc::vec::Vec<u8>| u32::from(bytes[0] >> 3);
        let basic = |action| first_field(encode(action));
        assert_eq!(basic(lock(true)), DOOR_LOCK);
        assert_eq!(basic(charge_port(true)), CHARGE_PORT);
        assert_eq!(
            basic(Action::FlashLightsCommand(proto::FlashLightsCommand {})),
            FLASH_LIGHTS
        );
        assert_eq!(
            basic(Action::ClimateControlCommand(
                proto::ClimateControlCommand::default()
            )),
            CLIMATE_CONTROL
        );
        assert_eq!(
            basic(Action::WindowCommand(proto::WindowCommand::default())),
            WINDOW
        );
        let advanced = proto::AdvancedCommand {
            action: Some(
                proto::advanced_command::Action::BatteryPreconditioningCommand(
                    proto::BatteryPreconditioningCommand { enable: true },
                ),
            ),
        }
        .encode_to_vec();
        assert_eq!(first_field(advanced), BATTERY_PRECONDITIONING);
    }

    #[test]
    fn every_basic_command_is_listed() {
        let ids: alloc::vec::Vec<u32> = basic_availability(&parked(), &open_gate())
            .iter()
            .map(|(id, _)| *id)
            .collect();
        assert_eq!(ids, [1, 2, 3, 4, 5]);
    }

    #[test]
    fn parked_car_with_known_state_has_everything_available() {
        let state = EgmpCarState {
            are_doors_locked: Some(true),
            charge_port_state: Some(ChargePortState::Closed),
            ..parked()
        };
        let list = basic_availability(&state, &open_gate());
        assert!(list.iter().all(|(_, a)| *a == Availability::Available));
        assert_eq!(
            advanced_availability(&open_gate()),
            [(command_id::BATTERY_PRECONDITIONING, Availability::Available)]
        );
    }

    #[test]
    fn driving_makes_the_charge_port_and_windows_unavailable_only() {
        let state = EgmpCarState {
            are_doors_locked: Some(true),
            charge_port_state: Some(ChargePortState::Closed),
            ..driving()
        };
        let list = basic_availability(&state, &open_gate());
        assert_eq!(
            avail(&list, command_id::CHARGE_PORT),
            Availability::Unavailable
        );
        assert_eq!(avail(&list, command_id::WINDOW), Availability::Unavailable);
        assert_eq!(avail(&list, command_id::DOOR_LOCK), Availability::Available);
        assert_eq!(
            avail(&list, command_id::FLASH_LIGHTS),
            Availability::Available
        );
    }

    #[test]
    fn unknown_state_makes_the_commands_that_need_it_unavailable() {
        let list = basic_availability(&EgmpCarState::new(), &open_gate());
        assert_eq!(
            avail(&list, command_id::DOOR_LOCK),
            Availability::Unavailable
        );
        assert_eq!(
            avail(&list, command_id::CHARGE_PORT),
            Availability::Unavailable
        );
        assert_eq!(
            avail(&list, command_id::FLASH_LIGHTS),
            Availability::Available
        );
    }

    #[test]
    fn a_silent_or_read_only_device_makes_everything_unavailable() {
        let stale = Snapshot {
            state_age: Some(MAX_STATE_AGE + Duration::from_secs(1)),
            ..open_gate()
        };
        let read_only = Snapshot {
            read_only_reason: Some("the car is not identified yet"),
            ..open_gate()
        };
        for snapshot in [stale, read_only] {
            assert!(
                basic_availability(&parked(), &snapshot)
                    .iter()
                    .all(|(_, a)| *a == Availability::Unavailable)
            );
            assert!(
                advanced_availability(&snapshot)
                    .iter()
                    .all(|(_, a)| *a == Availability::Unavailable)
            );
        }
    }

    #[test]
    fn availability_matches_what_the_handler_accepts() {
        // The app greys out exactly the commands the handler would reject.
        for state in [driving(), parked(), EgmpCarState::new()] {
            let list = basic_availability(&state, &open_gate());
            for (id, variants) in basic_variants() {
                let accepted = variants
                    .iter()
                    .any(|a| basic_preconditions(a, &state).check(&open_gate()).is_ok());
                assert_eq!(avail(&list, id) == Availability::Available, accepted);
            }
        }
    }

    #[test]
    fn reported_availability_reaches_core_and_follows_the_policy() {
        use core_interface::command_availability;
        core_interface::reset_command_availability_for_tests();
        open_the_gate();
        {
            let mut state = block_on(CAR_STATE.lock());
            *state = EgmpCarState {
                are_doors_locked: Some(false),
                charge_port_state: Some(ChargePortState::Closed),
                ..parked()
            };
        }
        block_on(report_availability());
        let shown = command_availability().expect("reported");
        assert_eq!(
            shown.basic[&command_id::DOOR_LOCK],
            Availability::Available as i32
        );

        // Read-only again: core holds everything back without waiting for a new report.
        boot();
        let shown = command_availability().expect("reported");
        assert!(
            shown
                .basic
                .values()
                .all(|a| *a == Availability::Unavailable as i32)
        );
        *block_on(CAR_STATE.lock()) = EgmpCarState::new();
    }
}
