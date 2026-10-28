//! What each E-GMP command needs from the car before it may run. The generic check lives in
//! `core_interface` (`CommandRequirements`); this module declares the E-GMP requirements and
//! reports which commands are available. The table of requirements per command is in
//! `docs/commands.md`.
//!
//! No command is implemented yet (firmware#37 to #42), so for now every command needs only
//! what all of them need: the device is not read-only and the car was heard from recently.
//! Each command's PR adds the requirements that command has, here.

use alloc::vec::Vec;
use core_interface::proto::CommandAvailabilityState as Availability;
use core_interface::{
    ActivityClock, CommandKind, CommandRequirements, Rejection, Snapshot,
    report_command_availability,
};
use embassy_time::Duration;

use crate::CAR_STATE;
use crate::EgmpCarState;
use crate::proto::{self, basic_command::Action};

/// When a CAN frame that matched the vehicle filters last arrived (touched by `can_rx_task`).
/// Any matching frame counts, not only the ones that feed the fact a command needs; this is
/// the "is the bus alive" input, not a per-signal age (see firmware#11).
pub(crate) static CAR_ACTIVITY: ActivityClock = ActivityClock::new();

/// A command is refused when the car has been silent for longer than this. The car sends
/// several of the filtered messages every second while awake, so a gap this long means it is
/// asleep or the bus is silent. Provisional: confirm against a log of the car falling asleep.
pub const MAX_STATE_AGE: Duration = Duration::from_secs(10);

// ── Requirements ──────────────────────────────────────────────────────────────

/// Requirements of a basic command, evaluated against `state`. A command's own requirements
/// (the car is not driving, the doors are not already locked, ...) are added in the PR that
/// implements it, from `state`.
pub fn basic_requirements(action: &Action, _state: &EgmpCarState) -> CommandRequirements {
    let base = CommandRequirements::new(MAX_STATE_AGE);
    match action {
        Action::DoorLockCommand(_)
        | Action::ChargePortCommand(_)
        | Action::FlashLightsCommand(_)
        | Action::ClimateControlCommand(_)
        | Action::WindowCommand(_) => base,
    }
}

/// Requirements of an advanced command, see [`basic_requirements`].
pub fn advanced_requirements(action: &proto::advanced_command::Action) -> CommandRequirements {
    match action {
        proto::advanced_command::Action::BatteryPreconditioningCommand(_) => {
            CommandRequirements::new(MAX_STATE_AGE)
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
/// at least one of them would be accepted, so a command whose requirements depend on the
/// request (locking and unlocking, say) lists each one.
fn basic_variants() -> Vec<(u32, Vec<Action>)> {
    use command_id::*;
    alloc::vec![
        (
            DOOR_LOCK,
            alloc::vec![Action::DoorLockCommand(proto::DoorLockCommand::default())]
        ),
        (
            CHARGE_PORT,
            alloc::vec![Action::ChargePortCommand(
                proto::ChargePortCommand::default()
            )]
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
/// [`basic_requirements`] would let the command through.
pub fn basic_availability(state: &EgmpCarState, snapshot: &Snapshot) -> Vec<(u32, Availability)> {
    basic_variants()
        .into_iter()
        .map(|(id, actions)| {
            let accepted = actions
                .iter()
                .any(|action| basic_requirements(action, state).check(snapshot).is_ok());
            (id, availability(accepted))
        })
        .collect()
}

/// What the app is told about each advanced command, see [`basic_availability`].
pub fn advanced_availability(snapshot: &Snapshot) -> Vec<(u32, Availability)> {
    let action = proto::advanced_command::Action::BatteryPreconditioningCommand(
        proto::BatteryPreconditioningCommand { enable: true },
    );
    let accepted = advanced_requirements(&action).check(snapshot).is_ok();
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
    // Build the requirements under the lock, but check after releasing it.
    let requirements = basic_requirements(action, &*CAR_STATE.lock().await);
    requirements.check(&Snapshot::capture(&CAR_ACTIVITY))
}

/// Runs the check for an advanced command against the live gate.
pub fn admit_advanced(action: &proto::advanced_command::Action) -> Result<(), Rejection> {
    advanced_requirements(action).check(&Snapshot::capture(&CAR_ACTIVITY))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::basic_command::Action;
    use core_interface::{CMD_RESP_CHANNEL, CommandError, Transport, respond_to_command};
    use embassy_futures::block_on;
    use embassy_time::Instant;
    use prost::Message;

    fn open_gate() -> Snapshot {
        Snapshot {
            read_only_reason: None,
            state_age: Some(Duration::from_secs(1)),
        }
    }

    fn stale() -> Snapshot {
        Snapshot {
            state_age: Some(MAX_STATE_AGE + Duration::from_secs(1)),
            ..open_gate()
        }
    }

    fn read_only() -> Snapshot {
        Snapshot {
            read_only_reason: Some("the car is not identified yet"),
            ..open_gate()
        }
    }

    fn flash_lights() -> Action {
        Action::FlashLightsCommand(proto::FlashLightsCommand {})
    }

    fn every_basic_action() -> alloc::vec::Vec<Action> {
        basic_variants().into_iter().flat_map(|(_, a)| a).collect()
    }

    // ── Requirements ──

    #[test]
    fn every_command_needs_only_the_gate_for_now() {
        // Per-command requirements arrive with each command (firmware#37 to #42).
        let state = EgmpCarState::new();
        for action in every_basic_action() {
            assert_eq!(
                basic_requirements(&action, &state).check(&open_gate()),
                Ok(())
            );
        }
        let advanced = proto::advanced_command::Action::BatteryPreconditioningCommand(
            proto::BatteryPreconditioningCommand { enable: true },
        );
        assert_eq!(advanced_requirements(&advanced).check(&open_gate()), Ok(()));
    }

    #[test]
    fn a_read_only_or_silent_device_rejects_every_command() {
        let state = EgmpCarState::new();
        for action in every_basic_action() {
            assert!(matches!(
                basic_requirements(&action, &state).check(&read_only()),
                Err(Rejection::ReadOnly(_))
            ));
            assert_eq!(
                basic_requirements(&action, &state).check(&stale()),
                Err(Rejection::StaleState)
            );
            assert_eq!(
                basic_requirements(&action, &state).check(&Snapshot {
                    state_age: None,
                    ..open_gate()
                }),
                Err(Rejection::NoCarData)
            );
        }
    }

    // ── Through the handlers (shares the process-global policy) ──

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

    fn error_message(result: Result<(), CommandError>) -> alloc::string::String {
        match result {
            Err(CommandError::Rejected(r)) => r.message(),
            Err(CommandError::Failed(m)) => alloc::string::String::from(m),
            Ok(()) => alloc::string::String::from("ok"),
        }
    }

    #[test]
    fn handler_rejects_before_not_implemented_while_read_only() {
        boot();
        let bytes = encode(flash_lights());
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(
            message,
            "command rejected: the device is read-only: the car platform is not recognised yet"
        );
    }

    #[test]
    fn handler_reaches_not_implemented_when_the_gate_is_open() {
        open_the_gate();
        let bytes = encode(flash_lights());
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(message, "Not implemented");
        boot();
    }

    #[test]
    fn handler_rejects_when_the_bus_was_never_heard() {
        open_the_gate();
        CAR_ACTIVITY.reset();
        let bytes = encode(flash_lights());
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(message, Rejection::NoCarData.message());
        boot();
    }

    #[test]
    fn rejected_command_is_always_answered() {
        boot();
        let bytes = encode(flash_lights());
        block_on(respond_to_command(
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
        assert_eq!(
            basic(Action::DoorLockCommand(proto::DoorLockCommand {
                lock: true
            })),
            DOOR_LOCK
        );
        assert_eq!(
            basic(Action::ChargePortCommand(proto::ChargePortCommand {
                open: true
            })),
            CHARGE_PORT
        );
        assert_eq!(basic(flash_lights()), FLASH_LIGHTS);
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
        let ids: alloc::vec::Vec<u32> = basic_availability(&EgmpCarState::new(), &open_gate())
            .iter()
            .map(|(id, _)| *id)
            .collect();
        assert_eq!(ids, [1, 2, 3, 4, 5]);
    }

    #[test]
    fn everything_is_available_while_the_gate_is_open() {
        let list = basic_availability(&EgmpCarState::new(), &open_gate());
        assert!(list.iter().all(|(_, a)| *a == Availability::Available));
        assert_eq!(
            advanced_availability(&open_gate()),
            [(command_id::BATTERY_PRECONDITIONING, Availability::Available)]
        );
    }

    #[test]
    fn a_silent_or_read_only_device_makes_everything_unavailable() {
        for snapshot in [stale(), read_only()] {
            let basic = basic_availability(&EgmpCarState::new(), &snapshot);
            assert!(basic.iter().all(|(_, a)| *a == Availability::Unavailable));
            assert!(
                advanced_availability(&snapshot)
                    .iter()
                    .all(|(_, a)| *a == Availability::Unavailable)
            );
            assert_eq!(
                avail(&basic, command_id::DOOR_LOCK),
                Availability::Unavailable
            );
        }
    }

    #[test]
    fn availability_matches_what_the_handler_accepts() {
        // The app greys out exactly the commands the handler would reject.
        let state = EgmpCarState::new();
        for snapshot in [open_gate(), stale(), read_only()] {
            let list = basic_availability(&state, &snapshot);
            for (id, variants) in basic_variants() {
                let accepted = variants
                    .iter()
                    .any(|a| basic_requirements(a, &state).check(&snapshot).is_ok());
                assert_eq!(avail(&list, id) == Availability::Available, accepted);
            }
        }
    }

    #[test]
    fn reported_availability_reaches_core_and_follows_the_policy() {
        use core_interface::command_availability;
        core_interface::reset_command_availability_for_tests();
        open_the_gate();
        block_on(report_availability());
        let shown = command_availability().expect("reported");
        assert_eq!(
            shown.basic[&command_id::FLASH_LIGHTS],
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
    }
}
