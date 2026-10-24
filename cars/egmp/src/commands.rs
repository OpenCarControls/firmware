//! What each E-GMP command needs from the car before it may run. The generic check lives in
//! `core_interface` (`Preconditions`); this module declares the E-GMP requirements and answers
//! the command. The table of preconditions per command is in `docs/commands.md`.

use alloc::string::String;
use core_interface::{
    ActivityClock, CMD_RESP_CHANNEL, Preconditions, Rejection, Requirement, Snapshot, Transport,
    rejection_response, try_send_response,
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
            identity_confirmed: true,
            can_read_only: false,
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

    fn open_the_gate() {
        core_interface::confirm_car_identity();
        core_interface::set_can_read_only(false);
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
            Err(CommandError::Rejected(r)) => String::from(r.message()),
            Err(CommandError::Failed(m)) => String::from(m),
            Ok(()) => String::from("ok"),
        }
    }

    #[test]
    fn handler_rejects_before_not_implemented_while_identity_unconfirmed() {
        core_interface::revoke_car_identity();
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(message, Rejection::IdentityNotConfirmed.message());
    }

    #[test]
    fn handler_reaches_not_implemented_when_the_gate_is_open() {
        open_the_gate();
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(message, "Not implemented");
        core_interface::revoke_car_identity();
    }

    #[test]
    fn handler_rejects_when_the_bus_was_never_heard() {
        core_interface::confirm_car_identity();
        core_interface::set_can_read_only(false);
        CAR_ACTIVITY.reset();
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        let message = error_message(block_on(crate::process_basic_command(&bytes)));
        assert_eq!(message, Rejection::NoCarData.message());
        core_interface::revoke_car_identity();
    }

    #[test]
    fn rejected_command_is_always_answered() {
        core_interface::revoke_car_identity();
        while CMD_RESP_CHANNEL.try_receive().is_ok() {}
        let bytes = encode(Action::FlashLightsCommand(proto::FlashLightsCommand {}));
        block_on(respond(
            Transport::Ble,
            9,
            block_on(crate::process_basic_command(&bytes)),
        ));
        let (_, response) = CMD_RESP_CHANNEL.try_receive().expect("a response");
        assert_eq!(response.message_id, 9);
        assert!(!response.success);
        assert_eq!(
            response.error_message,
            Rejection::IdentityNotConfirmed.message()
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
}
