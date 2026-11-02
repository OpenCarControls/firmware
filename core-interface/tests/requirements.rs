use core_interface::{
    ActivityClock, CMD_RESP_CHANNEL, CommandError, CommandKind, CommandRequirements, Rejection,
    RepeatGuard, Requirement, Snapshot, Transport, proto, rejection_response, respond_to_command,
    try_send_response,
};
use embassy_time::{Duration, Instant};

const MAX_AGE: Duration = Duration::from_secs(10);

/// A snapshot that passes the gate: identity confirmed, bus unlocked, state 1 s old.
fn open_gate() -> Snapshot {
    Snapshot {
        read_only_reason: None,
        state_age: Some(Duration::from_secs(1)),
    }
}

fn parked(holds: Option<bool>) -> Requirement {
    Requirement::new(holds, "car is driving", "driving state unknown")
}

fn needs_parked(holds: Option<bool>) -> CommandRequirements {
    CommandRequirements::new(MAX_AGE).require(parked(holds))
}

#[test]
fn passes_when_gate_open_and_requirements_hold() {
    assert_eq!(needs_parked(Some(true)).check(&open_gate()), Ok(()));
}

#[test]
fn passes_with_no_requirements() {
    assert_eq!(
        CommandRequirements::new(MAX_AGE).check(&open_gate()),
        Ok(())
    );
}

#[test]
fn read_only_rejects_with_the_reason() {
    let snapshot = Snapshot {
        read_only_reason: Some("the developer write lock is locked"),
        ..open_gate()
    };
    let rejection = needs_parked(Some(true)).check(&snapshot).unwrap_err();
    assert_eq!(
        rejection,
        Rejection::ReadOnly("the developer write lock is locked")
    );
    assert_eq!(
        rejection.message(),
        "command rejected: the device is read-only: the developer write lock is locked"
    );
}

#[test]
fn read_only_is_reported_before_staleness_and_requirements() {
    let snapshot = Snapshot {
        read_only_reason: Some("CAN logging is active"),
        state_age: None,
    };
    assert!(matches!(
        needs_parked(Some(false)).check(&snapshot),
        Err(Rejection::ReadOnly(_))
    ));
}

#[test]
fn no_car_data_rejects() {
    let snapshot = Snapshot {
        state_age: None,
        ..open_gate()
    };
    assert_eq!(
        needs_parked(Some(true)).check(&snapshot),
        Err(Rejection::NoCarData)
    );
}

#[test]
fn state_at_exactly_the_limit_is_fresh_and_past_it_is_stale() {
    let at_limit = Snapshot {
        state_age: Some(MAX_AGE),
        ..open_gate()
    };
    let past_limit = Snapshot {
        state_age: Some(MAX_AGE + Duration::from_millis(1)),
        ..open_gate()
    };
    let p = CommandRequirements::new(MAX_AGE);
    assert_eq!(p.check(&at_limit), Ok(()));
    assert_eq!(p.check(&past_limit), Err(Rejection::StaleState));
}

#[test]
fn staleness_is_reported_before_requirements() {
    // A stale "parked" fact must not be trusted, whatever it says.
    let snapshot = Snapshot {
        state_age: Some(Duration::from_secs(60)),
        ..open_gate()
    };
    assert_eq!(
        needs_parked(Some(false)).check(&snapshot),
        Err(Rejection::StaleState)
    );
}

#[test]
fn unmet_requirement_rejects_with_its_message() {
    let rejection = needs_parked(Some(false)).check(&open_gate()).unwrap_err();
    assert_eq!(rejection, Rejection::Unmet("car is driving"));
    assert_eq!(rejection.message(), "car is driving");
}

#[test]
fn unknown_requirement_fails_closed() {
    let rejection = needs_parked(None).check(&open_gate()).unwrap_err();
    assert_eq!(rejection, Rejection::Unknown("driving state unknown"));
}

#[test]
fn requirements_are_checked_in_order() {
    let p = CommandRequirements::new(MAX_AGE)
        .require(Requirement::new(Some(false), "first", "?"))
        .require(Requirement::new(Some(false), "second", "?"));
    assert_eq!(p.check(&open_gate()), Err(Rejection::Unmet("first")));
}

#[test]
fn activity_clock_ages() {
    let clock = ActivityClock::new();
    let t0 = Instant::from_millis(1_000);
    assert_eq!(clock.age_at(t0), None);
    clock.touch_at(t0);
    assert_eq!(
        clock.age_at(Instant::from_millis(4_500)),
        Some(Duration::from_millis(3_500))
    );
    // A touch later than `now` (clock read racing a touch) is age 0, not a panic.
    assert_eq!(
        clock.age_at(Instant::from_millis(500)),
        Some(Duration::from_millis(0))
    );
    clock.reset();
    assert_eq!(clock.age_at(t0), None);
}

#[test]
fn snapshot_capture_reads_the_live_policy() {
    let clock = ActivityClock::new();
    core_interface::reset_can_access_for_tests();
    core_interface::revoke_car_identity();
    let snapshot = Snapshot::capture(&clock);
    assert_eq!(
        snapshot.read_only_reason,
        core_interface::can_read_only_reason()
    );
    assert!(snapshot.read_only_reason.is_some());
    assert_eq!(snapshot.state_age, None);

    clock.touch();
    assert!(Snapshot::capture(&clock).state_age.is_some());
}

#[test]
fn rejection_response_is_unavailable_with_the_message() {
    let response = rejection_response(7, &Rejection::ReadOnly("CAN logging is active"));
    assert_eq!(response.message_id, 7);
    assert!(!response.success);
    assert_eq!(
        response.error_message,
        Rejection::ReadOnly("CAN logging is active").message()
    );
    assert_eq!(
        response.status_code,
        proto::CommandStatusCode::RejectedUnavailable as i32
    );
}

#[test]
fn try_send_response_never_blocks_on_a_full_channel() {
    while CMD_RESP_CHANNEL.try_receive().is_ok() {}
    let mut sent = 0;
    for id in 0..10 {
        if try_send_response(
            Transport::Ble,
            rejection_response(id, &Rejection::StaleState),
        ) {
            sent += 1;
        }
    }
    // The channel holds 4: the rest were dropped rather than waited for.
    assert_eq!(sent, 4);
    while CMD_RESP_CHANNEL.try_receive().is_ok() {}
}

fn answer(message_id: u64, result: Result<(), CommandError>) {
    embassy_futures::block_on(respond_to_command(Transport::Ble, message_id, result));
}

fn take_response() -> proto::CommandResponse {
    CMD_RESP_CHANNEL.try_receive().expect("a response").1
}

#[test]
fn every_outcome_gets_a_response_with_its_status_code() {
    while CMD_RESP_CHANNEL.try_receive().is_ok() {}

    answer(1, Ok(()));
    let ok = take_response();
    assert!(ok.success);
    assert_eq!(ok.status_code, proto::CommandStatusCode::Ok as i32);

    answer(2, Err(CommandError::Failed("Not implemented")));
    let failed = take_response();
    assert!(!failed.success);
    assert_eq!(failed.error_message, "Not implemented");
    assert_eq!(failed.status_code, proto::CommandStatusCode::Failed as i32);

    answer(3, Err(CommandError::Rejected(Rejection::StaleState)));
    let rejected = take_response();
    assert!(!rejected.success);
    assert_eq!(rejected.message_id, 3);
    assert_eq!(rejected.error_message, Rejection::StaleState.message());
    assert_eq!(
        rejected.status_code,
        proto::CommandStatusCode::RejectedUnavailable as i32
    );
}

#[test]
fn a_rejection_does_not_wait_for_a_full_response_channel() {
    while CMD_RESP_CHANNEL.try_receive().is_ok() {}
    for id in 0..4 {
        answer(id, Ok(()));
    }
    // Would hang forever if a rejection awaited the full channel.
    answer(99, Err(CommandError::Rejected(Rejection::StaleState)));
    let mut ids = Vec::new();
    while let Ok((_, response)) = CMD_RESP_CHANNEL.try_receive() {
        ids.push(response.message_id);
    }
    assert_eq!(ids, [0, 1, 2, 3], "the rejection was dropped, not queued");
}

// ── Repeat limit ──────────────────────────────────────────────────────────────

const DOOR_LOCK: (CommandKind, u32) = (CommandKind::Basic, 1);
const INTERVAL: Duration = Duration::from_secs(2);

fn limited() -> CommandRequirements {
    needs_parked(Some(true)).min_interval(INTERVAL)
}

fn at(ms: u64) -> Instant {
    Instant::from_millis(1_000_000 + ms)
}

#[test]
fn the_same_command_inside_its_interval_is_rejected_with_the_time_left() {
    let repeats = RepeatGuard::new();
    let p = limited();
    assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)), Ok(()));
    let rejection = p
        .admit_at(&open_gate(), &repeats, DOOR_LOCK, at(500))
        .unwrap_err();
    assert_eq!(rejection, Rejection::TooSoon(Duration::from_millis(1500)));
    assert_eq!(
        rejection.message(),
        "command rejected: the same command was sent too recently, try again in 2 s"
    );
    assert_eq!(
        rejection_response(1, &rejection).status_code,
        proto::CommandStatusCode::RejectedUnavailable as i32
    );
}

#[test]
fn the_interval_runs_from_the_last_admitted_run() {
    let repeats = RepeatGuard::new();
    let p = limited();
    assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)), Ok(()));
    // A refused repeat doesn't restart the interval.
    assert!(
        p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(1999))
            .is_err()
    );
    assert_eq!(
        p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(2000)),
        Ok(())
    );
    assert!(
        p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(3000))
            .is_err()
    );
}

#[test]
fn other_commands_have_their_own_interval() {
    let repeats = RepeatGuard::new();
    let p = limited();
    assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)), Ok(()));
    assert_eq!(
        p.admit_at(&open_gate(), &repeats, (CommandKind::Basic, 2), at(1)),
        Ok(())
    );
    // Advanced commands number their ids separately.
    assert_eq!(
        p.admit_at(&open_gate(), &repeats, (CommandKind::Advanced, 1), at(2)),
        Ok(())
    );
}

#[test]
fn a_command_rejected_by_its_requirements_does_not_start_the_interval() {
    let repeats = RepeatGuard::new();
    let driving = needs_parked(Some(false)).min_interval(INTERVAL);
    assert_eq!(
        driving.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)),
        Err(Rejection::Unmet("car is driving"))
    );
    assert_eq!(
        limited().admit_at(&open_gate(), &repeats, DOOR_LOCK, at(1)),
        Ok(())
    );
}

#[test]
fn the_requirements_are_reported_before_the_repeat_limit() {
    let repeats = RepeatGuard::new();
    let p = limited();
    assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)), Ok(()));
    let stale = Snapshot {
        state_age: Some(MAX_AGE + Duration::from_secs(1)),
        ..open_gate()
    };
    assert_eq!(
        p.admit_at(&stale, &repeats, DOOR_LOCK, at(1)),
        Err(Rejection::StaleState)
    );
}

#[test]
fn no_interval_never_limits() {
    let repeats = RepeatGuard::new();
    let p = needs_parked(Some(true));
    for _ in 0..3 {
        assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)), Ok(()));
    }
}

#[test]
fn check_ignores_the_repeat_limit() {
    // Availability is `check`: a command stays available inside its interval.
    let repeats = RepeatGuard::new();
    let p = limited();
    assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)), Ok(()));
    assert_eq!(p.check(&open_gate()), Ok(()));
}

#[test]
fn the_time_left_is_rounded_up_to_whole_seconds() {
    let message = |ms| Rejection::TooSoon(Duration::from_millis(ms)).message();
    assert!(message(1).ends_with("try again in 1 s"));
    assert!(message(1000).ends_with("try again in 1 s"));
    assert!(message(1001).ends_with("try again in 2 s"));
}

#[test]
fn reset_forgets_every_run() {
    let repeats = RepeatGuard::new();
    let p = limited();
    assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(0)), Ok(()));
    repeats.reset();
    assert_eq!(p.admit_at(&open_gate(), &repeats, DOOR_LOCK, at(1)), Ok(()));
}
