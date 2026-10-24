use core_interface::{
    ActivityClock, CMD_RESP_CHANNEL, Preconditions, Rejection, Requirement, Snapshot, Transport,
    proto, rejection_response, try_send_response,
};
use embassy_time::{Duration, Instant};

const MAX_AGE: Duration = Duration::from_secs(10);

/// A snapshot that passes the gate: identity confirmed, bus unlocked, state 1 s old.
fn open_gate() -> Snapshot {
    Snapshot {
        identity_confirmed: true,
        can_read_only: false,
        state_age: Some(Duration::from_secs(1)),
    }
}

fn parked(holds: Option<bool>) -> Requirement {
    Requirement::new(holds, "car is driving", "driving state unknown")
}

fn needs_parked(holds: Option<bool>) -> Preconditions {
    Preconditions::new(MAX_AGE).require(parked(holds))
}

#[test]
fn passes_when_gate_open_and_requirements_hold() {
    assert_eq!(needs_parked(Some(true)).check(&open_gate()), Ok(()));
}

#[test]
fn passes_with_no_requirements() {
    assert_eq!(Preconditions::new(MAX_AGE).check(&open_gate()), Ok(()));
}

#[test]
fn unconfirmed_identity_is_reported_before_read_only() {
    let snapshot = Snapshot {
        identity_confirmed: false,
        can_read_only: true,
        ..open_gate()
    };
    assert_eq!(
        needs_parked(Some(true)).check(&snapshot),
        Err(Rejection::IdentityNotConfirmed)
    );
}

#[test]
fn read_only_rejects_even_with_confirmed_identity() {
    let snapshot = Snapshot {
        can_read_only: true,
        ..open_gate()
    };
    assert_eq!(
        needs_parked(Some(true)).check(&snapshot),
        Err(Rejection::CanReadOnly)
    );
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
    let p = Preconditions::new(MAX_AGE);
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
    let p = Preconditions::new(MAX_AGE)
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
fn snapshot_capture_reads_the_live_gate() {
    let clock = ActivityClock::new();
    core_interface::revoke_car_identity();
    let snapshot = Snapshot::capture(&clock);
    assert!(!snapshot.identity_confirmed);
    assert!(snapshot.can_read_only);
    assert_eq!(snapshot.state_age, None);

    core_interface::confirm_car_identity();
    core_interface::set_can_read_only(false);
    clock.touch();
    let snapshot = Snapshot::capture(&clock);
    assert!(snapshot.identity_confirmed);
    assert!(!snapshot.can_read_only);
    assert!(snapshot.state_age.is_some());

    core_interface::revoke_car_identity();
}

#[test]
fn rejection_response_is_failed_with_the_message() {
    let response = rejection_response(7, &Rejection::CanReadOnly);
    assert_eq!(response.message_id, 7);
    assert!(!response.success);
    assert_eq!(response.error_message, Rejection::CanReadOnly.message());
    assert_eq!(
        response.status_code,
        proto::CommandStatusCode::Failed as i32
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
