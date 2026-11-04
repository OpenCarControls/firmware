//! Phone validation sessions: a bonded phone proves it is the owner's by something done in
//! the car (see docs/ble.md, "Phone trust").
//!
//! `core-interface` runs the session; the vehicle provides the methods. The vehicle
//! declares them once ([`declare_validation_methods`]), follows the running challenge
//! ([`validation_challenge_receiver`]: show the code, or watch for the in-car action) and
//! reports a completed in-car action ([`complete_validation_action`]). The session always
//! belongs to the phone that started it, so the vehicle never chooses who is validated.
//!
//! One session runs at a time, for one phone, until it succeeds, fails, is cancelled or
//! times out ([`VALIDATION_SESSION_MS`]). A code entry session uses a fresh
//! [`CODE_LENGTH`]-digit code from the board's random source, accepts [`MAX_CODE_ATTEMPTS`]
//! wrong codes, and then starts a cooldown ([`VALIDATION_COOLDOWN_MS`]) during which no
//! session can start. The limits come from the contract (`[ble.phone_validation]` in
//! `transport.toml`), which the app reads too.

use alloc::vec::Vec;
use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use embassy_time::Instant;

use crate::can_access::CarDigest;
use crate::proto;
use crate::proto::PhoneValidationStatus as Status;

include!(concat!(env!("OUT_DIR"), "/phone_validation.rs"));

/// The number of distinct codes, 10^[`CODE_LENGTH`].
const CODE_SPACE: u32 = 10u32.pow(CODE_LENGTH as u32);

/// How the owner validates a phone with a method.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationKind {
    /// The car shows a code the owner types in the app.
    CodeEntry,
    /// The owner does something in the car, which the vehicle detects.
    InCarAction,
}

/// One way of validating a phone, declared by the vehicle.
#[derive(Debug)]
pub struct ValidationMethod {
    /// Stable identifier the app maps to its text, e.g. `"egmp.cluster_code"`.
    pub id: &'static str,
    pub kind: ValidationKind,
    /// The method sends frames to the car (a code on the cluster), so it can only start
    /// while the device may send the platform's frames (read-write mode).
    pub needs_approval: bool,
}

/// The running challenge, for the vehicle: `code` holds the ASCII digits to show, for a
/// code entry method.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ValidationChallenge {
    /// Identifies the session in [`complete_validation_action`].
    pub session: u32,
    pub method_id: &'static str,
    pub code: Option<[u8; CODE_LENGTH]>,
}

/// Why a validation command was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationError {
    UnknownMethod,
    /// The method needs read-write mode and the device is read-only.
    MethodUnavailable,
    CarNotIdentified,
    OtherPhoneSession,
    CoolingDown,
    NoSession,
    NotCodeEntry,
    NoRandomSource,
    WrongCode,
}

impl ValidationError {
    pub fn message(self) -> &'static str {
        match self {
            Self::UnknownMethod => "this car has no such validation method",
            Self::MethodUnavailable => {
                "this method needs CAN read-write mode, and the device is read-only"
            }
            Self::CarNotIdentified => {
                "the car is not identified yet, so the phone can't be validated"
            }
            Self::OtherPhoneSession => "another phone is validating, try again later",
            Self::CoolingDown => "too many wrong codes, try again later",
            Self::NoSession => "this phone has no validation in progress",
            Self::NotCodeEntry => "this validation doesn't take a code",
            Self::NoRandomSource => "the device can't generate a code",
            Self::WrongCode => "wrong code",
        }
    }
}

struct Session {
    id: u32,
    phone: Vec<u8>,
    method: &'static ValidationMethod,
    car: CarDigest,
    code: Option<[u8; CODE_LENGTH]>,
    attempts_left: u32,
    deadline_ms: u64,
    status: Status,
    ended_at_ms: u64,
}

struct State {
    methods: &'static [ValidationMethod],
    /// The latest session, kept after it ends for [`VALIDATION_RESULT_KEEP_MS`].
    session: Option<Session>,
    cooldown_until_ms: u64,
    next_id: u32,
    random: Option<fn(&mut [u8])>,
}

static STATE: Mutex<CriticalSectionRawMutex, RefCell<State>> = Mutex::new(RefCell::new(State {
    methods: &[],
    session: None,
    cooldown_until_ms: 0,
    next_id: 1,
    random: None,
}));

/// The running challenge for the vehicle, `None` while no session is in progress.
static CHALLENGE: Watch<CriticalSectionRawMutex, Option<ValidationChallenge>, 2> =
    Watch::new_with(None);

/// Declares the vehicle's validation methods, in the order the app offers them. Called
/// once at boot by the vehicle. A vehicle that declares none can't validate phones, so
/// its commands are never accepted.
pub fn declare_validation_methods(methods: &'static [ValidationMethod]) {
    STATE.lock(|s| s.borrow_mut().methods = methods);
}

/// Sets the source of the validation codes: a function that fills the buffer with
/// cryptographically random bytes. Called once at boot by the board. Without it, code
/// entry methods are refused rather than using guessable codes.
pub fn set_random_source(fill: fn(&mut [u8])) {
    STATE.lock(|s| s.borrow_mut().random = Some(fill));
}

/// A receiver for the running challenge: `changed()` resolves when a session starts or
/// ends. At most two (the vehicle, and a test).
pub fn validation_challenge_receiver()
-> Option<Receiver<'static, CriticalSectionRawMutex, Option<ValidationChallenge>, 2>> {
    CHALLENGE.receiver()
}

/// The running challenge now, `None` while no session is in progress.
pub fn current_validation_challenge() -> Option<ValidationChallenge> {
    CHALLENGE.try_get().flatten()
}

fn uniform_code(fill: fn(&mut [u8])) -> [u8; CODE_LENGTH] {
    // Rejection sampling, so every code is equally likely.
    const LIMIT: u32 = u32::MAX - (u32::MAX % CODE_SPACE);
    let value = loop {
        let mut buf = [0u8; 4];
        fill(&mut buf);
        let value = u32::from_le_bytes(buf);
        if value < LIMIT {
            break value % CODE_SPACE;
        }
    };
    let mut code = [b'0'; CODE_LENGTH];
    let mut rest = value;
    for digit in code.iter_mut().rev() {
        *digit = b'0' + (rest % 10) as u8;
        rest /= 10;
    }
    code
}

fn kind_proto(kind: ValidationKind) -> proto::PhoneValidationMethodKind {
    match kind {
        ValidationKind::CodeEntry => proto::PhoneValidationMethodKind::CodeEntry,
        ValidationKind::InCarAction => proto::PhoneValidationMethodKind::InCarAction,
    }
}

fn is_method_available(method: &ValidationMethod) -> bool {
    crate::can_access::identified_car().is_some()
        && (!method.needs_approval || !crate::can::is_can_read_only())
}

fn session_proto(session: &Session, state: &State, now_ms: u64) -> proto::PhoneValidationSession {
    let code_entry = session.method.kind == ValidationKind::CodeEntry;
    let in_progress = session.status == Status::InProgress;
    proto::PhoneValidationSession {
        method_id: session.method.id.into(),
        kind: kind_proto(session.method.kind) as i32,
        status: session.status as i32,
        code_length: if code_entry { CODE_LENGTH as u32 } else { 0 },
        attempts_left: if code_entry { session.attempts_left } else { 0 },
        expires_in_s: if in_progress {
            session.deadline_ms.saturating_sub(now_ms).div_ceil(1000) as u32
        } else {
            0
        },
        cooldown_s: state
            .cooldown_until_ms
            .saturating_sub(now_ms)
            .div_ceil(1000) as u32,
    }
}

/// Ends the running session with `status`. Returns whether there was one.
fn end_session(state: &mut State, status: Status, now_ms: u64) -> bool {
    match state.session.as_mut() {
        Some(session) if session.status == Status::InProgress => {
            session.status = status;
            session.ended_at_ms = now_ms;
            session.code = None;
            true
        }
        _ => false,
    }
}

fn after_change(challenge_ended: bool) {
    if challenge_ended {
        CHALLENGE.sender().send(None);
    }
    crate::phone_trust::notify_phone_access_changed();
}

/// The car's methods, with whether each can start now.
pub(crate) fn validation_methods() -> proto::PhoneValidationMethods {
    let methods = STATE.lock(|s| s.borrow().methods);
    proto::PhoneValidationMethods {
        methods: methods
            .iter()
            .map(|m| proto::PhoneValidationMethod {
                method_id: m.id.into(),
                kind: kind_proto(m.kind) as i32,
                needs_approval: m.needs_approval,
                available: is_method_available(m),
            })
            .collect(),
    }
}

/// `phone`'s latest session, while it runs and for [`VALIDATION_RESULT_KEEP_MS`] after.
pub(crate) fn session_for(phone: &[u8], now_ms: u64) -> Option<proto::PhoneValidationSession> {
    STATE.lock(|s| {
        let state = s.borrow();
        let session = state.session.as_ref()?;
        let shown = session.status == Status::InProgress
            || now_ms.saturating_sub(session.ended_at_ms) < VALIDATION_RESULT_KEEP_MS;
        (session.phone.as_slice() == phone && shown).then(|| session_proto(session, &state, now_ms))
    })
}

/// A refused validation command: why, and the phone's session when it explains it.
pub(crate) type Refused = (ValidationError, Option<proto::PhoneValidationSession>);

/// Starts validating `phone` with `method_id`. The phone must be bonded (checked by the
/// dispatcher). Replaces the phone's own running session.
pub(crate) fn start_validation(
    phone: &[u8],
    method_id: &str,
    now_ms: u64,
) -> Result<proto::PhoneValidationSession, Refused> {
    let car = crate::can_access::identified_car();
    let result = STATE.lock(|s| {
        let mut state = s.borrow_mut();
        let Some(method) = state.methods.iter().find(|m| m.id == method_id) else {
            return Err((ValidationError::UnknownMethod, None));
        };
        let Some(car) = car else {
            return Err((ValidationError::CarNotIdentified, None));
        };
        if now_ms < state.cooldown_until_ms {
            let own = state
                .session
                .as_ref()
                .filter(|s| s.phone.as_slice() == phone)
                .map(|s| session_proto(s, &state, now_ms));
            return Err((ValidationError::CoolingDown, own));
        }
        if let Some(session) = state.session.as_ref()
            && session.status == Status::InProgress
            && session.phone.as_slice() != phone
        {
            return Err((ValidationError::OtherPhoneSession, None));
        }
        if !is_method_available(method) {
            return Err((ValidationError::MethodUnavailable, None));
        }
        let code = match method.kind {
            ValidationKind::CodeEntry => match state.random {
                Some(fill) => Some(uniform_code(fill)),
                None => return Err((ValidationError::NoRandomSource, None)),
            },
            ValidationKind::InCarAction => None,
        };
        end_session(&mut state, Status::Cancelled, now_ms);
        let id = state.next_id;
        state.next_id = state.next_id.wrapping_add(1).max(1);
        let session = Session {
            id,
            phone: phone.to_vec(),
            method,
            car,
            code,
            attempts_left: MAX_CODE_ATTEMPTS,
            deadline_ms: now_ms.saturating_add(VALIDATION_SESSION_MS),
            status: Status::InProgress,
            ended_at_ms: 0,
        };
        let challenge = ValidationChallenge {
            session: id,
            method_id: method.id,
            code,
        };
        let reply = session_proto(&session, &state, now_ms);
        state.session = Some(session);
        Ok((reply, challenge))
    });
    let (reply, challenge) = result?;
    CHALLENGE.sender().send(Some(challenge));
    crate::phone_trust::notify_phone_access_changed();
    Ok(reply)
}

/// Marks the running session as succeeded and validates its phone, if the car is still
/// the one it started in.
fn succeed(state: &mut State, now_ms: u64) -> Option<(CarDigest, Vec<u8>)> {
    let current = crate::can_access::identified_car();
    let session = state.session.as_ref()?;
    if current != Some(session.car) {
        end_session(state, Status::Cancelled, now_ms);
        return None;
    }
    let validated = (session.car, session.phone.clone());
    end_session(state, Status::Succeeded, now_ms);
    Some(validated)
}

/// Checks the code `phone` typed against its running code entry session.
pub(crate) fn submit_validation_code(
    phone: &[u8],
    code: &str,
    now_ms: u64,
) -> Result<proto::PhoneValidationSession, Refused> {
    tick_validation(now_ms);
    let (result, validated, ended) = STATE.lock(|s| {
        let mut state = s.borrow_mut();
        let Some(session) = state
            .session
            .as_mut()
            .filter(|s| s.phone.as_slice() == phone && s.status == Status::InProgress)
        else {
            return (Err((ValidationError::NoSession, None)), None, false);
        };
        let Some(expected) = session.code else {
            return (Err((ValidationError::NotCodeEntry, None)), None, false);
        };
        let typed = code.trim().as_bytes();
        // Compares every digit, whatever the first mismatch.
        let matches = typed.len() == CODE_LENGTH
            && typed
                .iter()
                .zip(expected.iter())
                .fold(0u8, |acc, (a, b)| acc | (a ^ b))
                == 0;
        if matches {
            let validated = succeed(&mut state, now_ms);
            let reply = session_proto(state.session.as_ref().unwrap(), &state, now_ms);
            return (Ok(reply), validated, true);
        }
        session.attempts_left = session.attempts_left.saturating_sub(1);
        let failed = session.attempts_left == 0;
        if failed {
            end_session(&mut state, Status::Failed, now_ms);
            state.cooldown_until_ms = now_ms.saturating_add(VALIDATION_COOLDOWN_MS);
        }
        let reply = session_proto(state.session.as_ref().unwrap(), &state, now_ms);
        (Err((ValidationError::WrongCode, Some(reply))), None, failed)
    });
    if let Some((car, phone)) = validated {
        crate::phone_trust::add_phone_validation(car, &phone);
    }
    after_change(ended);
    result
}

/// Ends `phone`'s running session.
pub(crate) fn cancel_validation(phone: &[u8], now_ms: u64) -> Result<(), ValidationError> {
    let cancelled = STATE.lock(|s| {
        let mut state = s.borrow_mut();
        let own = state
            .session
            .as_ref()
            .is_some_and(|s| s.phone.as_slice() == phone);
        own && end_session(&mut state, Status::Cancelled, now_ms)
    });
    if !cancelled {
        return Err(ValidationError::NoSession);
    }
    after_change(true);
    Ok(())
}

/// Reported by the vehicle when the owner completed the in-car action of `session`
/// ([`ValidationChallenge::session`]). Validates the phone that started it. Ignored
/// (returns `false`) unless that session is still running and uses an in-car action.
pub fn complete_validation_action(session: u32) -> bool {
    complete_validation_action_at(session, Instant::now().as_millis())
}

/// [`complete_validation_action`] at `now_ms`.
pub fn complete_validation_action_at(session: u32, now_ms: u64) -> bool {
    tick_validation(now_ms);
    let validated = STATE.lock(|s| {
        let mut state = s.borrow_mut();
        let running = state.session.as_ref().is_some_and(|s| {
            s.id == session
                && s.status == Status::InProgress
                && s.method.kind == ValidationKind::InCarAction
        });
        if running {
            Some(succeed(&mut state, now_ms))
        } else {
            None
        }
    });
    let Some(validated) = validated else {
        return false;
    };
    let ok = validated.is_some();
    if let Some((car, phone)) = validated {
        crate::phone_trust::add_phone_validation(car, &phone);
    }
    after_change(true);
    ok
}

/// Times out the running session, and cancels it if the identified car changed. Called
/// every second by the access state publisher, and before every session change.
pub fn tick_validation(now_ms: u64) {
    let car = crate::can_access::identified_car();
    let ended = STATE.lock(|s| {
        let mut state = s.borrow_mut();
        let Some(session) = state.session.as_ref() else {
            return false;
        };
        if session.status != Status::InProgress {
            return false;
        }
        if car != Some(session.car) {
            end_session(&mut state, Status::Cancelled, now_ms)
        } else if now_ms >= session.deadline_ms {
            end_session(&mut state, Status::TimedOut, now_ms)
        } else {
            false
        }
    });
    if ended {
        after_change(true);
    }
}

/// Forgets the methods, the session, the cooldown and the random source. For tests only.
pub fn reset_phone_validation_for_tests() {
    STATE.lock(|s| {
        let mut state = s.borrow_mut();
        state.methods = &[];
        state.session = None;
        state.cooldown_until_ms = 0;
        state.random = None;
    });
    CHALLENGE.sender().send(None);
}
