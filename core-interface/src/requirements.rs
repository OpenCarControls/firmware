//! Shared command requirements: the check every vehicle command runs before it builds a frame.
//!
//! This module knows nothing about cars. A vehicle crate declares what each command needs as a
//! [`CommandRequirements`] (how fresh its state must be, plus a list of [`Requirement`]s it evaluated
//! from its own state), and calls [`CommandRequirements::check`] with a [`Snapshot`] of the gate
//! inputs. A command that doesn't hold gets a [`Rejection`] with a clear message, which
//! [`rejection_response`] turns into a failed `CommandResponse` and [`try_send_response`]
//! queues without ever blocking.
//!
//! The BLE/MQTT dispatcher already refuses commands while the device is read-only (the CAN
//! read-write policy, `can_access.rs`). The check here repeats that against the live flag,
//! because a command can wait in the queue while the policy changes, and adds what only the
//! vehicle knows: how fresh its state is and what the car must be doing.
//!
//! On top of that, [`CommandRequirements::admit`] limits how often the same command may run
//! ([`RepeatGuard`]), so a stuck button or an app bug can't send it to the car again and again.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::cell::{Cell, RefCell};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};

use crate::availability::CommandKind;
use crate::channels::CMD_RESP_CHANNEL;
use crate::proto;
use crate::types::Transport;

// ── Activity clock ────────────────────────────────────────────────────────────

/// When the vehicle last heard from the car. The vehicle calls [`touch`](Self::touch) for every
/// CAN frame that matched its filters (or whatever else it counts as proof the state is live),
/// and reads the age back when a command arrives.
pub struct ActivityClock {
    last: Mutex<CriticalSectionRawMutex, Cell<Option<Instant>>>,
}

impl ActivityClock {
    pub const fn new() -> Self {
        Self {
            last: Mutex::new(Cell::new(None)),
        }
    }

    /// Records activity now.
    pub fn touch(&self) {
        self.touch_at(Instant::now());
    }

    /// Records activity at `at`. For callers (and tests) that control the clock.
    pub fn touch_at(&self, at: Instant) {
        self.last.lock(|last| last.set(Some(at)));
    }

    /// Time since the last activity, or `None` if there has been none since boot (or the last
    /// [`reset`](Self::reset)).
    pub fn age(&self) -> Option<Duration> {
        self.age_at(Instant::now())
    }

    /// Like [`age`](Self::age) at a given `now`. An activity time after `now` counts as age 0.
    pub fn age_at(&self, now: Instant) -> Option<Duration> {
        self.last
            .lock(|last| last.get())
            .map(|at| now.checked_duration_since(at).unwrap_or(Duration::MIN))
    }

    /// Forgets all activity, e.g. when the vehicle revokes the car identity.
    pub fn reset(&self) {
        self.last.lock(|last| last.set(None));
    }
}

impl Default for ActivityClock {
    fn default() -> Self {
        Self::new()
    }
}

// ── Snapshot ──────────────────────────────────────────────────────────────────

/// The inputs of the gate at the moment a command arrives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// Why the device is read-only, `None` while it may transmit
    /// ([`can_read_only_reason`](crate::can_read_only_reason)).
    pub read_only_reason: Option<&'static str>,
    /// Time since the vehicle last heard from the car; `None` if it never has.
    pub state_age: Option<Duration>,
}

impl Snapshot {
    /// Reads the live read-write policy, plus the age from `clock`.
    pub fn capture(clock: &ActivityClock) -> Self {
        Self {
            read_only_reason: crate::can_read_only_reason(),
            state_age: clock.age(),
        }
    }
}

// ── Requirements ──────────────────────────────────────────────────────────────

/// One condition on the car a command needs, already evaluated by the vehicle from its state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Requirement {
    /// `Some(true)` it holds, `Some(false)` it doesn't, `None` the state needed to tell isn't
    /// known yet. An unknown condition rejects the command (fail closed).
    pub holds: Option<bool>,
    /// The rejection message when the condition doesn't hold.
    pub unmet: &'static str,
    /// The rejection message when it isn't known.
    pub unknown: &'static str,
}

impl Requirement {
    pub const fn new(holds: Option<bool>, unmet: &'static str, unknown: &'static str) -> Self {
        Self {
            holds,
            unmet,
            unknown,
        }
    }
}

/// What a command needs before its frames may be built.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandRequirements {
    /// The state must have been refreshed at most this long ago.
    pub max_state_age: Duration,
    pub required: Vec<Requirement>,
    /// The same command is refused until this long after it was last admitted. Zero means no
    /// limit.
    pub min_interval: Duration,
}

impl CommandRequirements {
    pub fn new(max_state_age: Duration) -> Self {
        Self {
            max_state_age,
            required: Vec::new(),
            min_interval: Duration::from_ticks(0),
        }
    }

    pub fn min_interval(mut self, min_interval: Duration) -> Self {
        self.min_interval = min_interval;
        self
    }

    pub fn require(mut self, requirement: Requirement) -> Self {
        self.required.push(requirement);
        self
    }

    /// Checks the gate, in this order, and returns the first reason the command can't run:
    /// the device is read-only, state freshness, then each requirement in the order added.
    pub fn check(&self, snapshot: &Snapshot) -> Result<(), Rejection> {
        if let Some(reason) = snapshot.read_only_reason {
            return Err(Rejection::ReadOnly(reason));
        }
        match snapshot.state_age {
            None => return Err(Rejection::NoCarData),
            Some(age) if age > self.max_state_age => return Err(Rejection::StaleState),
            Some(_) => {}
        }
        for requirement in &self.required {
            match requirement.holds {
                Some(true) => {}
                Some(false) => return Err(Rejection::Unmet(requirement.unmet)),
                None => return Err(Rejection::Unknown(requirement.unknown)),
            }
        }
        Ok(())
    }

    /// Admits `command` now: [`check`](Self::check), then the repeat limit in `repeats`. An
    /// admitted command starts its interval, whether or not it then reaches the car.
    pub fn admit(
        &self,
        snapshot: &Snapshot,
        repeats: &RepeatGuard,
        command: (CommandKind, u32),
    ) -> Result<(), Rejection> {
        self.admit_at(snapshot, repeats, command, Instant::now())
    }

    /// Like [`admit`](Self::admit) at a given `now`.
    ///
    /// The repeat limit is not part of [`check`](Self::check) on purpose: availability is
    /// `check` alone, so a command stays available inside its interval instead of flickering
    /// after every press.
    pub fn admit_at(
        &self,
        snapshot: &Snapshot,
        repeats: &RepeatGuard,
        command: (CommandKind, u32),
        now: Instant,
    ) -> Result<(), Rejection> {
        self.check(snapshot)?;
        repeats
            .run_at(command, self.min_interval, now)
            .map_err(Rejection::TooSoon)
    }
}

// ── Repeat limit ──────────────────────────────────────────────────────────────

/// When each command of a vehicle was last admitted, to refuse the same command again within
/// its minimum interval. A stuck button sends a new `message_id` each time, so the dedupe in
/// the dispatcher doesn't catch it.
///
/// The limit is per command, whoever sent it and over whichever transport: it protects the
/// car. A command is identified by its kind and its id (`command_id` in the vehicle).
pub struct RepeatGuard {
    last: Mutex<CriticalSectionRawMutex, RefCell<Vec<LastRun>>>,
}

/// A command (kind and id) and when it was last admitted.
type LastRun = (CommandKind, u32, Instant);

impl RepeatGuard {
    pub const fn new() -> Self {
        Self {
            last: Mutex::new(RefCell::new(Vec::new())),
        }
    }

    /// Records a run of `command` at `now`, unless its last run is less than `min_interval`
    /// ago; then returns how long is left and records nothing.
    pub fn run_at(
        &self,
        command: (CommandKind, u32),
        min_interval: Duration,
        now: Instant,
    ) -> Result<(), Duration> {
        let (kind, id) = command;
        self.last.lock(|last| {
            let mut last = last.borrow_mut();
            match last.iter_mut().find(|(k, i, _)| *k == kind && *i == id) {
                Some((_, _, at)) => {
                    let elapsed = now.checked_duration_since(*at).unwrap_or(Duration::MIN);
                    if elapsed < min_interval {
                        return Err(min_interval - elapsed);
                    }
                    *at = now;
                }
                None => last.push((kind, id, now)),
            }
            Ok(())
        })
    }

    /// Forgets every run.
    pub fn reset(&self) {
        self.last.lock(|last| last.borrow_mut().clear());
    }
}

impl Default for RepeatGuard {
    fn default() -> Self {
        Self::new()
    }
}

// ── Rejection ─────────────────────────────────────────────────────────────────

/// Why a command was not run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rejection {
    /// The device must not transmit; carries the reason, e.g. "the car is not identified yet".
    ReadOnly(&'static str),
    /// The vehicle has not heard from the car since boot.
    NoCarData,
    /// The last word from the car is older than the command allows, e.g. the car is asleep or
    /// the bus is silent.
    StaleState,
    /// A requirement doesn't hold; the message comes from the vehicle.
    Unmet(&'static str),
    /// A requirement can't be evaluated yet; the message comes from the vehicle.
    Unknown(&'static str),
    /// The same command was admitted less than its minimum interval ago; carries the time
    /// left.
    TooSoon(Duration),
}

impl Rejection {
    /// The message for the app. The read-only wording matches the dispatcher's.
    pub fn message(&self) -> String {
        match self {
            Rejection::ReadOnly(reason) => {
                format!("command rejected: the device is read-only: {reason}")
            }
            Rejection::NoCarData => String::from("command rejected: no data from the car yet"),
            Rejection::StaleState => String::from(
                "command rejected: car state is out of date (car asleep or CAN bus silent)",
            ),
            Rejection::Unmet(message) | Rejection::Unknown(message) => String::from(*message),
            Rejection::TooSoon(left) => format!(
                "command rejected: the same command was sent too recently, try again in {} s",
                left.as_millis().div_ceil(1000).max(1)
            ),
        }
    }
}

// ── Responses ─────────────────────────────────────────────────────────────────

/// The failed `CommandResponse` for a rejected command, with the dedicated
/// `REJECTED_UNAVAILABLE` status code so the app can tell it from a failure.
pub fn rejection_response(message_id: u64, rejection: &Rejection) -> proto::CommandResponse {
    proto::CommandResponse {
        message_id,
        success: false,
        error_message: rejection.message(),
        response_data: None,
        status_code: proto::CommandStatusCode::RejectedUnavailable as i32,
    }
}

/// Queues a response for the router without waiting. Returns `false` if `CMD_RESP_CHANNEL` is
/// full and the response was dropped.
///
/// Use this for responses to commands that did nothing (rejections): blocking on a full
/// channel would stall the command handler behind the router and the BLE/MQTT drivers, which
/// backs up `BASIC_CMD_CHANNEL` and from there the BLE dispatcher. A dropped rejection costs
/// the app a retry; nothing was sent to the car.
pub fn try_send_response(transport: Transport, response: proto::CommandResponse) -> bool {
    CMD_RESP_CHANNEL.try_send((transport, response)).is_ok()
}

// ── Answering a command ───────────────────────────────────────────────────────

/// Why a command did not succeed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CommandError {
    /// The requirements did not hold. Nothing was built or sent to the car.
    Rejected(Rejection),
    /// The command was admitted but failed (decode error, not implemented, ...).
    Failed(&'static str),
}

/// Answers a command from a vehicle's handler. A response is always produced, also for a
/// rejected command.
///
/// A rejection is queued with `try_send` ([`try_send_response`]): nothing was done, so losing
/// it only costs the app a retry. Commands that ran, and ones that failed, keep the blocking
/// send so their result is not lost.
pub async fn respond_to_command(
    transport: Transport,
    message_id: u64,
    result: Result<(), CommandError>,
) {
    let (success, error_message) = match result {
        Ok(()) => (true, String::new()),
        Err(CommandError::Rejected(rejection)) => {
            try_send_response(transport, rejection_response(message_id, &rejection));
            return;
        }
        Err(CommandError::Failed(message)) => (false, String::from(message)),
    };
    let response = proto::CommandResponse {
        message_id,
        success,
        error_message,
        response_data: None,
        status_code: if success {
            proto::CommandStatusCode::Ok
        } else {
            proto::CommandStatusCode::Failed
        } as i32,
    };
    CMD_RESP_CHANNEL.sender().send((transport, response)).await;
}
