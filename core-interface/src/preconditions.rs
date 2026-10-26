//! Shared command preconditions: the check every vehicle command runs before it builds a frame.
//!
//! This module knows nothing about cars. A vehicle crate declares what each command needs as a
//! [`Preconditions`] (how fresh its state must be, plus a list of [`Requirement`]s it evaluated
//! from its own state), and calls [`Preconditions::check`] with a [`Snapshot`] of the gate
//! inputs. A command that doesn't hold gets a [`Rejection`] with a clear message, which
//! [`rejection_response`] turns into a failed `CommandResponse` and [`try_send_response`]
//! queues without ever blocking.
//!
//! The BLE/MQTT dispatcher already refuses commands while the device is read-only (the CAN
//! read-write policy, `can_access.rs`). The check here repeats that against the live flag,
//! because a command can wait in the queue while the policy changes, and adds what only the
//! vehicle knows: how fresh its state is and what the car must be doing.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::cell::Cell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};

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
pub struct Preconditions {
    /// The state must have been refreshed at most this long ago.
    pub max_state_age: Duration,
    pub requirements: Vec<Requirement>,
}

impl Preconditions {
    pub fn new(max_state_age: Duration) -> Self {
        Self {
            max_state_age,
            requirements: Vec::new(),
        }
    }

    pub fn require(mut self, requirement: Requirement) -> Self {
        self.requirements.push(requirement);
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
        for requirement in &self.requirements {
            match requirement.holds {
                Some(true) => {}
                Some(false) => return Err(Rejection::Unmet(requirement.unmet)),
                None => return Err(Rejection::Unknown(requirement.unknown)),
            }
        }
        Ok(())
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
