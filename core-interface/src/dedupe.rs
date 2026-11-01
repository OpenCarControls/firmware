//! Duplicate command detection.
//!
//! The app may resend a command with the same `message_id` (after switching transport,
//! or reconnecting), and the broker redelivers an MQTT QoS 1 message it didn't see
//! acknowledged. A command must not run twice because of that, so the dispatchers drop
//! a command whose (source, `message_id`) pair they saw recently, across both
//! transports, without answering it: the app then reports the result as unknown and
//! re-reads the state.
//!
//! The source is the transport-verified one (the BLE address), never the app's. MQTT
//! has no verified source yet (contracts#8, firmware#16), so its commands are keyed with
//! an empty source and only match other MQTT commands.

use alloc::collections::VecDeque;
use alloc::vec::Vec;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// How many recent commands are remembered. The oldest is forgotten first.
pub const DEDUPE_CAPACITY: usize = 16;
/// How long a command is remembered: longer than the app waits for a response before
/// resending on another transport, and than the broker takes to redeliver after a
/// reconnect.
pub const DEDUPE_WINDOW_MS: u64 = 5 * 60 * 1_000;

struct Seen {
    source: Vec<u8>,
    message_id: u64,
    at_ms: u64,
}

/// The commands seen in the last [`DEDUPE_WINDOW_MS`], at most [`DEDUPE_CAPACITY`].
pub struct RecentCommands {
    seen: VecDeque<Seen>,
}

impl RecentCommands {
    pub const fn new() -> Self {
        Self {
            seen: VecDeque::new(),
        }
    }

    /// Whether (`source`, `message_id`) was seen in the last [`DEDUPE_WINDOW_MS`]
    /// before `now_ms`. If not, remembers it. `message_id` 0 is the proto default (not
    /// set, as in hand-written test payloads), so it is never treated as a duplicate.
    pub fn is_duplicate(&mut self, source: &[u8], message_id: u64, now_ms: u64) -> bool {
        if message_id == 0 {
            return false;
        }
        self.seen
            .retain(|seen| now_ms.saturating_sub(seen.at_ms) < DEDUPE_WINDOW_MS);
        if self
            .seen
            .iter()
            .any(|seen| seen.message_id == message_id && seen.source == source)
        {
            return true;
        }
        if self.seen.len() == DEDUPE_CAPACITY {
            self.seen.pop_front();
        }
        self.seen.push_back(Seen {
            source: Vec::from(source),
            message_id,
            at_ms: now_ms,
        });
        false
    }
}

impl Default for RecentCommands {
    fn default() -> Self {
        Self::new()
    }
}

static RECENT_COMMANDS: Mutex<CriticalSectionRawMutex, RefCell<RecentCommands>> =
    Mutex::new(RefCell::new(RecentCommands::new()));

/// [`RecentCommands::is_duplicate`] on the commands both dispatchers share, now.
pub(crate) fn is_duplicate_command(source: &[u8], message_id: u64) -> bool {
    let now_ms = embassy_time::Instant::now().as_millis();
    RECENT_COMMANDS.lock(|recent| recent.borrow_mut().is_duplicate(source, message_id, now_ms))
}

/// Forgets every command seen. For use in tests only.
pub fn reset_recent_commands_for_tests() {
    RECENT_COMMANDS.lock(|recent| *recent.borrow_mut() = RecentCommands::new());
}
