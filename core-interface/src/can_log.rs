//! CAN logging for SavvyCAN, using its network protocol (GVRET binary mode over TCP).
//!
//! The board crates own the TCP listener and call [`run_can_log_session`] for each
//! accepted client; this module holds everything else so it stays host-testable:
//!
//! * the GVRET parser and frame encoder,
//! * a dedicated frame channel ([`CAN_LOG_CHANNEL`]), kept apart from the vehicle and
//!   BLE/MQTT queues so a slow SavvyCAN client can never stall them,
//! * the session state: while a client is connected, the buses are forced read-only and
//!   the board drivers stop feeding the vehicle (see [`is_can_logging`]).
//!
//! Frames sent by SavvyCAN are parsed and discarded: logging is receive-only.

use alloc::vec::Vec;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, with_timeout};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use crate::channels::MAX_CAN_BUSES;
use crate::types::CanFrame;

// ── State and channel ─────────────────────────────────────────────────────────

/// `true` while a SavvyCAN client is connected.
static CAN_LOG_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Frames dropped since the session started because [`CAN_LOG_CHANNEL`] was full.
static CAN_LOG_DROPPED: AtomicU32 = AtomicU32::new(0);

/// A received frame with a microsecond timestamp, as SavvyCAN wants it.
pub struct CanLogFrame {
    /// Device uptime in microseconds, truncated to 32 bits (wraps every ~71 minutes,
    /// which is what GVRET expects).
    pub timestamp_us: u32,
    pub bus_id: u8,
    pub id: embedded_can::Id,
    pub data: [u8; 8],
    pub dlc: u8,
}

/// Raw frames from the board CAN drivers to the active logging session. Only filled
/// while a session is active, and only ever read by it.
pub static CAN_LOG_CHANNEL: Channel<CriticalSectionRawMutex, CanLogFrame, 64> = Channel::new();

/// Returns `true` while a SavvyCAN client is connected.
///
/// Board CAN drivers use this (lock-free) to decide whether to tap frames, and to stop
/// forwarding frames to the vehicle while the logger has the buses.
pub fn is_can_logging() -> bool {
    CAN_LOG_ACTIVE.load(Ordering::Relaxed)
}

/// Frames dropped in the current (or last) session because the client could not keep up.
pub fn can_log_dropped_count() -> u32 {
    CAN_LOG_DROPPED.load(Ordering::Relaxed)
}

/// Taps a received frame into the logging session. Board drivers call this for every
/// frame they receive, before any vehicle filtering. It never blocks: a full channel
/// drops the frame and counts it. Frames for a bus that doesn't exist are ignored.
pub fn log_can_frame(frame: &CanFrame) {
    if !is_can_logging() || frame.bus_id as usize >= MAX_CAN_BUSES {
        return;
    }
    let log_frame = CanLogFrame {
        timestamp_us: Instant::now().as_micros() as u32,
        bus_id: frame.bus_id,
        id: frame.id,
        data: frame.data,
        dlc: frame.dlc.min(8),
    };
    if CAN_LOG_CHANNEL.try_send(log_frame).is_err() {
        CAN_LOG_DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}

/// Marks a logging session as active for as long as it lives.
///
/// Starting one relocks the CAN buses (the vehicle has to validate the car again
/// afterwards) and keeps them read-only until it is dropped, even if the vehicle asks
/// to unlock. Dropping it, including when the client disconnects, stops the logging.
pub struct CanLogSession(());

impl CanLogSession {
    /// Returns `None` if a session is already active.
    pub fn start() -> Option<Self> {
        CAN_LOG_ACTIVE
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
            .ok()?;
        while CAN_LOG_CHANNEL.try_receive().is_ok() {}
        CAN_LOG_DROPPED.store(0, Ordering::Relaxed);
        crate::can::CAN_READ_ONLY.store(true, Ordering::Relaxed);
        crate::can::notify_can_read_only_changed();
        Some(Self(()))
    }
}

impl Drop for CanLogSession {
    fn drop(&mut self) {
        CAN_LOG_ACTIVE.store(false, Ordering::Release);
        while CAN_LOG_CHANNEL.try_receive().is_ok() {}
        // Still read-only: the relock from `start` stays until the vehicle unlocks again.
        crate::can::notify_can_read_only_changed();
    }
}

// ── GVRET protocol ────────────────────────────────────────────────────────────

const HEADER: u8 = 0xF1;

const CMD_FRAME: u8 = 0;
const CMD_TIME_SYNC: u8 = 1;
const CMD_DIG_INPUTS: u8 = 2;
const CMD_ANA_INPUTS: u8 = 3;
const CMD_SET_DIG_OUT: u8 = 4;
const CMD_SETUP_CANBUS: u8 = 5;
const CMD_GET_CANBUS_PARAMS: u8 = 6;
const CMD_GET_DEVICE_INFO: u8 = 7;
const CMD_SET_SINGLEWIRE_MODE: u8 = 8;
const CMD_KEEP_ALIVE: u8 = 9;
const CMD_SET_SYSTEM_TYPE: u8 = 10;
const CMD_ECHO_FRAME: u8 = 11;
const CMD_GET_NUM_BUSES: u8 = 12;

/// Bit 31 of a GVRET frame id marks an extended (29-bit) id.
const EXTENDED_FLAG: u32 = 1 << 31;

/// A command from SavvyCAN that needs an answer or is worth knowing about.
#[derive(Debug, PartialEq, Eq)]
pub enum GvretCommand {
    TimeSync,
    GetCanBusParams,
    GetDeviceInfo,
    KeepAlive,
    GetNumBuses,
    /// SavvyCAN asked to transmit a frame. Never acted on: logging is receive-only.
    TransmitFrame {
        bus_id: u8,
    },
    /// A command we parse (to stay in sync) but have nothing to do for.
    Ignored,
}

enum ParseState {
    Idle,
    Cmd,
    /// Collecting a command's fixed-size body.
    Body {
        need: usize,
    },
    /// Collecting a frame: 6 header bytes (id, bus, length), then the data and checksum.
    Frame {
        cmd: u8,
        need: usize,
    },
}

/// Incremental parser for the bytes SavvyCAN sends. Feed it one byte at a time; bytes
/// outside a `0xF1` command (such as the `0xE7` binary-mode switch) are skipped.
pub struct GvretParser {
    state: ParseState,
    buf: [u8; 16],
    have: usize,
}

impl Default for GvretParser {
    fn default() -> Self {
        Self::new()
    }
}

impl GvretParser {
    pub const fn new() -> Self {
        Self {
            state: ParseState::Idle,
            buf: [0; 16],
            have: 0,
        }
    }

    pub fn push(&mut self, byte: u8) -> Option<GvretCommand> {
        match self.state {
            ParseState::Idle => {
                if byte == HEADER {
                    self.state = ParseState::Cmd;
                }
                None
            }
            ParseState::Cmd => self.start_command(byte),
            ParseState::Body { need } => {
                self.have += 1;
                if self.have < need {
                    return None;
                }
                self.state = ParseState::Idle;
                Some(GvretCommand::Ignored)
            }
            ParseState::Frame { cmd, need } => {
                if self.have < self.buf.len() {
                    self.buf[self.have] = byte;
                }
                self.have += 1;
                if self.have == 6 {
                    // id (4) + bus (1) + length (1): classic CAN is at most 8 bytes.
                    let len = self.buf[5] as usize;
                    if len > 8 {
                        self.state = ParseState::Idle;
                        return None;
                    }
                    self.state = ParseState::Frame {
                        cmd,
                        need: 6 + len + 1,
                    };
                    return None;
                }
                if self.have < need {
                    return None;
                }
                self.state = ParseState::Idle;
                if cmd == CMD_FRAME || cmd == CMD_ECHO_FRAME {
                    Some(GvretCommand::TransmitFrame {
                        bus_id: self.buf[4],
                    })
                } else {
                    Some(GvretCommand::Ignored)
                }
            }
        }
    }

    fn start_command(&mut self, cmd: u8) -> Option<GvretCommand> {
        self.have = 0;
        let body = |need| ParseState::Body { need };
        match cmd {
            CMD_FRAME | CMD_ECHO_FRAME => {
                // Not a valid length yet; replaced once the length byte arrives.
                self.state = ParseState::Frame {
                    cmd,
                    need: usize::MAX,
                };
                None
            }
            CMD_SET_DIG_OUT | CMD_SET_SINGLEWIRE_MODE | CMD_SET_SYSTEM_TYPE => {
                self.state = body(1);
                None
            }
            CMD_SETUP_CANBUS => {
                self.state = body(8);
                None
            }
            CMD_DIG_INPUTS | CMD_ANA_INPUTS => {
                self.state = ParseState::Idle;
                Some(GvretCommand::Ignored)
            }
            _ => {
                self.state = ParseState::Idle;
                let command = match cmd {
                    CMD_TIME_SYNC => GvretCommand::TimeSync,
                    CMD_GET_CANBUS_PARAMS => GvretCommand::GetCanBusParams,
                    CMD_GET_DEVICE_INFO => GvretCommand::GetDeviceInfo,
                    CMD_KEEP_ALIVE => GvretCommand::KeepAlive,
                    CMD_GET_NUM_BUSES => GvretCommand::GetNumBuses,
                    // Unknown or unsupported (e.g. external buses): resync on the next header.
                    _ => return None,
                };
                Some(command)
            }
        }
    }
}

/// Appends `frame` in GVRET's receive format to `out`. Returns `false` (appending
/// nothing) if the frame's bus can't be represented, so a bad `bus_id` never reaches
/// the client.
pub fn encode_gvret_frame(out: &mut Vec<u8>, frame: &CanLogFrame) -> bool {
    if frame.bus_id as usize >= MAX_CAN_BUSES {
        return false;
    }
    let dlc = frame.dlc.min(8);
    let id = match frame.id {
        embedded_can::Id::Standard(sid) => sid.as_raw() as u32,
        embedded_can::Id::Extended(eid) => eid.as_raw() | EXTENDED_FLAG,
    };
    out.extend_from_slice(&[HEADER, CMD_FRAME]);
    out.extend_from_slice(&frame.timestamp_us.to_le_bytes());
    out.extend_from_slice(&id.to_le_bytes());
    out.push((frame.bus_id << 4) | dlc);
    out.extend_from_slice(&frame.data[..dlc as usize]);
    out.push(0); // checksum, unused
    true
}

fn encode_reply(out: &mut Vec<u8>, command: &GvretCommand, num_buses: u8) {
    match command {
        GvretCommand::TimeSync => {
            out.extend_from_slice(&[HEADER, CMD_TIME_SYNC]);
            out.extend_from_slice(&(Instant::now().as_micros() as u32).to_le_bytes());
        }
        GvretCommand::GetCanBusParams => {
            out.extend_from_slice(&[HEADER, CMD_GET_CANBUS_PARAMS]);
            // Two buses: flags (bit 0 enabled, bit 4 listen-only), then the speed. The
            // speed is configured at build time and not known here, so it reads 0. The
            // buses are always listen-only while SavvyCAN is connected.
            for bus in 0..2u8 {
                let flags = if bus < num_buses { 1 | (1 << 4) } else { 0 };
                out.push(flags);
                out.extend_from_slice(&0u32.to_le_bytes());
            }
        }
        GvretCommand::GetDeviceInfo => {
            // Build number (2 bytes), settings version, then three unused bytes.
            out.extend_from_slice(&[HEADER, CMD_GET_DEVICE_INFO, 1, 0, 0x20, 0, 0, 0]);
        }
        GvretCommand::KeepAlive => out.extend_from_slice(&[HEADER, CMD_KEEP_ALIVE, 0xDE, 0xAD]),
        GvretCommand::GetNumBuses => out.extend_from_slice(&[
            HEADER,
            CMD_GET_NUM_BUSES,
            num_buses.min(MAX_CAN_BUSES as u8),
        ]),
        GvretCommand::TransmitFrame { .. } | GvretCommand::Ignored => {}
    }
}

// ── Session ───────────────────────────────────────────────────────────────────

/// How a [`run_can_log_session`] ended.
#[derive(Debug, PartialEq, Eq)]
pub struct SessionEnd {
    /// `false` if another session was already running and this one never started.
    pub started: bool,
    /// Frames dropped because the client couldn't keep up.
    pub dropped: u32,
    /// Transmit requests from SavvyCAN that were discarded.
    pub tx_requests_ignored: u32,
}

/// A write that doesn't finish in this long means the client is gone or stalled.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);

/// Serves one SavvyCAN client on `stream` until it disconnects or errors.
///
/// The session owns the buses meanwhile (see [`CanLogSession`]): logging stops as soon
/// as this returns, however it ends. `num_buses` is the number of configured CAN buses.
pub async fn run_can_log_session<S>(stream: &mut S, num_buses: u8) -> SessionEnd
where
    S: embedded_io_async::Read + embedded_io_async::Write,
{
    let mut end = SessionEnd {
        started: false,
        dropped: 0,
        tx_requests_ignored: 0,
    };
    let Some(_session) = CanLogSession::start() else {
        return end;
    };
    end.started = true;

    let mut parser = GvretParser::new();
    let mut rx = [0u8; 64];
    let mut out: Vec<u8> = Vec::with_capacity(512);

    loop {
        match select(stream.read(&mut rx), CAN_LOG_CHANNEL.receive()).await {
            Either::First(Ok(n)) if n > 0 => {
                for &byte in &rx[..n] {
                    match parser.push(byte) {
                        Some(GvretCommand::TransmitFrame { .. }) => end.tx_requests_ignored += 1,
                        Some(command) => encode_reply(&mut out, &command, num_buses),
                        None => {}
                    }
                }
            }
            // Disconnect (0 bytes) or a read error.
            Either::First(_) => break,
            Either::Second(first) => {
                // Batch what is already queued into one write.
                let mut next = Some(first);
                while let Some(frame) = next.take() {
                    // A bus beyond the configured ones can't come from a board driver.
                    if frame.bus_id < num_buses {
                        encode_gvret_frame(&mut out, &frame);
                    }
                    if out.len() >= 400 {
                        break;
                    }
                    next = CAN_LOG_CHANNEL.try_receive().ok();
                }
            }
        }

        if !out.is_empty() {
            let written = with_timeout(WRITE_TIMEOUT, stream.write_all(&out)).await;
            out.clear();
            if !matches!(written, Ok(Ok(()))) {
                break;
            }
        }
    }

    end.dropped = can_log_dropped_count();
    end
}
