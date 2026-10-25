//! Tests for the SavvyCAN (GVRET) logger: protocol parsing and encoding, the session
//! lifecycle and its effect on the CAN read-only flag.
//! Run with `--test-threads=1`: the channels and flags are process-global.

use core_interface::{
    CAN_LOG_CHANNEL, CanFrame, CanLogFrame, CanLogSession, GvretCommand, GvretParser,
    encode_gvret_frame, is_can_logging, is_can_read_only, log_can_frame, run_can_log_session,
    set_can_read_only,
};
use embassy_futures::block_on;
use embassy_futures::yield_now;
use embedded_can::{ExtendedId, Id, StandardId};

fn frame(bus_id: u8, id: Id, data: &[u8]) -> CanFrame {
    let mut buf = [0u8; 8];
    buf[..data.len()].copy_from_slice(data);
    CanFrame {
        bus_id,
        id,
        data: buf,
        dlc: data.len() as u8,
    }
}

fn std_id(raw: u16) -> Id {
    Id::Standard(StandardId::new(raw).unwrap())
}

fn parse(bytes: &[u8]) -> Vec<GvretCommand> {
    let mut parser = GvretParser::new();
    bytes.iter().filter_map(|&b| parser.push(b)).collect()
}

fn reset() {
    while CAN_LOG_CHANNEL.try_receive().is_ok() {}
    set_can_read_only(true);
}

// ── Parser ────────────────────────────────────────────────────────────────────

#[test]
fn parses_the_savvycan_handshake() {
    // Binary mode switch, then number of buses, bus params, device info, keep-alive.
    let cmds = parse(&[
        0xE7, 0xF1, 0x0C, 0xF1, 0x06, 0xF1, 0x07, 0xF1, 0x09, 0xF1, 0x01,
    ]);
    assert_eq!(
        cmds,
        [
            GvretCommand::GetNumBuses,
            GvretCommand::GetCanBusParams,
            GvretCommand::GetDeviceInfo,
            GvretCommand::KeepAlive,
            GvretCommand::TimeSync,
        ]
    );
}

#[test]
fn setup_canbus_body_is_skipped() {
    // The 8 speed bytes contain 0xF1 and must not be read as a command.
    let cmds = parse(&[
        0xF1, 0x05, 0xF1, 0xF1, 0xF1, 0xF1, 0x0C, 0x0C, 0x0C, 0x0C, 0xF1, 0x09,
    ]);
    assert_eq!(cmds, [GvretCommand::Ignored, GvretCommand::KeepAlive]);
}

#[test]
fn transmit_requests_are_parsed_to_the_end() {
    // F1 00, id (4), bus, len, data, checksum — followed by a keep-alive.
    let cmds = parse(&[
        0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x01, 0x02, 0xAA, 0xBB, 0x00, 0xF1, 0x09,
    ]);
    assert_eq!(
        cmds,
        [
            GvretCommand::TransmitFrame { bus_id: 1 },
            GvretCommand::KeepAlive
        ]
    );
}

#[test]
fn oversized_frame_length_resyncs() {
    // A length of 200 is not classic CAN: drop the command and pick up the next one.
    let cmds = parse(&[0xF1, 0x00, 0, 0, 0, 0, 0, 200, 0xF1, 0x09]);
    assert_eq!(cmds, [GvretCommand::KeepAlive]);
}

#[test]
fn unknown_commands_are_dropped() {
    assert_eq!(parse(&[0xF1, 0x7F, 0xF1, 0x09]), [GvretCommand::KeepAlive]);
}

// ── Encoder ───────────────────────────────────────────────────────────────────

fn log_frame(bus_id: u8, id: Id, data: &[u8]) -> CanLogFrame {
    let f = frame(bus_id, id, data);
    CanLogFrame {
        timestamp_us: 0x0403_0201,
        bus_id,
        id,
        data: f.data,
        dlc: f.dlc,
    }
}

#[test]
fn encodes_a_standard_frame() {
    let mut out = Vec::new();
    assert!(encode_gvret_frame(
        &mut out,
        &log_frame(1, std_id(0x123), &[0xDE, 0xAD])
    ));
    assert_eq!(
        out,
        [
            0xF1, 0x00, 0x01, 0x02, 0x03, 0x04, 0x23, 0x01, 0x00, 0x00, 0x12, 0xDE, 0xAD, 0x00
        ]
    );
}

#[test]
fn encodes_an_extended_frame_with_the_flag_bit() {
    let mut out = Vec::new();
    let id = Id::Extended(ExtendedId::new(0x1ABC_DEF0).unwrap());
    assert!(encode_gvret_frame(&mut out, &log_frame(0, id, &[])));
    // id 0x9ABCDEF0 little-endian, bus 0, length 0.
    assert_eq!(&out[6..10], [0xF0, 0xDE, 0xBC, 0x9A]);
    assert_eq!(out[10], 0x00);
}

#[test]
fn rejects_a_bus_that_does_not_exist() {
    let mut out = Vec::new();
    assert!(!encode_gvret_frame(
        &mut out,
        &log_frame(8, std_id(1), &[1])
    ));
    assert!(!encode_gvret_frame(
        &mut out,
        &log_frame(255, std_id(1), &[1])
    ));
    assert!(out.is_empty());
}

// ── Tap ───────────────────────────────────────────────────────────────────────

#[test]
fn tap_is_a_no_op_without_a_session() {
    reset();
    log_can_frame(&frame(0, std_id(1), &[1]));
    assert!(CAN_LOG_CHANNEL.try_receive().is_err());
}

#[test]
fn tap_ignores_buses_beyond_the_maximum() {
    reset();
    let _session = CanLogSession::start().unwrap();
    log_can_frame(&frame(8, std_id(1), &[1]));
    log_can_frame(&frame(255, std_id(1), &[1]));
    assert!(CAN_LOG_CHANNEL.try_receive().is_err());
}

#[test]
fn full_channel_counts_drops_instead_of_blocking() {
    reset();
    let _session = CanLogSession::start().unwrap();
    let capacity = CAN_LOG_CHANNEL.capacity();
    for _ in 0..capacity + 16 {
        log_can_frame(&frame(0, std_id(1), &[1]));
    }
    assert_eq!(core_interface::can_log_dropped_count(), 16);
}

// ── Session and read-only ─────────────────────────────────────────────────────

#[test]
fn session_forces_read_only_and_relocks_afterwards() {
    reset();
    core_interface::confirm_car_identity();
    assert!(core_interface::confirm_car_fingerprint(b"TEST-CAR"));
    core_interface::approve_can_read_write().unwrap();
    core_interface::set_can_dev_write_unlocked(true).unwrap();
    set_can_read_only(false);
    assert!(!is_can_read_only());

    let session = CanLogSession::start().unwrap();
    assert!(is_can_logging());
    // Relocked on start, and the vehicle can't unlock while logging.
    assert!(is_can_read_only());
    set_can_read_only(false);
    assert!(is_can_read_only());

    // A second client is refused.
    assert!(CanLogSession::start().is_none());

    // The vehicle's own unlock request doesn't survive the session: it has to
    // validate the car again.
    drop(session);
    assert!(!is_can_logging());
    assert!(is_can_read_only());
}

/// A scripted client: yields `script`, then waits until `until` is satisfied by what
/// the device has written, then disconnects.
struct MockClient {
    script: Vec<u8>,
    sent: bool,
    written: Vec<u8>,
    until: fn(&[u8]) -> bool,
    inject: Option<CanFrame>,
}

impl embedded_io_async::ErrorType for MockClient {
    type Error = core::convert::Infallible;
}

impl embedded_io_async::Read for MockClient {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if !self.sent {
            self.sent = true;
            // The session is running by now, so the tap works.
            if let Some(f) = self.inject.take() {
                log_can_frame(&f);
            }
            buf[..self.script.len()].copy_from_slice(&self.script);
            return Ok(self.script.len());
        }
        while !(self.until)(&self.written) {
            yield_now().await;
        }
        Ok(0)
    }
}

impl embedded_io_async::Write for MockClient {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.written.extend_from_slice(buf);
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn session_answers_the_handshake_and_streams_frames() {
    reset();
    let mut client = MockClient {
        script: vec![0xE7, 0xF1, 0x0C, 0xF1, 0x09],
        sent: false,
        written: Vec::new(),
        until: |w| w.len() >= 4 + 14,
        inject: Some(frame(1, std_id(0x7E8), &[1, 2, 3])),
    };
    let end = block_on(run_can_log_session(&mut client, 2));
    assert!(end.started);
    assert_eq!(end.dropped, 0);
    assert_eq!(client.written[..7], [0xF1, 0x0C, 2, 0xF1, 0x09, 0xDE, 0xAD]);
    // Then the tapped frame: bus 1, id 0x7E8, 3 data bytes.
    let f = &client.written[7..];
    assert_eq!(&f[..2], [0xF1, 0x00]);
    assert_eq!(&f[6..10], [0xE8, 0x07, 0x00, 0x00]);
    assert_eq!(f[10], 0x13);
    assert_eq!(&f[11..14], [1, 2, 3]);

    // Disconnect stops the logging.
    assert!(!is_can_logging());
}

#[test]
fn session_ignores_transmit_requests() {
    reset();
    let mut client = MockClient {
        script: vec![0xF1, 0x00, 0x23, 0x01, 0x00, 0x00, 0x00, 0x01, 0xAA, 0x00],
        sent: false,
        written: Vec::new(),
        until: |_| true,
        inject: None,
    };
    let end = block_on(run_can_log_session(&mut client, 1));
    assert_eq!(end.tx_requests_ignored, 1);
    assert!(client.written.is_empty());
    assert!(
        core_interface::can_tx_channel(0)
            .unwrap()
            .try_receive()
            .is_err()
    );
}

#[test]
fn second_client_is_refused_while_one_is_connected() {
    reset();
    let _session = CanLogSession::start().unwrap();
    let mut client = MockClient {
        script: vec![],
        sent: false,
        written: Vec::new(),
        until: |_| true,
        inject: None,
    };
    let end = block_on(run_can_log_session(&mut client, 1));
    assert!(!end.started);
    // The refused client didn't stop the first one's logging.
    assert!(is_can_logging());
}
