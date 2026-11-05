//! Host tests for the AT channel against a scripted modem.

use std::collections::VecDeque;
use std::string::String;
use std::vec::Vec;

use embassy_futures::block_on;
use embedded_io_async::{BufRead, ErrorType, Read, ReadReady, Write};

use super::at::Final;
use super::channel::{AtChannel, BufferedIo, ChannelError};

/// A modem that answers each command line with a canned reply. Bytes queued with
/// `pending` are already waiting before the next command is written. A read with
/// nothing to return reports end of input instead of blocking.
#[derive(Default)]
struct ScriptedModem {
    replies: VecDeque<(&'static str, &'static str)>,
    rx: VecDeque<u8>,
    /// Hands out at most this many bytes per read, to split lines across reads.
    max_read: usize,
    written: Vec<u8>,
    line: Vec<u8>,
}

impl ScriptedModem {
    fn new(replies: &[(&'static str, &'static str)]) -> Self {
        Self {
            replies: replies.iter().copied().collect(),
            max_read: usize::MAX,
            ..Default::default()
        }
    }

    fn pending(mut self, bytes: &str) -> Self {
        self.rx.extend(bytes.bytes());
        self
    }

    fn written(&self) -> String {
        String::from_utf8(self.written.clone()).unwrap()
    }
}

impl ErrorType for ScriptedModem {
    type Error = core::convert::Infallible;
}

impl Read for ScriptedModem {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = buf.len().min(self.rx.len()).min(self.max_read);
        for b in buf.iter_mut().take(n) {
            *b = self.rx.pop_front().unwrap();
        }
        Ok(n)
    }
}

impl embedded_io::Read for ScriptedModem {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        block_on(Read::read(self, buf))
    }
}

impl ReadReady for ScriptedModem {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.rx.is_empty())
    }
}

impl Write for ScriptedModem {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.written.extend_from_slice(buf);
        for &b in buf {
            if b == b'\r' {
                let cmd = String::from_utf8(core::mem::take(&mut self.line)).unwrap();
                let (expected, reply) = self.replies.pop_front().expect("unexpected command");
                assert_eq!(cmd, expected);
                self.rx.extend(reply.bytes());
            } else {
                self.line.push(b);
            }
        }
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn channel(modem: ScriptedModem) -> AtChannel<ScriptedModem, 16> {
    AtChannel::new(modem)
}

#[test]
fn command_returns_final_result_and_info_lines() {
    let mut at = channel(ScriptedModem::new(&[(
        "AT+CEREG?",
        "\r\n+CEREG: 0,1\r\n\r\nOK\r\n",
    )]));
    let mut lines = Vec::new();
    let result = block_on(at.command("AT+CEREG?", |l| lines.push(l.to_string())));
    assert_eq!(result, Ok(Final::Ok));
    assert_eq!(lines, ["+CEREG: 0,1"]);
}

#[test]
fn echo_is_skipped() {
    let mut at = channel(ScriptedModem::new(&[("AT", "AT\r\r\nOK\r\n")]));
    let mut lines = Vec::new();
    assert_eq!(
        block_on(at.command("AT", |l| lines.push(l.to_string()))),
        Ok(Final::Ok)
    );
    assert!(lines.is_empty());
}

#[test]
fn errors_and_connect_end_the_command() {
    let mut at = channel(ScriptedModem::new(&[
        ("AT+CPIN?", "\r\n+CME ERROR: 10\r\n"),
        ("ATD*99#", "\r\nCONNECT 115200\r\n"),
    ]));
    assert_eq!(block_on(at.command("AT+CPIN?", |_| {})), Ok(Final::Error));
    assert_eq!(block_on(at.command("ATD*99#", |_| {})), Ok(Final::Connect));
}

#[test]
fn lines_split_across_reads_are_joined() {
    let mut modem = ScriptedModem::new(&[("AT+CSQ", "\r\n+CSQ: 20,99\r\n\r\nOK\r\n")]);
    modem.max_read = 3;
    let mut at = channel(modem);
    let mut lines = Vec::new();
    assert_eq!(
        block_on(at.command("AT+CSQ", |l| lines.push(l.to_string()))),
        Ok(Final::Ok)
    );
    assert_eq!(lines, ["+CSQ: 20,99"]);
}

#[test]
fn stale_input_is_dropped_before_a_command() {
    // An `OK` left over from a command that timed out must not answer the next one.
    let modem = ScriptedModem::new(&[("AT+CPIN?", "\r\n+CPIN: READY\r\n\r\nOK\r\n")])
        .pending("\r\nOK\r\n+CEREG: 1\r\n");
    let mut at = channel(modem);
    let mut lines = Vec::new();
    assert_eq!(
        block_on(at.command("AT+CPIN?", |l| lines.push(l.to_string()))),
        Ok(Final::Ok)
    );
    assert_eq!(lines, ["+CPIN: READY"]);
}

#[test]
fn long_lines_are_truncated_not_split() {
    let long: &'static str = Box::leak(
        format!(
            "\r\n+X: {}\r\nOK\r\n",
            "a".repeat(super::at::MAX_LINE_LEN * 2)
        )
        .into_boxed_str(),
    );
    let mut at = channel(ScriptedModem::new(&[("AT+X", long)]));
    let mut lines = Vec::new();
    assert_eq!(
        block_on(at.command("AT+X", |l| lines.push(l.to_string()))),
        Ok(Final::Ok)
    );
    assert_eq!(lines.len(), 1);
    assert_eq!(lines[0].len(), super::at::MAX_LINE_LEN);
}

#[test]
fn noise_bytes_are_dropped() {
    let modem = ScriptedModem::new(&[]).pending("\u{0}\u{ff}RDY\r\n");
    let mut at = channel(modem);
    // `pending` bytes count as unread input, so read them directly.
    assert_eq!(block_on(at.read_line()).unwrap().as_str(), "RDY");
}

#[test]
fn end_of_input_is_reported() {
    let mut at = channel(ScriptedModem::new(&[("AT", "\r\nOK")]));
    assert_eq!(block_on(at.command("AT", |_| {})), Err(ChannelError::Eof));
}

#[test]
fn data_mode_keeps_bytes_after_connect() {
    // PPP frames the modem sends right after CONNECT are already in the buffer.
    let mut at = channel(ScriptedModem::new(&[(
        "ATD*99#",
        "\r\nCONNECT 115200\r\n~\u{7d}#",
    )]));
    assert_eq!(block_on(at.command("ATD*99#", |_| {})), Ok(Final::Connect));
    let io = at.data_mode();
    let data = block_on(io.fill_buf()).unwrap().to_vec();
    assert_eq!(data, b"~}#");
}

#[test]
fn commands_and_raw_bytes_reach_the_port() {
    let mut at = channel(ScriptedModem::new(&[("ATE0", "\r\nOK\r\n")]));
    block_on(at.command("ATE0", |_| {})).unwrap();
    block_on(at.write_raw(b"+++")).unwrap();
    assert_eq!(at.port().written(), "ATE0\r+++");
}

#[test]
fn buffered_io_reads_through_the_buffer() {
    let modem = ScriptedModem::new(&[]).pending("abcdef");
    let mut io: BufferedIo<ScriptedModem, 4> = BufferedIo::new(modem);
    let mut out = [0u8; 3];
    assert_eq!(block_on(Read::read(&mut io, &mut out)).unwrap(), 3);
    assert_eq!(&out, b"abc");
    assert_eq!(block_on(io.fill_buf()).unwrap(), b"d");
    io.consume(1);
    assert_eq!(block_on(io.fill_buf()).unwrap(), b"ef");
    io.consume(2);
    assert_eq!(block_on(io.fill_buf()).unwrap(), b"");
}
