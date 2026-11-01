//! A minimal diagnostic client over ISO-TP (ISO 15765-2) on classic CAN, normal
//! addressing: one single-frame request (UDS `22 F18C`, OBD `09 02`, ...) and its
//! answer, single or multi-frame.
//!
//! It does no I/O. The vehicle sends [`UdsRequest::request_frame`], feeds every frame
//! from the response ID to [`UdsRequest::on_frame`], sends
//! [`UdsRequest::flow_control_frame`] when asked, and gives up when no frame arrives
//! within [`UdsRequest::timeout_ms`]. Before the car is identified, the frames it sends
//! can only go out through an identity probe (see `can_probe.rs`), which allows exactly
//! the frames [`uds_single_frame`] and [`iso_tp_flow_control`] build.

/// The largest answer kept (service byte, the request's parameters and the value).
/// Longer answers fail with [`UdsError::TooLong`] before any flow control is sent.
pub const UDS_MAX_RESPONSE: usize = 128;
/// The longest request: what fits in one single frame.
pub const UDS_MAX_REQUEST: usize = 7;

/// How long to wait for the answer, and for each consecutive frame (ISO-TP N_Cr).
const RESPONSE_TIMEOUT_MS: u64 = 1_000;
/// How long to wait after the ECU answered "response pending" (UDS P2*).
const PENDING_TIMEOUT_MS: u64 = 5_000;

const NEGATIVE_RESPONSE: u8 = 0x7F;
const POSITIVE_RESPONSE_OFFSET: u8 = 0x40;
const NRC_RESPONSE_PENDING: u8 = 0x78;

/// The single frame carrying `request` (service byte, then its parameters), the rest
/// filled with `padding`: `uds_single_frame(&[0x22, 0xF1, 0x8C], 0x00)` is
/// `03 22 F1 8C 00 00 00 00`. Fails to build (at compile time in a `const`) for an
/// empty request or one longer than [`UDS_MAX_REQUEST`].
pub const fn uds_single_frame(request: &[u8], padding: u8) -> [u8; 8] {
    assert!(
        !request.is_empty() && request.len() <= UDS_MAX_REQUEST,
        "a request must fit in a single frame"
    );
    let mut frame = [padding; 8];
    frame[0] = request.len() as u8;
    let mut i = 0;
    while i < request.len() {
        frame[1 + i] = request[i];
        i += 1;
    }
    frame
}

/// The flow control sent after a first frame: send everything (block size 0), with no
/// minimum gap between frames (STmin 0), the rest filled with `padding`.
pub const fn iso_tp_flow_control(padding: u8) -> [u8; 8] {
    let mut frame = [padding; 8];
    frame[0] = 0x30;
    frame[1] = 0x00;
    frame[2] = 0x00;
    frame
}

/// Why a request failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UdsError {
    /// The ECU refused with this negative response code (e.g. `0x31`: out of range).
    Negative(u8),
    /// A frame that doesn't follow ISO-TP, or an answer to another request.
    Malformed,
    /// A consecutive frame out of order: one was lost.
    Sequence,
    /// The answer is longer than [`UDS_MAX_RESPONSE`].
    TooLong,
}

/// What to do after a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UdsStep {
    /// Keep waiting, within [`UdsRequest::timeout_ms`].
    Wait,
    /// Send [`UdsRequest::flow_control_frame`] now, then keep waiting.
    SendFlowControl,
    /// The answer is complete: read it with [`UdsRequest::value`].
    Done,
    /// The request failed; nothing more will come.
    Failed(UdsError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    AwaitingResponse { pending: bool },
    AwaitingConsecutive,
    Done,
    Failed(UdsError),
}

/// One request and its answer.
pub struct UdsRequest {
    request: [u8; UDS_MAX_REQUEST],
    request_len: usize,
    padding: u8,
    state: State,
    buf: [u8; UDS_MAX_RESPONSE],
    len: usize,
    total: usize,
    next_sequence: u8,
}

impl UdsRequest {
    /// `request` is the service byte and its parameters (`[0x22, 0xF1, 0x8C]`), and
    /// `padding` fills the unused bytes of the frames sent. Panics for an empty request or
    /// one longer than [`UDS_MAX_REQUEST`].
    pub const fn new(request: &[u8], padding: u8) -> Self {
        assert!(
            !request.is_empty() && request.len() <= UDS_MAX_REQUEST,
            "a request must fit in a single frame"
        );
        let mut copy = [0; UDS_MAX_REQUEST];
        let mut i = 0;
        while i < request.len() {
            copy[i] = request[i];
            i += 1;
        }
        Self {
            request: copy,
            request_len: request.len(),
            padding,
            state: State::AwaitingResponse { pending: false },
            buf: [0; UDS_MAX_RESPONSE],
            len: 0,
            total: 0,
            next_sequence: 1,
        }
    }

    fn request(&self) -> &[u8] {
        &self.request[..self.request_len]
    }

    /// The request frame to send first (see [`uds_single_frame`]).
    pub fn request_frame(&self) -> [u8; 8] {
        uds_single_frame(self.request(), self.padding)
    }

    /// The flow control to send on [`UdsStep::SendFlowControl`].
    pub fn flow_control_frame(&self) -> [u8; 8] {
        iso_tp_flow_control(self.padding)
    }

    /// How long to wait for the next frame before giving up.
    pub fn timeout_ms(&self) -> u64 {
        match self.state {
            State::AwaitingResponse { pending: true } => PENDING_TIMEOUT_MS,
            _ => RESPONSE_TIMEOUT_MS,
        }
    }

    /// The answer after the positive response byte and the request's parameters it
    /// repeats (`62 F1 8C <value>`, `49 02 <value>`), once the request is done.
    pub fn value(&self) -> Option<&[u8]> {
        (self.state == State::Done).then(|| &self.buf[self.request_len..self.total])
    }

    /// Handles one frame received on the response ID.
    pub fn on_frame(&mut self, data: &[u8]) -> UdsStep {
        let step = match self.state {
            State::AwaitingResponse { .. } => self.on_response(data),
            State::AwaitingConsecutive => self.on_consecutive(data),
            State::Done => return UdsStep::Done,
            State::Failed(e) => return UdsStep::Failed(e),
        };
        match step {
            Ok(step) => step,
            Err(e) => {
                self.state = State::Failed(e);
                UdsStep::Failed(e)
            }
        }
    }

    fn on_response(&mut self, data: &[u8]) -> Result<UdsStep, UdsError> {
        let pci = *data.first().ok_or(UdsError::Malformed)?;
        match pci >> 4 {
            // Single frame: the whole answer.
            0x0 => {
                let len = (pci & 0x0F) as usize;
                let payload = data.get(1..1 + len).ok_or(UdsError::Malformed)?;
                if let [NEGATIVE_RESPONSE, service, nrc] = *payload {
                    if service != self.request[0] {
                        return Err(UdsError::Malformed);
                    }
                    if nrc == NRC_RESPONSE_PENDING {
                        self.state = State::AwaitingResponse { pending: true };
                        return Ok(UdsStep::Wait);
                    }
                    return Err(UdsError::Negative(nrc));
                }
                self.store(payload)?;
                self.total = len;
                self.finish()
            }
            // First frame: the length, then the first 6 bytes.
            0x1 => {
                let low = *data.get(1).ok_or(UdsError::Malformed)?;
                let total = (((pci & 0x0F) as usize) << 8) | low as usize;
                let first = data.get(2..8).ok_or(UdsError::Malformed)?;
                if total <= 7 {
                    return Err(UdsError::Malformed);
                }
                if total > UDS_MAX_RESPONSE {
                    return Err(UdsError::TooLong);
                }
                self.total = total;
                self.store(first)?;
                self.state = State::AwaitingConsecutive;
                Ok(UdsStep::SendFlowControl)
            }
            // A flow control on the response ID isn't for us.
            0x3 => Ok(UdsStep::Wait),
            _ => Err(UdsError::Malformed),
        }
    }

    fn on_consecutive(&mut self, data: &[u8]) -> Result<UdsStep, UdsError> {
        let pci = *data.first().ok_or(UdsError::Malformed)?;
        if pci >> 4 != 0x2 {
            return Err(UdsError::Malformed);
        }
        if pci & 0x0F != self.next_sequence {
            return Err(UdsError::Sequence);
        }
        self.next_sequence = (self.next_sequence + 1) & 0x0F;
        let take = (self.total - self.len).min(7);
        self.store(data.get(1..1 + take).ok_or(UdsError::Malformed)?)?;
        if self.len < self.total {
            return Ok(UdsStep::Wait);
        }
        self.finish()
    }

    fn store(&mut self, bytes: &[u8]) -> Result<(), UdsError> {
        let end = self.len + bytes.len();
        self.buf
            .get_mut(self.len..end)
            .ok_or(UdsError::TooLong)?
            .copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }

    /// Checks that the answer is the positive response to this request: the service byte
    /// plus 0x40, then the request's parameters repeated.
    fn finish(&mut self) -> Result<UdsStep, UdsError> {
        let request = &self.request[..self.request_len];
        let answer = &self.buf[..self.total];
        if answer.len() < request.len()
            || answer[0] != request[0].wrapping_add(POSITIVE_RESPONSE_OFFSET)
            || answer[1..request.len()] != request[1..]
        {
            return Err(UdsError::Malformed);
        }
        self.state = State::Done;
        Ok(UdsStep::Done)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const READ_SERIAL: [u8; 3] = [0x22, 0xF1, 0x8C];

    /// The head unit (0x788) answering 22 F18C on an EV6, in Ready, ACC and off
    /// (2026-10-31, the same frames as Car Scanner's log_41).
    const SERIAL_FRAMES: [[u8; 8]; 6] = [
        [0x10, 0x29, 0x62, 0xF1, 0x8C, 0x32, 0x34, 0x30],
        [0x21, 0x34, 0x30, 0x34, 0x41, 0x30, 0x34, 0x20],
        [0x22, 0x4A, 0x58, 0x49, 0x4F, 0x34, 0x30, 0x30],
        [0x23, 0x30, 0x32, 0x33, 0x00, 0x00, 0x00, 0x00],
        [0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        [0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    ];

    #[test]
    fn frames_are_padded() {
        let request = UdsRequest::new(&READ_SERIAL, 0x00);
        assert_eq!(
            request.request_frame(),
            [0x03, 0x22, 0xF1, 0x8C, 0x00, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            request.flow_control_frame(),
            [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
        let request = UdsRequest::new(&[0x09, 0x02], 0xAA);
        assert_eq!(
            request.request_frame(),
            [0x02, 0x09, 0x02, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA]
        );
        assert_eq!(
            request.flow_control_frame(),
            [0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA]
        );
        // The same frames, built for a probe declared at compile time.
        const FRAME: [u8; 8] = uds_single_frame(&READ_SERIAL, 0x00);
        assert_eq!(FRAME, UdsRequest::new(&READ_SERIAL, 0x00).request_frame());
    }

    #[test]
    fn reads_the_head_unit_serial() {
        let mut request = UdsRequest::new(&READ_SERIAL, 0x00);
        assert_eq!(
            request.on_frame(&SERIAL_FRAMES[0]),
            UdsStep::SendFlowControl
        );
        for frame in &SERIAL_FRAMES[1..5] {
            assert_eq!(request.on_frame(frame), UdsStep::Wait);
            assert_eq!(request.value(), None);
        }
        assert_eq!(request.on_frame(&SERIAL_FRAMES[5]), UdsStep::Done);

        let value = request.value().unwrap();
        assert_eq!(value.len(), 0x29 - 3);
        assert_eq!(&value[..20], b"240404A04 JXIO400023");
        assert!(value[20..].iter().all(|&b| b == 0), "padding");
        assert_eq!(
            request.on_frame(&SERIAL_FRAMES[5]),
            UdsStep::Done,
            "stays done"
        );
    }

    #[test]
    fn reads_an_obd_vin() {
        // Mode 09 PID 02: 49 02, the number of data items (1), then the VIN.
        let mut request = UdsRequest::new(&[0x09, 0x02], 0x00);
        let frames = [
            [0x10, 0x14, 0x49, 0x02, 0x01, b'K', b'N', b'A'],
            [0x21, b'C', b'3', b'8', b'1', b'A', b'A', b'P'],
            [0x22, b'5', b'0', b'0', b'0', b'0', b'0', b'1'],
        ];
        assert_eq!(request.on_frame(&frames[0]), UdsStep::SendFlowControl);
        assert_eq!(request.on_frame(&frames[1]), UdsStep::Wait);
        assert_eq!(request.on_frame(&frames[2]), UdsStep::Done);
        assert_eq!(request.value(), Some(&b"\x01KNAC381AAP5000001"[..]));
    }

    #[test]
    fn reads_a_single_frame_answer() {
        let mut request = UdsRequest::new(&[0x22, 0xF1, 0x00], 0x00);
        assert_eq!(
            request.on_frame(&[0x05, 0x62, 0xF1, 0x00, 0xAB, 0xCD, 0x00, 0x00]),
            UdsStep::Done
        );
        assert_eq!(request.value(), Some(&[0xAB, 0xCD][..]));
    }

    #[test]
    fn a_refusal_fails_the_request() {
        // The head unit's answer to 22 F190 (the VIN) in log_41.
        let mut request = UdsRequest::new(&[0x22, 0xF1, 0x90], 0x00);
        assert_eq!(
            request.on_frame(&[0x03, 0x7F, 0x22, 0x31, 0x00, 0x00, 0x00, 0x00]),
            UdsStep::Failed(UdsError::Negative(0x31))
        );
        assert_eq!(request.value(), None);
    }

    #[test]
    fn a_refusal_of_another_service_is_not_ours() {
        let mut request = UdsRequest::new(&READ_SERIAL, 0x00);
        assert_eq!(
            request.on_frame(&[0x03, 0x7F, 0x10, 0x31, 0x00, 0x00, 0x00, 0x00]),
            UdsStep::Failed(UdsError::Malformed)
        );
    }

    #[test]
    fn response_pending_waits_longer() {
        let mut request = UdsRequest::new(&READ_SERIAL, 0x00);
        assert_eq!(request.timeout_ms(), RESPONSE_TIMEOUT_MS);
        assert_eq!(
            request.on_frame(&[0x03, 0x7F, 0x22, 0x78, 0x00, 0x00, 0x00, 0x00]),
            UdsStep::Wait
        );
        assert_eq!(request.timeout_ms(), PENDING_TIMEOUT_MS);
        assert_eq!(
            request.on_frame(&SERIAL_FRAMES[0]),
            UdsStep::SendFlowControl
        );
        assert_eq!(request.timeout_ms(), RESPONSE_TIMEOUT_MS);
    }

    #[test]
    fn a_lost_consecutive_frame_fails_the_request() {
        let mut request = UdsRequest::new(&READ_SERIAL, 0x00);
        request.on_frame(&SERIAL_FRAMES[0]);
        request.on_frame(&SERIAL_FRAMES[1]);
        assert_eq!(
            request.on_frame(&SERIAL_FRAMES[3]),
            UdsStep::Failed(UdsError::Sequence)
        );
        assert_eq!(
            request.on_frame(&SERIAL_FRAMES[2]),
            UdsStep::Failed(UdsError::Sequence),
            "stays failed"
        );
    }

    #[test]
    fn an_answer_to_another_identifier_fails_the_request() {
        let mut request = UdsRequest::new(&[0x22, 0xF1, 0x90], 0x00);
        for frame in &SERIAL_FRAMES[..5] {
            request.on_frame(frame);
        }
        assert_eq!(
            request.on_frame(&SERIAL_FRAMES[5]),
            UdsStep::Failed(UdsError::Malformed)
        );
    }

    #[test]
    fn a_too_long_answer_fails_before_the_flow_control() {
        let mut request = UdsRequest::new(&READ_SERIAL, 0x00);
        assert_eq!(
            request.on_frame(&[0x11, 0x00, 0x62, 0xF1, 0x8C, 0x00, 0x00, 0x00]),
            UdsStep::Failed(UdsError::TooLong)
        );
    }

    #[test]
    fn malformed_frames_fail_the_request() {
        for frame in [
            &[][..],
            &[0x10, 0x05, 0x62, 0xF1, 0x8C, 0, 0, 0][..],
            &[0x05, 0x62],
        ] {
            let mut request = UdsRequest::new(&READ_SERIAL, 0x00);
            assert_eq!(
                request.on_frame(frame),
                UdsStep::Failed(UdsError::Malformed)
            );
        }
    }

    #[test]
    #[should_panic(expected = "a request must fit in a single frame")]
    fn a_request_must_fit_in_one_frame() {
        UdsRequest::new(&[0x22; 8], 0x00);
    }
}
