//! Identifies the exact car: the second stage of the car identity (see `docs/can.md`,
//! "Car identity"), once `platform.rs` has confirmed the platform.
//!
//! North American cars don't broadcast the VIN on M-CAN, so the fingerprint is the head
//! unit's serial number, read with UDS `22 F18C` on 0x780 (answer on 0x788) through the
//! identity probe, on every car. Checked on an EV6: the head unit answers in the default
//! session whenever the bus is awake, with the car in Ready, ACC or off.
//!
//! Driven by received frames, from `can_rx_task`: it only moves on while the bus is
//! busy, so it never sends anything on a sleeping bus (a request there woke an EV6).

use alloc::vec::Vec;

use core_interface::{
    CanFrame, CanTxFrame, IdentityProbe, PROBE_SETTLE_MS, ProbeRefused, ProbeWindow, UdsRequest,
    UdsStep, iso_tp_flow_control, standard_id, uds_single_frame,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};

const REQUEST_ID: u16 = 0x780;
pub(crate) const RESPONSE_ID: u32 = 0x788;
/// ReadDataByIdentifier F18C: the ECU serial number.
const READ_SERIAL: [u8; 3] = [0x22, 0xF1, 0x8C];
const PADDING: u8 = 0x00;

/// The only frames E-GMP sends before the car is identified.
static HEAD_UNIT_SERIAL_PROBE: IdentityProbe = IdentityProbe {
    bus_id: 0,
    frames: &[
        (
            standard_id(REQUEST_ID),
            uds_single_frame(&READ_SERIAL, PADDING),
        ),
        (standard_id(REQUEST_ID), iso_tp_flow_control(PADDING)),
    ],
};

/// Keeps the two kinds of identifier from ever matching each other.
const FINGERPRINT_PREFIX: &[u8] = b"hu-serial:";
/// Wait after a failed attempt, then the next ones; the last repeats until
/// `core-interface` refuses more attempts.
const RETRY_DELAYS_MS: [u64; 4] = [2_000, 5_000, 15_000, 30_000];
/// Wait before trying again when the probe can't be opened for now (bus asleep, logging).
const REFUSED_RETRY_MS: u64 = 1_000;

enum Stage {
    /// Nothing open; the next attempt is not before `next_try`.
    Idle { next_try: Instant },
    /// A window is open; the request goes out once the controller has left listen-only.
    Settling { window: ProbeWindow, since: Instant },
    /// The request is out; waiting for the head unit.
    Waiting {
        _window: ProbeWindow,
        request: UdsRequest,
        deadline: Instant,
    },
    /// The car is identified. Read again if the identity is revoked.
    Identified,
    /// No attempts left: nothing more until a reboot.
    GaveUp,
}

/// What the caller has to do after a frame.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Action {
    None,
    /// Send this frame on 0x780.
    Send([u8; 8]),
    /// The head unit answered: confirm this fingerprint.
    Identified(Vec<u8>),
}

pub(crate) struct CarIdentifier {
    stage: Stage,
    failures: usize,
}

impl CarIdentifier {
    pub(crate) const fn new() -> Self {
        Self {
            stage: Stage::Idle {
                next_try: Instant::from_ticks(0),
            },
            failures: 0,
        }
    }

    /// Moves on after a frame received at `now` (any frame: they show the bus is awake).
    pub(crate) fn on_frame(&mut self, id: u32, data: &[u8], now: Instant) -> Action {
        if core_interface::is_car_fingerprint_confirmed() {
            self.stage = Stage::Identified;
            return Action::None;
        }
        if !core_interface::is_car_identity_confirmed() {
            // Wait for the platform; a probe in progress is dropped.
            if !matches!(self.stage, Stage::GaveUp) {
                self.stage = Stage::Idle { next_try: now };
            }
            return Action::None;
        }

        match &mut self.stage {
            Stage::GaveUp => Action::None,
            // Identified, then the identity was revoked: read the serial again.
            Stage::Identified => {
                self.stage = Stage::Idle { next_try: now };
                Action::None
            }
            Stage::Idle { next_try } => {
                if now < *next_try {
                    return Action::None;
                }
                match core_interface::open_identity_probe(&HEAD_UNIT_SERIAL_PROBE) {
                    Ok(window) => self.stage = Stage::Settling { window, since: now },
                    Err(ProbeRefused::NoAttemptsLeft | ProbeRefused::Invalid) => {
                        self.stage = Stage::GaveUp;
                    }
                    Err(_) => *next_try = now + Duration::from_millis(REFUSED_RETRY_MS),
                }
                Action::None
            }
            Stage::Settling { since, .. } => {
                if now.saturating_duration_since(*since) < Duration::from_millis(PROBE_SETTLE_MS) {
                    return Action::None;
                }
                let Stage::Settling { window, .. } =
                    core::mem::replace(&mut self.stage, Stage::GaveUp)
                else {
                    unreachable!()
                };
                let request = UdsRequest::new(&READ_SERIAL, PADDING);
                let frame = request.request_frame();
                self.stage = Stage::Waiting {
                    deadline: now + Duration::from_millis(request.timeout_ms()),
                    _window: window,
                    request,
                };
                Action::Send(frame)
            }
            Stage::Waiting {
                request, deadline, ..
            } => {
                if id != RESPONSE_ID {
                    if now > *deadline {
                        self.fail(now);
                    }
                    return Action::None;
                }
                let step = request.on_frame(data);
                *deadline = now + Duration::from_millis(request.timeout_ms());
                match step {
                    UdsStep::Wait => Action::None,
                    UdsStep::SendFlowControl => Action::Send(request.flow_control_frame()),
                    UdsStep::Done => {
                        let fingerprint = request.value().and_then(fingerprint);
                        match fingerprint {
                            Some(fingerprint) => {
                                self.stage = Stage::Identified;
                                Action::Identified(fingerprint)
                            }
                            None => {
                                self.fail(now);
                                Action::None
                            }
                        }
                    }
                    UdsStep::Failed(_) => {
                        self.fail(now);
                        Action::None
                    }
                }
            }
        }
    }

    /// Closes the window and schedules the next attempt.
    fn fail(&mut self, now: Instant) {
        let delay = RETRY_DELAYS_MS[self.failures.min(RETRY_DELAYS_MS.len() - 1)];
        self.failures += 1;
        self.stage = Stage::Idle {
            next_try: now + Duration::from_millis(delay),
        };
    }
}

/// The fingerprint for a serial as the head unit sends it: the prefix, then the serial
/// without its trailing padding. `None` if nothing is left.
fn fingerprint(serial: &[u8]) -> Option<Vec<u8>> {
    let end = serial
        .iter()
        .rposition(|b| !matches!(b, 0x00 | b' ' | 0xFF))?;
    let mut fingerprint = Vec::with_capacity(FINGERPRINT_PREFIX.len() + end + 1);
    fingerprint.extend_from_slice(FINGERPRINT_PREFIX);
    fingerprint.extend_from_slice(&serial[..=end]);
    Some(fingerprint)
}

static IDENTIFIER: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<CarIdentifier>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(CarIdentifier::new()));

/// Runs the identification on a received frame: sends what the probe needs and, once the
/// head unit answered, confirms the car and asks to leave read-only mode (it still waits
/// for the owner's approval and the dev lock).
pub(crate) fn track_identity(id: u32, data: &[u8]) {
    match IDENTIFIER.lock(|i| i.borrow_mut().on_frame(id, data, Instant::now())) {
        Action::None => {}
        Action::Send(data) => {
            let frame = CanFrame {
                bus_id: HEAD_UNIT_SERIAL_PROBE.bus_id,
                id: standard_id(REQUEST_ID),
                data,
                dlc: 8,
            };
            // Never block the RX path: a full channel loses the frame, and the attempt
            // then times out and is retried. A platform frame: it only reads the serial.
            let _ = core_interface::can_tx_channel(frame.bus_id)
                .map(|channel| channel.try_send(CanTxFrame::platform(frame)));
        }
        Action::Identified(fingerprint) => {
            if core_interface::confirm_car_fingerprint(&fingerprint) {
                core_interface::set_can_read_only(false);
            }
        }
    }
}

#[cfg(test)]
pub(crate) fn reset_identity_for_tests() {
    IDENTIFIER.lock(|i| *i.borrow_mut() = CarIdentifier::new());
}

#[cfg(test)]
mod tests {
    use super::*;
    use core_interface::MAX_PROBE_ATTEMPTS;

    /// The head unit (0x788) answering 22 F18C on an EV6 (2026-10-31).
    const SERIAL_FRAMES: [[u8; 8]; 6] = [
        [0x10, 0x29, 0x62, 0xF1, 0x8C, 0x32, 0x34, 0x30],
        [0x21, 0x34, 0x30, 0x34, 0x41, 0x30, 0x34, 0x20],
        [0x22, 0x4A, 0x58, 0x49, 0x4F, 0x34, 0x30, 0x30],
        [0x23, 0x30, 0x32, 0x33, 0x00, 0x00, 0x00, 0x00],
        [0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        [0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    ];
    const REQUEST: [u8; 8] = [0x03, 0x22, 0xF1, 0x8C, 0x00, 0x00, 0x00, 0x00];
    const FLOW_CONTROL: [u8; 8] = [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    /// Any other frame from the car (CLIMATE_TEMPERATURE).
    const CAR_ID: u32 = 0x380;

    fn on_780(data: [u8; 8]) -> CanFrame {
        CanFrame {
            bus_id: 0,
            id: standard_id(0x780),
            data,
            dlc: 8,
        }
    }

    /// The platform confirmed on an awake bus, nothing else.
    fn platform_confirmed() {
        boot();
        core_interface::confirm_car_identity();
        awake();
    }

    fn boot() {
        core_interface::reset_identity_probe_for_tests();
        core_interface::reset_can_access_for_tests();
        core_interface::revoke_car_identity();
        reset_identity_for_tests();
    }

    /// A frame the board saw, so the bus counts as awake for `core-interface`.
    fn awake() {
        core_interface::log_can_frame(&CanFrame {
            bus_id: 0,
            id: standard_id(0x380),
            data: [0; 8],
            dlc: 8,
        });
    }

    fn ms(t: Instant, ms: u64) -> Instant {
        t + Duration::from_millis(ms)
    }

    /// Opens a window and sends the request, as on the bus: returns when it is out.
    fn send_request(identifier: &mut CarIdentifier, t: Instant) {
        assert_eq!(identifier.on_frame(CAR_ID, &[0; 8], t), Action::None);
        assert_eq!(
            identifier.on_frame(CAR_ID, &[0; 8], ms(t, 10)),
            Action::None,
            "settling"
        );
        assert_eq!(
            identifier.on_frame(CAR_ID, &[0; 8], ms(t, PROBE_SETTLE_MS)),
            Action::Send(REQUEST)
        );
    }

    #[test]
    fn reads_the_head_unit_serial() {
        platform_confirmed();
        let t = Instant::now();
        let mut identifier = CarIdentifier::new();
        send_request(&mut identifier, t);
        assert!(
            core_interface::allow_can_tx(0, &CanTxFrame::platform(on_780(REQUEST))),
            "the board sends it"
        );

        let t = ms(t, PROBE_SETTLE_MS + 5);
        assert_eq!(
            identifier.on_frame(RESPONSE_ID, &SERIAL_FRAMES[0], t),
            Action::Send(FLOW_CONTROL)
        );
        assert!(core_interface::allow_can_tx(
            0,
            &CanTxFrame::platform(on_780(FLOW_CONTROL))
        ));
        for frame in &SERIAL_FRAMES[1..5] {
            assert_eq!(identifier.on_frame(RESPONSE_ID, frame, t), Action::None);
        }
        assert_eq!(
            identifier.on_frame(RESPONSE_ID, &SERIAL_FRAMES[5], t),
            Action::Identified(b"hu-serial:240404A04 JXIO400023".to_vec())
        );
        assert!(
            !core_interface::allow_can_tx(0, &CanTxFrame::platform(on_780(REQUEST))),
            "the window is closed"
        );
        boot();
    }

    #[test]
    fn waits_for_the_platform() {
        boot();
        awake();
        let t = Instant::now();
        let mut identifier = CarIdentifier::new();
        for i in 0..10 {
            assert_eq!(
                identifier.on_frame(CAR_ID, &[0; 8], ms(t, i * 100)),
                Action::None
            );
        }
        core_interface::confirm_car_identity();
        send_request(&mut identifier, ms(t, 1_000));
        boot();
    }

    #[test]
    fn waits_for_an_awake_bus() {
        boot();
        core_interface::confirm_car_identity();
        let t = Instant::now();
        let mut identifier = CarIdentifier::new();
        for i in 0..3 {
            assert_eq!(
                identifier.on_frame(CAR_ID, &[0; 8], ms(t, i * 100)),
                Action::None
            );
        }
        assert!(
            core_interface::is_can_bus_listen_only(0),
            "no window without traffic seen by the board"
        );
        awake();
        // The next try is a second after the refusal.
        assert_eq!(
            identifier.on_frame(CAR_ID, &[0; 8], ms(t, 900)),
            Action::None
        );
        send_request(&mut identifier, ms(t, 1_000));
        boot();
    }

    #[test]
    fn retries_after_no_answer_then_gives_up() {
        platform_confirmed();
        let mut t = Instant::now();
        let mut identifier = CarIdentifier::new();
        for attempt in 0..MAX_PROBE_ATTEMPTS as usize {
            send_request(&mut identifier, t);
            // No answer within the timeout: the window closes.
            t = ms(t, PROBE_SETTLE_MS + 1_001);
            assert_eq!(identifier.on_frame(CAR_ID, &[0; 8], t), Action::None);
            assert!(core_interface::is_can_bus_listen_only(0));
            let delay = RETRY_DELAYS_MS[attempt.min(RETRY_DELAYS_MS.len() - 1)];
            assert_eq!(
                identifier.on_frame(CAR_ID, &[0; 8], ms(t, delay - 1)),
                Action::None,
                "not before the back-off"
            );
            t = ms(t, delay);
        }
        // Every attempt used: it stops for good.
        for i in 0..5 {
            assert_eq!(
                identifier.on_frame(CAR_ID, &[0; 8], ms(t, i * 60_000)),
                Action::None
            );
        }
        assert!(core_interface::is_can_bus_listen_only(0));
        boot();
    }

    #[test]
    fn a_refusal_is_retried() {
        platform_confirmed();
        let t = Instant::now();
        let mut identifier = CarIdentifier::new();
        send_request(&mut identifier, t);
        let t = ms(t, PROBE_SETTLE_MS + 5);
        assert_eq!(
            identifier.on_frame(RESPONSE_ID, &[0x03, 0x7F, 0x22, 0x22, 0, 0, 0, 0], t),
            Action::None
        );
        assert!(core_interface::is_can_bus_listen_only(0), "window closed");
        send_request(&mut identifier, ms(t, RETRY_DELAYS_MS[0]));
        boot();
    }

    #[test]
    fn a_revoked_identity_is_read_again() {
        platform_confirmed();
        let t = Instant::now();
        let mut identifier = CarIdentifier::new();
        assert!(core_interface::confirm_car_fingerprint(b"hu-serial:TEST"));
        assert_eq!(identifier.on_frame(CAR_ID, &[0; 8], t), Action::None);
        assert!(core_interface::is_can_bus_listen_only(0), "nothing to read");

        // The platform check stopped matching, then matched again.
        core_interface::revoke_car_identity();
        assert_eq!(identifier.on_frame(CAR_ID, &[0; 8], t), Action::None);
        core_interface::confirm_car_identity();
        send_request(&mut identifier, ms(t, 100));
        boot();
    }

    #[test]
    fn an_empty_serial_is_not_a_fingerprint() {
        assert_eq!(fingerprint(&[0x00, 0x20, 0xFF]), None);
        assert_eq!(
            fingerprint(b"ABC  \x00\x00"),
            Some(b"hu-serial:ABC".to_vec())
        );
    }

    #[test]
    fn tracking_identifies_the_car_in_core_interface() {
        platform_confirmed();
        let channel = core_interface::can_tx_channel(0).unwrap();
        while channel.try_receive().is_ok() {}
        let start = Instant::now();
        // Real time: wait out the settle delay between the first two frames.
        track_identity(CAR_ID, &[0; 8]);
        while start.elapsed() < Duration::from_millis(PROBE_SETTLE_MS + 5) {}
        track_identity(CAR_ID, &[0; 8]);
        assert_eq!(channel.try_receive().unwrap().frame().data, REQUEST);
        track_identity(RESPONSE_ID, &SERIAL_FRAMES[0]);
        assert_eq!(channel.try_receive().unwrap().frame().data, FLOW_CONTROL);
        for frame in &SERIAL_FRAMES[1..] {
            track_identity(RESPONSE_ID, frame);
        }
        assert!(core_interface::is_car_fingerprint_confirmed());
        assert!(
            core_interface::is_can_read_only(),
            "still waiting for the owner's approval"
        );
        core_interface::approve_can_read_write().unwrap();
        core_interface::set_can_dev_write_unlocked(true).ok();
        assert!(
            !core_interface::is_can_read_only(),
            "read-write once approved"
        );
        boot();
    }
}
