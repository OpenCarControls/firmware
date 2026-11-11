//! Manual battery preconditioning (`BatteryPreconditioningCommand`, see `docs/can.md`,
//! "E-GMP battery preconditioning").
//!
//! The protocol is wicant-i-precondition's (github.com/L1Z3/wicant-i-precondition), in its
//! single-bus mode and with its "Once" behaviour: start and stop bursts on HU_EV_E (0x0C7),
//! the BMS answer on BMS_PRECOND_STATUS (0x2AD), and, while a session is on, a copy of each
//! head-unit NAV_PRECOND_STATUS (0x4ED) right after it, with the route bytes that keep
//! preconditioning on. The device taps M-CAN, so it can't remove the head unit's frames, only
//! follow them with its own.
//!
//! A start the car doesn't take is retried. A session ends when it's turned off, when the car
//! stops it (the battery is warm enough), when the car leaves Ready, or when the retries run
//! out. The cluster shows its progress in the popup the validation code uses
//! (`cluster_code.rs`).
//!
//! Driven by received frames, from `can_rx_task`, so nothing is sent on a sleeping bus. The
//! frames that act on the car are command frames, sent with the authorization of the command
//! that started or stopped the session, so they stop going out if that phone loses its
//! validation.

use alloc::format;
use alloc::string::String;
use core::cell::RefCell;

use core_interface::{CanFrame, CanTxFrame, CommandAuthorization, standard_id};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};

use crate::proto::advanced_state::PreconditioningState as Status;

const BUS_ID: u8 = 0;
/// HU_EV_E: the head unit's requests to the BMU.
pub(crate) const HU_EV_E_ID: u16 = 0x0C7;
/// NAV_PRECOND_STATUS: route data the head unit sends for the BMU.
pub(crate) const NAV_PRECOND_STATUS_ID: u32 = 0x4ED;

/// HU_EV_E bytes 3-4 of a start burst: three frames of each pattern, in order.
const START_PATTERNS: [[u8; 2]; 2] = [[0x40, 0x03], [0xE0, 0x07]];
/// The same for a stop burst.
const STOP_PATTERNS: [[u8; 2]; 2] = [[0x00, 0x00], [0xE0, 0x07]];
const FRAMES_PER_PATTERN: u8 = 3;
const BURST_FRAMES: u8 = FRAMES_PER_PATTERN * 2;
/// Between two frames of a burst.
const BURST_PERIOD_MS: u64 = 40;
/// How long the car has to report preparing after a start burst, or off after a stop burst,
/// before the burst is sent again.
const RETRY_MS: u64 = 10_000;
/// How long the car has from a start burst to heating. Preparing lasts about a minute but
/// varies; wicant-i-precondition uses 70 s on Ioniq 5 and EV6.
const STARTED_TIMEOUT_MS: u64 = 70_000;
/// Bursts sent again after the first, for a start or a stop.
const MAX_RETRIES: u8 = 4;
/// NAV_PRECOND_STATUS bytes 5-7 that keep preconditioning on.
const KEEP_ON: [u8; 3] = [0x10, 0xA0, 0x00];
/// How long the cluster says a start failed: one popup.
const FAILED_NOTICE_MS: u64 = 3_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    /// Sending the start burst; `sent` frames are out.
    StartBurst {
        sent: u8,
    },
    /// Waiting for the car to report preparing, then heating.
    Starting,
    /// The battery heater is on.
    On,
    /// Sending the stop burst.
    StopBurst {
        sent: u8,
    },
    /// Waiting for the car to report off after a stop that was asked for.
    Stopping,
}

/// Why a request was refused.
pub(crate) const NOT_ON: &str = "command rejected: manual preconditioning is not on";

pub(crate) struct Session {
    phase: Phase,
    /// Counts sessions, so the cluster starts a new popup for each.
    id: u32,
    /// The car's last report.
    status: Status,
    /// When the current start or stop burst began.
    attempt_at: Instant,
    last_frame_at: Option<Instant>,
    /// Bursts sent again for the current start or stop.
    retries: u8,
    /// Whether the stop being sent waits for the car to confirm it. A stop that cleans up after
    /// the car ended preconditioning, or after a failed start, doesn't.
    confirm_stop: bool,
    authorization: Option<CommandAuthorization>,
    /// Until when the cluster says the last start failed. The car shows preconditioning
    /// starting and stopping itself (a message and the battery icon), but nothing for a start
    /// it never took.
    failed_until: Option<Instant>,
}

impl Session {
    pub(crate) const fn new() -> Self {
        Self {
            phase: Phase::Idle,
            id: 0,
            status: Status::Unspecified,
            attempt_at: Instant::from_ticks(0),
            last_frame_at: None,
            retries: 0,
            confirm_stop: false,
            authorization: None,
            failed_until: None,
        }
    }

    /// Whether a session is running, from the command that started it until it has ended,
    /// stop included. `AdvancedState.is_manual_preconditioning_requested`.
    pub(crate) fn is_requested(&self) -> bool {
        self.phase != Phase::Idle
    }

    /// Whether the session keeps preconditioning on: started and not being stopped.
    fn keeps_on(&self) -> bool {
        matches!(
            self.phase,
            Phase::StartBurst { .. } | Phase::Starting | Phase::On
        )
    }

    /// Starts (`enable`) or stops the session. Starting a running session, or stopping one
    /// being stopped, changes nothing but the authorization its frames go out with.
    pub(crate) fn request(
        &mut self,
        enable: bool,
        authorization: CommandAuthorization,
        now: Instant,
    ) -> Result<(), &'static str> {
        if enable {
            if self.keeps_on() {
                log::info!(
                    "Preconditioning: start asked, session {} already on",
                    self.id
                );
            } else {
                self.id = self.id.wrapping_add(1);
                self.failed_until = None;
                log::info!("Preconditioning: session {} starting", self.id);
                self.begin_start(now, 0);
            }
        } else {
            match self.phase {
                Phase::Idle => return Err(NOT_ON),
                Phase::StopBurst { .. } | Phase::Stopping => {
                    log::info!(
                        "Preconditioning: stop asked, session {} already stopping",
                        self.id
                    );
                }
                _ => {
                    log::info!("Preconditioning: session {} stopping (turned off)", self.id);
                    self.begin_stop(now, 0, true);
                }
            }
        }
        self.authorization = Some(authorization);
        Ok(())
    }

    fn begin_start(&mut self, now: Instant, retries: u8) {
        if retries > 0 {
            log::warn!(
                "Preconditioning: session {}, start sent again ({retries}/{MAX_RETRIES}), car: {:?}",
                self.id,
                self.status
            );
        }
        self.phase = Phase::StartBurst { sent: 0 };
        self.attempt_at = now;
        self.last_frame_at = None;
        self.retries = retries;
    }

    fn begin_stop(&mut self, now: Instant, retries: u8, confirm: bool) {
        if retries > 0 {
            log::warn!(
                "Preconditioning: session {}, stop sent again ({retries}/{MAX_RETRIES})",
                self.id
            );
        }
        self.phase = Phase::StopBurst { sent: 0 };
        self.attempt_at = now;
        self.last_frame_at = None;
        self.retries = retries;
        self.confirm_stop = confirm;
    }

    fn end(&mut self, why: &str) {
        log::info!("Preconditioning: session {} ended ({why})", self.id);
        self.phase = Phase::Idle;
        self.authorization = None;
    }

    /// The car's report, from BMS_PRECOND_STATUS.
    pub(crate) fn on_status(&mut self, status: Status, now: Instant) {
        if status != self.status && self.is_requested() {
            log::info!(
                "Preconditioning: session {}, car {:?} -> {:?}",
                self.id,
                self.status,
                status
            );
        }
        self.status = status;
        match (self.phase, status) {
            (Phase::Starting, Status::Heating) => {
                log::info!("Preconditioning: session {} on", self.id);
                self.phase = Phase::On;
            }
            // The car ended it, most likely because the battery is warm enough. Stop too, so
            // the car doesn't start again when it cools down.
            (Phase::On, Status::Off) => {
                log::info!(
                    "Preconditioning: session {} stopping (the car ended it)",
                    self.id
                );
                self.begin_stop(now, 0, false);
            }
            (Phase::Stopping, Status::Off) => self.end("the car confirmed the stop"),
            _ => {}
        }
    }

    /// Whether the car is in Ready. Leaving it ends preconditioning, so the session ends with
    /// nothing to send.
    pub(crate) fn set_ready(&mut self, ready: bool) {
        if !ready && self.is_requested() {
            self.end("the car left Ready");
            self.failed_until = None;
        }
    }

    /// Moves the session on at `now`, and returns the HU_EV_E data to send, if any.
    pub(crate) fn poll(&mut self, now: Instant) -> Option<[u8; 8]> {
        match self.phase {
            Phase::StartBurst { sent } => self.burst_frame(sent, &START_PATTERNS, now),
            Phase::StopBurst { sent } => self.burst_frame(sent, &STOP_PATTERNS, now),
            Phase::Starting => {
                let limit = if self.status == Status::Preparing {
                    STARTED_TIMEOUT_MS
                } else {
                    RETRY_MS
                };
                if now > self.attempt_at + Duration::from_millis(limit) {
                    if self.retries < MAX_RETRIES {
                        self.begin_start(now, self.retries + 1);
                    } else {
                        // Give up, and make sure the car doesn't start late.
                        log::warn!(
                            "Preconditioning: session {} failed to start, car: {:?}",
                            self.id,
                            self.status
                        );
                        self.failed_until = Some(now + Duration::from_millis(FAILED_NOTICE_MS));
                        self.begin_stop(now, 0, false);
                    }
                }
                None
            }
            Phase::Stopping => {
                if now > self.attempt_at + Duration::from_millis(RETRY_MS) {
                    if self.retries < MAX_RETRIES {
                        self.begin_stop(now, self.retries + 1, true);
                    } else {
                        self.end("the car never confirmed the stop");
                    }
                }
                None
            }
            Phase::Idle | Phase::On => None,
        }
    }

    fn burst_frame(&mut self, sent: u8, patterns: &[[u8; 2]; 2], now: Instant) -> Option<[u8; 8]> {
        if self
            .last_frame_at
            .is_some_and(|at| now < at + Duration::from_millis(BURST_PERIOD_MS))
        {
            return None;
        }
        let [b3, b4] = patterns[(sent / FRAMES_PER_PATTERN) as usize];
        self.last_frame_at = Some(now);
        let sent = sent + 1;
        let starting = matches!(self.phase, Phase::StartBurst { .. });
        if sent == BURST_FRAMES {
            log::info!(
                "Preconditioning: session {}, {} burst sent",
                self.id,
                if starting { "start" } else { "stop" }
            );
        }
        self.phase = match (starting, sent < BURST_FRAMES) {
            (true, true) => Phase::StartBurst { sent },
            (false, true) => Phase::StopBurst { sent },
            (true, false) if self.status == Status::Heating => {
                log::info!("Preconditioning: session {} on", self.id);
                Phase::On
            }
            (true, false) => Phase::Starting,
            (false, false) if self.confirm_stop && self.status != Status::Off => Phase::Stopping,
            (false, false) => {
                self.end(if self.confirm_stop {
                    "the car was already off"
                } else {
                    "cleanup stop sent"
                });
                Phase::Idle
            }
        };
        Some([0, 0, 0, b3, b4, 0, 0, 0])
    }

    /// The copy to send after a head-unit NAV_PRECOND_STATUS, while the session keeps
    /// preconditioning on. A frame that already carries the bytes (ours, or the head unit
    /// routing to a charger) needs none.
    pub(crate) fn keep_on_copy(&self, data: &[u8]) -> Option<[u8; 8]> {
        if !self.keeps_on() {
            return None;
        }
        let mut frame: [u8; 8] = data.try_into().ok()?;
        if frame[5..8] == KEEP_ON {
            return None;
        }
        frame[5..8].copy_from_slice(&KEEP_ON);
        Some(frame)
    }

    /// What the cluster shows now, with the session it belongs to: the countdown while
    /// starting, or that the start failed. The car shows the rest itself.
    pub(crate) fn cluster_text(&self, now: Instant) -> Option<(u32, String)> {
        let text = match self.phase {
            Phase::StartBurst { .. } | Phase::Starting => {
                let elapsed = now.saturating_duration_since(self.attempt_at).as_secs();
                let left = (STARTED_TIMEOUT_MS / 1000).saturating_sub(elapsed);
                format!("Precond. starting... {left}")
            }
            _ if self.failed_until.is_some_and(|until| now < until) => {
                String::from("Precond. failed")
            }
            _ => return None,
        };
        Some((self.id, text))
    }
}

static SESSION: Mutex<CriticalSectionRawMutex, RefCell<Session>> =
    Mutex::new(RefCell::new(Session::new()));

/// Starts or stops the session for a command. Its requirements were checked before.
pub(crate) fn request(
    enable: bool,
    authorization: CommandAuthorization,
) -> Result<(), &'static str> {
    SESSION.lock(|s| {
        s.borrow_mut()
            .request(enable, authorization, Instant::now())
    })
}

pub(crate) fn is_requested() -> bool {
    SESSION.lock(|s| s.borrow().is_requested())
}

pub(crate) fn on_status(status: Status) {
    SESSION.lock(|s| s.borrow_mut().on_status(status, Instant::now()));
}

pub(crate) fn set_ready(ready: bool) {
    SESSION.lock(|s| s.borrow_mut().set_ready(ready));
}

/// What the cluster shows for preconditioning now.
pub(crate) fn cluster_text() -> Option<(u32, String)> {
    SESSION.lock(|s| s.borrow().cluster_text(Instant::now()))
}

/// Runs the session on a received frame: the next burst frame when one is due, and the copy
/// of NAV_PRECOND_STATUS while it keeps preconditioning on.
pub(crate) fn track_preconditioning(id: u32, data: &[u8]) {
    let frames = SESSION.lock(|s| {
        let mut s = s.borrow_mut();
        let keep_on = (id == NAV_PRECOND_STATUS_ID)
            .then(|| s.keep_on_copy(data))
            .flatten()
            .map(|data| (NAV_PRECOND_STATUS_ID as u16, data));
        let burst = s.poll(Instant::now()).map(|data| (HU_EV_E_ID, data));
        let authorization = s.authorization.as_ref()?;
        Some(
            [keep_on, burst]
                .into_iter()
                .flatten()
                .map(|(id, data)| {
                    CanTxFrame::command(
                        authorization,
                        CanFrame {
                            bus_id: BUS_ID,
                            id: standard_id(id),
                            data,
                            dlc: 8,
                        },
                    )
                })
                .collect::<alloc::vec::Vec<_>>(),
        )
    });
    // Never block the RX path: a full channel loses the frame. The burst is sent again if the
    // car doesn't answer, and the next NAV_PRECOND_STATUS gets its copy.
    if let Some(channel) = core_interface::can_tx_channel(BUS_ID) {
        for frame in frames.into_iter().flatten() {
            let _ = channel.try_send(frame);
        }
    }
}

#[cfg(test)]
pub(crate) fn reset_preconditioning_for_tests() {
    SESSION.lock(|s| *s.borrow_mut() = Session::new());
}

#[cfg(test)]
mod tests {
    use super::*;

    const PHONE: &[u8] = b"ble:\x01\x02\x03\x04\x05\x06";
    /// NAV_PRECOND_STATUS with no route, on an EV6.
    const NO_ROUTE: [u8; 8] = [0, 0, 0, 0, 0, 0xFF, 0xFF, 0x1F];

    fn auth() -> CommandAuthorization {
        CommandAuthorization::for_tests(PHONE)
    }

    fn ms(t: Instant, ms: u64) -> Instant {
        t + Duration::from_millis(ms)
    }

    /// Polls every 10 ms from `from` for `for_ms`, and returns the frames sent with their time.
    fn run(s: &mut Session, from: Instant, for_ms: u64) -> alloc::vec::Vec<(u64, [u8; 8])> {
        (0..=for_ms / 10)
            .filter_map(|i| {
                let now = ms(from, i * 10);
                s.poll(now).map(|f| ((now - from).as_millis(), f))
            })
            .collect()
    }

    fn start_burst() -> alloc::vec::Vec<[u8; 8]> {
        let p = |b3, b4| [0, 0, 0, b3, b4, 0, 0, 0];
        alloc::vec![
            p(0x40, 0x03),
            p(0x40, 0x03),
            p(0x40, 0x03),
            p(0xE0, 0x07),
            p(0xE0, 0x07),
            p(0xE0, 0x07)
        ]
    }

    fn stop_burst() -> alloc::vec::Vec<[u8; 8]> {
        let p = |b3, b4| [0, 0, 0, b3, b4, 0, 0, 0];
        alloc::vec![
            p(0, 0),
            p(0, 0),
            p(0, 0),
            p(0xE0, 0x07),
            p(0xE0, 0x07),
            p(0xE0, 0x07)
        ]
    }

    fn frames(sent: &[(u64, [u8; 8])]) -> alloc::vec::Vec<[u8; 8]> {
        sent.iter().map(|&(_, f)| f).collect()
    }

    #[test]
    fn a_start_is_six_frames_40_ms_apart() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        let sent = run(&mut s, t, 1_000);
        assert_eq!(frames(&sent), start_burst());
        let times: alloc::vec::Vec<u64> = sent.iter().map(|&(at, _)| at).collect();
        assert_eq!(times, [0, 40, 80, 120, 160, 200]);
        assert_eq!(s.phase, Phase::Starting);
        assert!(s.is_requested());
    }

    #[test]
    fn the_session_follows_the_car_to_on() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        run(&mut s, t, 300);
        s.on_status(Status::Off, ms(t, 500));
        s.on_status(Status::Preparing, ms(t, 2_000));
        assert_eq!(s.phase, Phase::Starting);
        assert!(
            run(&mut s, ms(t, 2_000), 60_000).is_empty(),
            "no retry while preparing"
        );
        s.on_status(Status::Heating, ms(t, 62_000));
        assert_eq!(s.phase, Phase::On);
        assert_eq!(
            s.cluster_text(ms(t, 62_000)),
            None,
            "the car shows that it's on itself"
        );
    }

    #[test]
    fn a_start_the_car_ignores_is_retried_then_given_up() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        let sent = run(&mut s, t, 50_000);
        let bursts = frames(&sent).chunks(6).count();
        assert_eq!(
            bursts,
            1 + MAX_RETRIES as usize,
            "the first burst and the retries"
        );
        let starts: alloc::vec::Vec<u64> = sent.iter().step_by(6).map(|&(at, _)| at).collect();
        assert!(
            starts
                .windows(2)
                .all(|w| (10_000..10_100).contains(&(w[1] - w[0])))
        );

        // After the last retry, a cleanup stop and the end of the session.
        let last = ms(t, *starts.last().unwrap());
        let cleanup = run(&mut s, ms(last, 10_001), 1_000);
        assert_eq!(frames(&cleanup), stop_burst());
        assert!(!s.is_requested());
        assert_eq!(
            s.cluster_text(ms(last, 10_500)).map(|(_, text)| text),
            Some(String::from("Precond. failed"))
        );
        assert_eq!(s.cluster_text(ms(last, 14_000)), None, "one popup");
    }

    #[test]
    fn the_countdown_counts_to_the_started_timeout() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        let text = |at| s.cluster_text(ms(t, at)).unwrap().1;
        assert_eq!(text(0), "Precond. starting... 70");
        assert_eq!(text(25_400), "Precond. starting... 45");
    }

    #[test]
    fn turning_it_off_waits_for_the_car_and_retries() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        run(&mut s, t, 300);
        s.on_status(Status::Heating, ms(t, 1_000));

        s.request(false, auth(), ms(t, 5_000)).unwrap();
        assert!(
            s.keep_on_copy(&NO_ROUTE).is_none(),
            "the override stops at once"
        );
        assert_eq!(frames(&run(&mut s, ms(t, 5_000), 300)), stop_burst());
        assert_eq!(s.phase, Phase::Stopping);
        assert_eq!(
            frames(&run(&mut s, ms(t, 5_300), 10_000)),
            stop_burst(),
            "sent again without an answer"
        );
        assert_eq!(
            s.cluster_text(ms(t, 15_500)),
            None,
            "nothing while stopping"
        );
        s.on_status(Status::Off, ms(t, 16_000));
        assert!(!s.is_requested());
        assert_eq!(s.cluster_text(ms(t, 16_000)), None, "nor once off");
    }

    #[test]
    fn when_the_car_ends_it_the_session_cleans_up() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        run(&mut s, t, 300);
        s.on_status(Status::Heating, ms(t, 1_000));
        s.on_status(Status::Off, ms(t, 600_000));
        assert_eq!(frames(&run(&mut s, ms(t, 600_000), 300)), stop_burst());
        assert!(!s.is_requested(), "no confirmation to wait for");
    }

    #[test]
    fn leaving_ready_ends_it_silently() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        s.set_ready(false);
        assert!(!s.is_requested());
        assert!(run(&mut s, t, 1_000).is_empty());
        assert_eq!(s.cluster_text(t), None);
    }

    #[test]
    fn stopping_with_nothing_running_is_refused() {
        let mut s = Session::new();
        assert_eq!(s.request(false, auth(), Instant::from_secs(1)), Err(NOT_ON));
    }

    #[test]
    fn starting_again_keeps_the_running_session() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        run(&mut s, t, 300);
        s.request(true, auth(), ms(t, 1_000)).unwrap();
        assert_eq!(s.phase, Phase::Starting, "no new burst");
        assert_eq!(s.id, 1);
    }

    #[test]
    fn the_route_copy_follows_each_head_unit_frame_while_on() {
        let mut s = Session::new();
        assert!(s.keep_on_copy(&NO_ROUTE).is_none(), "idle");
        s.request(true, auth(), Instant::from_secs(1)).unwrap();
        let copy = s.keep_on_copy(&NO_ROUTE).unwrap();
        assert_eq!(copy, [0, 0, 0, 0, 0, 0x10, 0xA0, 0x00]);
        assert!(s.keep_on_copy(&copy).is_none(), "never a copy of a copy");
        assert!(s.keep_on_copy(&[0; 4]).is_none(), "short frame");
    }

    // ── Through the command handler and the RX path (process-global state) ──

    /// A car identified, approved and unlocked, in Ready with a cold battery, and heard now.
    fn open_the_gate() {
        use crate::proto::advanced_state::{PowerState, PreconditioningState};
        core_interface::reset_can_access_for_tests();
        core_interface::reset_recent_commands_for_tests();
        crate::commands::COMMAND_REPEATS.reset();
        core_interface::revoke_car_identity();
        core_interface::confirm_car_identity();
        assert!(core_interface::confirm_car_fingerprint(b"TEST-CAR"));
        core_interface::approve_can_read_write().unwrap();
        core_interface::set_can_dev_write_unlocked(true).unwrap();
        assert!(!core_interface::set_can_read_only(false));
        crate::commands::CAR_ACTIVITY.touch_at(Instant::now());
        reset_preconditioning_for_tests();
        set_ready(true);
        embassy_futures::block_on(async {
            let mut state = crate::CAR_STATE.lock().await;
            *state = crate::EgmpCarState {
                power_state: Some(PowerState::Ready),
                preconditioning_state: Some(PreconditioningState::Off),
                battery_minimum_temperature: Some(5.0),
                battery_soc: Some(60),
                ..crate::EgmpCarState::new()
            };
        });
        while core_interface::can_tx_channel(BUS_ID)
            .unwrap()
            .try_receive()
            .is_ok()
        {}
    }

    fn command(enable: bool, authorization: Option<CommandAuthorization>) -> Result<(), String> {
        use prost::Message;
        let bytes = crate::proto::AdvancedCommand {
            action: Some(
                crate::proto::advanced_command::Action::BatteryPreconditioningCommand(
                    crate::proto::BatteryPreconditioningCommand { enable },
                ),
            ),
        }
        .encode_to_vec();
        embassy_futures::block_on(crate::process_advanced_command(&bytes, authorization)).map_err(
            |e| match e {
                core_interface::CommandError::Rejected(r) => r.message(),
                core_interface::CommandError::Failed(m) => m.into(),
            },
        )
    }

    fn sent() -> Option<CanTxFrame> {
        core_interface::can_tx_channel(BUS_ID)
            .unwrap()
            .try_receive()
            .ok()
    }

    #[test]
    fn a_command_starts_a_session_that_sends_command_frames() {
        open_the_gate();
        assert_eq!(command(true, Some(auth())), Ok(()));
        assert!(is_requested());
        let published = embassy_futures::block_on(async {
            crate::CAR_STATE
                .lock()
                .await
                .is_manual_preconditioning_requested
        });
        assert_eq!(published, Some(true), "the app is told at once");

        track_preconditioning(0x380, &[0; 8]);
        let frame = sent().expect("the first burst frame");
        assert!(frame.is_command(), "it acts on the car");
        assert_eq!(frame.frame().id, standard_id(HU_EV_E_ID));
        assert_eq!(frame.frame().data, [0, 0, 0, 0x40, 0x03, 0, 0, 0]);

        track_preconditioning(NAV_PRECOND_STATUS_ID, &NO_ROUTE);
        let copy = sent().expect("the route copy");
        assert!(copy.is_command());
        assert_eq!(copy.frame().id, standard_id(NAV_PRECOND_STATUS_ID as u16));
        assert_eq!(copy.frame().data[5..], [0x10, 0xA0, 0x00]);
        reset_preconditioning_for_tests();
    }

    #[test]
    fn mqtt_cannot_start_it() {
        open_the_gate();
        assert_eq!(
            command(true, None),
            Err("command rejected: no validated phone sent it".into())
        );
        assert!(!is_requested());
    }

    #[test]
    fn the_cars_own_preconditioning_refuses_the_command() {
        open_the_gate();
        embassy_futures::block_on(async {
            crate::CAR_STATE.lock().await.preconditioning_state = Some(Status::Preparing);
        });
        assert_eq!(
            command(true, Some(auth())),
            Err(
                "command rejected: the car is already preconditioning the battery on its own"
                    .into()
            )
        );
        assert_eq!(command(false, Some(auth())), Err(NOT_ON.into()));
        assert!(sent().is_none());
    }

    #[test]
    fn a_new_session_has_a_new_id() {
        let mut s = Session::new();
        let t = Instant::from_secs(100);
        s.request(true, auth(), t).unwrap();
        s.set_ready(false);
        s.request(true, auth(), ms(t, 1_000)).unwrap();
        assert_eq!(s.cluster_text(ms(t, 1_000)).unwrap().0, 2);
    }
}
