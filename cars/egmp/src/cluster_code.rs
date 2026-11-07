//! Shows a phone validation code on the instrument cluster (`egmp.cluster_code`, see
//! `docs/can.md`, "E-GMP cluster code").
//!
//! The code goes in the popup the cluster shows for a new track: copies of the head
//! unit's MEDIA_STATUS with the popup byte set, then the text over ISO-TP on the playing
//! source's channel, as the head unit does (protocol from wicant-i-precondition,
//! github.com/L1Z3/wicant-i-precondition). A popup lasts 4 s from its trigger and a trigger
//! in that time extends it, so triggers every [`REFRESH_PERIOD_MS`] keep the code up for
//! the whole session. Text alone updates an open popup but doesn't extend it: that's what
//! animates the dots, which show the code is live.
//!
//! This needs a source with a text channel. The device taps M-CAN, so it can't stop the
//! head unit's own MEDIA_STATUS: forcing a source while the audio is off makes the cluster
//! flip between the two sources, and the text never shows.
//!
//! Driven by received frames, from `can_rx_task`, so nothing is sent on a sleeping bus.

use alloc::vec::Vec;
use core::cell::{Cell, RefCell};

use core_interface::{
    CODE_LENGTH, CanFrame, CanTxFrame, IsoTpSender, ValidationChallenge, ValidationKind,
    ValidationMethod, standard_id,
};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant};

/// Stable identifier the app maps to its text.
pub const METHOD_ID: &str = "egmp.cluster_code";

/// The validation methods, in the order the app offers them.
pub static VALIDATION_METHODS: [ValidationMethod; 1] = [ValidationMethod {
    id: METHOD_ID,
    kind: ValidationKind::CodeEntry,
    needs_approval: true,
    is_available: || current_text_channel().is_some(),
}];

const BUS_ID: u8 = 0;
pub(crate) const MEDIA_STATUS_ID: u32 = 0x4CE;
/// MEDIA_STATUS byte 1 that opens the popup.
const POPUP_TRIGGER: u8 = 0x11;
/// Frames that carry it for one trigger.
const TRIGGER_FRAMES: u8 = 3;
/// From the last trigger frame to the text.
const TRIGGER_SETTLE_MS: u64 = 5;
/// Between two triggers. Checked on an EV6: 3.5 s holds a popup open with no limit seen and
/// no flicker. Triggers wait for the next MEDIA_STATUS, so a gap can reach about 3.7 s.
const REFRESH_PERIOD_MS: u64 = 3_500;
/// Texts per trigger, one per step of the dots.
const TEXTS_PER_REFRESH: u8 = 3;
/// Between two texts. The last one of a cycle stays well before the next trigger: on an EV6,
/// text just before a trigger stopped the popup from being extended.
const TEXT_PERIOD_MS: u64 = REFRESH_PERIOD_MS / TEXTS_PER_REFRESH as u64;
/// How long the cluster has to answer a first frame.
const FLOW_CONTROL_TIMEOUT_MS: u64 = 500;
/// What the head unit pads its text frames with.
const PADDING: u8 = 0xAA;

/// The ISO-TP pair a source's text uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct TextChannel {
    text: u16,
    flow_control: u16,
}

const fn channel(text: u16, flow_control: u16) -> Option<TextChannel> {
    Some(TextChannel { text, flow_control })
}

/// The channel of a MEDIA_STATUS source. `None` for a source without one, the audio off
/// (0x14) and the car off (0xFE). Android Auto and radio were checked on an EV6; the others
/// are from wicant-i-precondition.
fn text_channel(source: u8) -> Option<TextChannel> {
    match source {
        0x02..=0x07
        | 0x2A..=0x2F
        | 0x3E..=0x41
        | 0x4E..=0x53
        | 0x5C..=0x61
        | 0x74..=0x75
        | 0x7A..=0x7B
        | 0x80..=0x81
        | 0x86..=0x89
        | 0x90..=0x95
        | 0x9A..=0x9F
        | 0xAA..=0xB1
        | 0xB8..=0xB9
        | 0xBE..=0xBF
        | 0xDA..=0xDB
        | 0xE0
        | 0xEA..=0xED
        | 0xF0..=0xF3 => channel(0x6E0, 0x6BE), // apps
        0x10..=0x11 => channel(0x6D4, 0x6B4), // Bluetooth
        0x1A..=0x1B | 0xC2..=0xC3 => channel(0x6E1, 0x6BF), // Pandora, iPod
        0x22..=0x23 | 0x28..=0x29 | 0x42..=0x49 => channel(0x6DC, 0x6BB), // radio
        0x62..=0x63 => channel(0x6EC, 0x6CA), // SiriusXM
        0x6C..=0x6D => channel(0x6D8, 0x6B8), // DAB
        0x82..=0x83 | 0x8A..=0x8D => channel(0x6D9, 0x6B9), // DMB, TV
        0xC4 | 0xC8..=0xCA => channel(0x6EA, 0x6C8), // USB
        0xDC..=0xDD => channel(0x6D6, 0x6B6), // CarPlay
        0xDE..=0xDF => channel(0x6D2, 0x6B2), // Android Auto
        _ => None,
    }
}

/// The code as the cluster shows it, in UTF-16LE: the digits in two groups, then `dots`.
fn code_text(code: &[u8; CODE_LENGTH], dots: usize) -> Vec<u8> {
    let (first, second) = code.split_at(CODE_LENGTH / 2);
    first
        .iter()
        .chain(b" ")
        .chain(second)
        .chain(b" ")
        .chain(core::iter::repeat_n(&b'.', dots))
        .flat_map(|&c| [c, 0])
        .collect()
}

/// What to show now.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Shown {
    session: u32,
    code: [u8; CODE_LENGTH],
    channel: TextChannel,
}

struct Transfer {
    sender: IsoTpSender,
    channel: TextChannel,
    deadline: Instant,
}

/// A frame to send: its ID and data.
type ToSend = (u16, [u8; 8]);

pub(crate) struct ClusterCode {
    session: Option<u32>,
    /// When the next trigger is due, `None` for the next MEDIA_STATUS.
    next_trigger: Option<Instant>,
    triggers_left: u8,
    /// When the next text is due, `None` once this cycle's are out.
    next_text: Option<Instant>,
    texts_left: u8,
    dots: usize,
    transfer: Option<Transfer>,
}

impl ClusterCode {
    pub(crate) const fn new() -> Self {
        Self {
            session: None,
            next_trigger: None,
            triggers_left: 0,
            next_text: None,
            texts_left: 0,
            dots: 1,
            transfer: None,
        }
    }

    /// Moves on after a frame received at `now`, and returns what to send, if anything.
    /// With nothing to show it forgets everything, so showing again starts a new popup.
    pub(crate) fn on_frame(
        &mut self,
        shown: Option<&Shown>,
        id: u32,
        data: &[u8],
        now: Instant,
    ) -> Option<ToSend> {
        let Some(shown) = shown else {
            *self = Self::new();
            return None;
        };
        if self.session != Some(shown.session) {
            *self = Self::new();
            self.session = Some(shown.session);
        }
        if id == MEDIA_STATUS_ID
            && let Some(frame) = self.trigger(data, now)
        {
            return Some((MEDIA_STATUS_ID as u16, frame));
        }
        if self.transfer.is_some() {
            return self.continue_transfer(id, data, now);
        }
        self.start_text(shown, now)
    }

    /// The copy of MEDIA_STATUS to send, while a trigger is running.
    fn trigger(&mut self, media_status: &[u8], now: Instant) -> Option<[u8; 8]> {
        if self.triggers_left == 0 && self.next_trigger.is_none_or(|at| now >= at) {
            self.triggers_left = TRIGGER_FRAMES;
            self.next_trigger = Some(now + Duration::from_millis(REFRESH_PERIOD_MS));
            self.next_text = None;
        }
        if self.triggers_left == 0 {
            return None;
        }
        let mut frame: [u8; 8] = media_status.try_into().ok()?;
        frame[1] = POPUP_TRIGGER;
        self.triggers_left -= 1;
        if self.triggers_left == 0 {
            self.next_text = Some(now + Duration::from_millis(TRIGGER_SETTLE_MS));
            self.texts_left = TEXTS_PER_REFRESH;
        }
        Some(frame)
    }

    fn start_text(&mut self, shown: &Shown, now: Instant) -> Option<ToSend> {
        let due = self.next_text.filter(|&at| now >= at)?;
        self.texts_left -= 1;
        self.next_text = (self.texts_left > 0).then(|| due + Duration::from_millis(TEXT_PERIOD_MS));
        let mut sender = IsoTpSender::new(code_text(&shown.code, self.dots), PADDING);
        self.dots = self.dots % TEXTS_PER_REFRESH as usize + 1;
        let frame = sender.first_frame();
        self.transfer = Some(Transfer {
            sender,
            channel: shown.channel,
            deadline: now + Duration::from_millis(FLOW_CONTROL_TIMEOUT_MS),
        });
        Some((shown.channel.text, frame))
    }

    /// The next frame of the text. A text lost to a title sent at the same time, or that
    /// the cluster doesn't answer, is replaced by the next one.
    fn continue_transfer(&mut self, id: u32, data: &[u8], now: Instant) -> Option<ToSend> {
        let transfer = self.transfer.as_mut()?;
        let channel = transfer.channel;
        if id == channel.flow_control as u32 {
            transfer.sender.on_flow_control(data, now.as_millis());
        }
        let frame = transfer.sender.next_frame(now.as_millis());
        let timed_out = transfer.sender.is_awaiting_flow_control() && now > transfer.deadline;
        if transfer.sender.is_finished() || timed_out {
            self.transfer = None;
        }
        frame.map(|frame| (channel.text, frame))
    }
}

static DISPLAY: Mutex<CriticalSectionRawMutex, RefCell<ClusterCode>> =
    Mutex::new(RefCell::new(ClusterCode::new()));

/// What the method's availability depends on, kept apart from `CAR_STATE` because
/// [`ValidationMethod::is_available`] can't wait for a lock.
static READY: Mutex<CriticalSectionRawMutex, Cell<bool>> = Mutex::new(Cell::new(false));
static MEDIA_SOURCE: Mutex<CriticalSectionRawMutex, Cell<Option<u8>>> = Mutex::new(Cell::new(None));

/// Records whether the car is in Ready.
pub(crate) fn set_ready(ready: bool) {
    READY.lock(|r| r.set(ready));
}

/// The channel to show the code on, while the car allows it: in Ready, with a source that
/// has one.
fn current_text_channel() -> Option<TextChannel> {
    if !READY.lock(Cell::get) {
        return None;
    }
    text_channel(MEDIA_SOURCE.lock(Cell::get)?)
}

fn shown_now(challenge: Option<ValidationChallenge>) -> Option<Shown> {
    let challenge = challenge.filter(|c| c.method_id == METHOD_ID)?;
    if core_interface::is_can_read_only() {
        return None;
    }
    Some(Shown {
        session: challenge.session,
        code: challenge.code?,
        channel: current_text_channel()?,
    })
}

/// Runs the display on a received frame.
pub(crate) fn track_cluster_code(id: u32, data: &[u8]) {
    if id == MEDIA_STATUS_ID
        && let Some(&source) = data.first()
    {
        MEDIA_SOURCE.lock(|s| s.set(Some(source)));
    }
    let shown = shown_now(core_interface::current_validation_challenge());
    let frame = DISPLAY.lock(|d| {
        d.borrow_mut()
            .on_frame(shown.as_ref(), id, data, Instant::now())
    });
    if let Some((id, data)) = frame {
        let frame = CanFrame {
            bus_id: BUS_ID,
            id: standard_id(id),
            data,
            dlc: 8,
        };
        // Never block the RX path: a full channel loses the frame, and the next text
        // replaces it. A platform frame: it only changes what the cluster displays.
        let _ = core_interface::can_tx_channel(BUS_ID)
            .map(|channel| channel.try_send(CanTxFrame::platform(frame)));
    }
}

#[cfg(test)]
pub(crate) fn reset_cluster_code_for_tests() {
    DISPLAY.lock(|d| *d.borrow_mut() = ClusterCode::new());
    set_ready(false);
    MEDIA_SOURCE.lock(|s| s.set(None));
}

#[cfg(test)]
mod tests {
    use super::*;

    const AA: TextChannel = TextChannel {
        text: 0x6D2,
        flow_control: 0x6B2,
    };
    const RADIO: TextChannel = TextChannel {
        text: 0x6DC,
        flow_control: 0x6BB,
    };
    const CODE: [u8; CODE_LENGTH] = *b"482913";
    /// The head unit with Android Auto playing, on an EV6 (2026-11-05).
    const MEDIA_STATUS: [u8; 8] = [0xDE, 0x01, 0x00, 0x00, 0xA0, 0x00, 0xF8, 0xFF];
    /// The EV6 cluster's answer to a first frame.
    const FLOW_CONTROL: [u8; 8] = [0x30, 0x00, 0x05, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA];
    /// Any other frame from the car (CLIMATE_TEMPERATURE).
    const CAR_ID: u32 = 0x380;

    fn shown(session: u32, channel: TextChannel) -> Shown {
        Shown {
            session,
            code: CODE,
            channel,
        }
    }

    fn ms(t: Instant, ms: u64) -> Instant {
        t + Duration::from_millis(ms)
    }

    /// Runs the three trigger frames, 200 ms apart, from `t`. Returns when the last went out.
    fn trigger(display: &mut ClusterCode, session: &Shown, t: Instant) -> Instant {
        for i in 0..3 {
            let (id, frame) = display
                .on_frame(
                    Some(session),
                    MEDIA_STATUS_ID,
                    &MEDIA_STATUS,
                    ms(t, i * 200),
                )
                .expect("a trigger frame");
            assert_eq!(id, 0x4CE);
            let mut expected = MEDIA_STATUS;
            expected[1] = 0x11;
            assert_eq!(frame, expected);
        }
        ms(t, 400)
    }

    #[test]
    fn a_popup_is_a_trigger_then_the_text() {
        let mut display = ClusterCode::new();
        let session = shown(1, AA);
        let t = Instant::from_secs(100);
        let last_trigger = trigger(&mut display, &session, t);

        assert_eq!(
            display.on_frame(Some(&session), CAR_ID, &[0; 8], ms(last_trigger, 4)),
            None,
            "the text waits for the trigger to settle"
        );
        // The frames sent to the EV6 in the tests of 2026-11-05.
        let t = ms(last_trigger, 5);
        assert_eq!(
            display.on_frame(Some(&session), CAR_ID, &[0; 8], t),
            Some((0x6D2, [0x10, 0x12, 0x34, 0x00, 0x38, 0x00, 0x32, 0x00]))
        );
        assert_eq!(
            display.on_frame(Some(&session), CAR_ID, &[0; 8], ms(t, 1)),
            None,
            "waits for the flow control"
        );
        assert_eq!(
            display.on_frame(Some(&session), 0x6B2, &FLOW_CONTROL, ms(t, 40)),
            Some((0x6D2, [0x21, 0x20, 0x00, 0x39, 0x00, 0x31, 0x00, 0x33]))
        );
        assert_eq!(
            display.on_frame(Some(&session), CAR_ID, &[0; 8], ms(t, 44)),
            None,
            "the cluster's separation time"
        );
        assert_eq!(
            display.on_frame(Some(&session), CAR_ID, &[0; 8], ms(t, 45)),
            Some((0x6D2, [0x22, 0x00, 0x20, 0x00, 0x2E, 0x00, 0xAA, 0xAA]))
        );
        assert_eq!(
            display.on_frame(Some(&session), CAR_ID, &[0; 8], ms(t, 50)),
            None
        );
    }

    #[test]
    fn the_text_uses_the_sources_channel() {
        let mut display = ClusterCode::new();
        let session = shown(1, RADIO);
        let last_trigger = trigger(&mut display, &session, Instant::from_secs(100));
        let (id, _) = display
            .on_frame(Some(&session), CAR_ID, &[0; 8], ms(last_trigger, 5))
            .unwrap();
        assert_eq!(id, 0x6DC);
        assert!(
            display
                .on_frame(Some(&session), 0x6B2, &FLOW_CONTROL, ms(last_trigger, 40))
                .is_none(),
            "another source's flow control"
        );
        assert!(
            display
                .on_frame(Some(&session), 0x6BB, &FLOW_CONTROL, ms(last_trigger, 41))
                .is_some()
        );
    }

    /// One entry per frame sent: (ms since the start, ID, the frame).
    fn run_bus(
        display: &mut ClusterCode,
        session: &Shown,
        seconds: u64,
    ) -> Vec<(u64, u16, [u8; 8])> {
        let start = Instant::from_secs(100);
        let mut sent = Vec::new();
        for now in 0..seconds * 1000 {
            let t = ms(start, now);
            let (id, data) = if now % 200 == 0 {
                (MEDIA_STATUS_ID, MEDIA_STATUS)
            } else {
                (CAR_ID, [0; 8])
            };
            let mut replies = Vec::new();
            if let Some((tx, frame)) = display.on_frame(Some(session), id, &data, t) {
                sent.push((now, tx, frame));
                if frame[0] >> 4 == 1 {
                    replies.push(FLOW_CONTROL);
                }
            }
            for reply in replies {
                if let Some((tx, frame)) = display.on_frame(
                    Some(session),
                    session.channel.flow_control as u32,
                    &reply,
                    t,
                ) {
                    sent.push((now, tx, frame));
                }
            }
        }
        sent
    }

    fn first_of_each_trigger(sent: &[(u64, u16, [u8; 8])]) -> Vec<u64> {
        let triggers: Vec<u64> = sent
            .iter()
            .filter(|(_, id, _)| *id == 0x4CE)
            .map(|(at, ..)| *at)
            .collect();
        triggers.iter().copied().step_by(3).collect()
    }

    #[test]
    fn triggers_keep_the_popup_open_for_a_whole_session() {
        let mut display = ClusterCode::new();
        let sent = run_bus(&mut display, &shown(1, AA), 120);
        let triggers = first_of_each_trigger(&sent);
        assert_eq!(triggers[0], 0);
        assert!(triggers.len() >= 33, "{}", triggers.len());
        for pair in triggers.windows(2) {
            let gap = pair[1] - pair[0];
            assert!((3_500..=3_700).contains(&gap), "{gap} ms");
        }
    }

    #[test]
    fn the_dots_step_evenly_and_never_right_before_a_trigger() {
        let mut display = ClusterCode::new();
        let sent = run_bus(&mut display, &shown(1, AA), 20);
        let texts: Vec<(u64, u8)> = sent
            .iter()
            .filter(|(_, id, frame)| *id == 0x6D2 && frame[0] == 0x10)
            .map(|(at, _, frame)| (*at, frame[1]))
            .collect();
        // 16 bytes of digits and spaces, then 2 per dot.
        let lengths: Vec<u8> = texts.iter().map(|(_, len)| *len).collect();
        assert_eq!(lengths[..7], [18, 20, 22, 18, 20, 22, 18]);

        for pair in texts.windows(2) {
            let gap = pair[1].0 - pair[0].0;
            assert!((1_100..=1_400).contains(&gap), "{gap} ms");
        }
        let triggers = first_of_each_trigger(&sent);
        for trigger in &triggers[1..] {
            let before = texts.iter().rev().find(|(at, _)| at < trigger).unwrap().0;
            assert!(trigger - before >= 500, "{before} ms, then {trigger} ms");
        }
    }

    #[test]
    fn no_flow_control_drops_the_text_and_the_next_one_goes_out() {
        let mut display = ClusterCode::new();
        let session = shown(1, AA);
        let t = Instant::from_secs(100);
        let last_trigger = trigger(&mut display, &session, t);
        let first = ms(last_trigger, 5);
        assert!(
            display
                .on_frame(Some(&session), CAR_ID, &[0; 8], first)
                .is_some()
        );
        assert_eq!(
            display.on_frame(Some(&session), CAR_ID, &[0; 8], ms(first, 501)),
            None
        );
        let (_, frame) = display
            .on_frame(Some(&session), CAR_ID, &[0; 8], ms(first, TEXT_PERIOD_MS))
            .expect("the next text");
        assert_eq!(frame[..2], [0x10, 20]);
    }

    #[test]
    fn nothing_is_sent_without_something_to_show() {
        let mut display = ClusterCode::new();
        let t = Instant::from_secs(100);
        assert_eq!(
            display.on_frame(None, MEDIA_STATUS_ID, &MEDIA_STATUS, t),
            None
        );
    }

    #[test]
    fn stopping_ends_the_text_and_showing_again_starts_a_popup() {
        let mut display = ClusterCode::new();
        let session = shown(1, AA);
        let last_trigger = trigger(&mut display, &session, Instant::from_secs(100));
        let t = ms(last_trigger, 5);
        assert!(
            display
                .on_frame(Some(&session), CAR_ID, &[0; 8], t)
                .is_some()
        );

        // The session ended, or the car left Ready: not even the transfer goes on.
        assert_eq!(display.on_frame(None, 0x6B2, &FLOW_CONTROL, ms(t, 1)), None);
        assert_eq!(display.on_frame(None, CAR_ID, &[0; 8], ms(t, 10)), None);

        // Available again right after: a new popup, without waiting for the refresh.
        trigger(&mut display, &session, ms(t, 20));
    }

    #[test]
    fn a_new_session_starts_a_new_popup() {
        let mut display = ClusterCode::new();
        let last_trigger = trigger(&mut display, &shown(1, AA), Instant::from_secs(100));
        trigger(&mut display, &shown(2, AA), ms(last_trigger, 100));
    }

    #[test]
    fn only_sources_with_a_text_channel_can_show_the_code() {
        // Checked on an EV6.
        assert_eq!(text_channel(0xDE), Some(AA));
        assert_eq!(text_channel(0x22), Some(RADIO));
        assert_eq!(text_channel(0x14), None, "audio off");
        assert_eq!(text_channel(0xFE), None, "car off");
        assert_eq!(
            text_channel(0x04),
            channel(0x6E0, 0x6BE),
            "only offered if the car reports it"
        );
    }

    #[test]
    fn every_channel_passes_the_can_filters() {
        let passes = |id: u32| {
            crate::CAN_FILTERS.iter().any(|f| {
                let embedded_can::Id::Standard(filter) = f.id else {
                    return false;
                };
                (id ^ filter.as_raw() as u32) & f.mask == 0
            })
        };
        assert!(passes(MEDIA_STATUS_ID));
        for source in 0..=u8::MAX {
            if let Some(channel) = text_channel(source) {
                assert!(passes(channel.flow_control as u32), "{source:#04x}");
            }
        }
    }

    #[test]
    fn the_code_reads_in_two_groups_with_its_dots() {
        let text = code_text(&CODE, 2);
        assert_eq!(
            text,
            "482 913 .."
                .encode_utf16()
                .flat_map(u16::to_le_bytes)
                .collect::<Vec<u8>>()
        );
    }

    // ── Through the car state and the session ─────────────────────────────────

    use core_interface::{
        BLE_TX_CHANNEL, handle_ble_message, proto, proto::app_to_device::Payload,
        proto::system_command::Action,
    };

    const PLATFORM_ID: u32 = 0xEB6E_0001;
    const PHONE: &[u8] = b"ble:\x01\x02\x03\x04\x05\x06";

    fn random(buf: &mut [u8]) {
        buf.fill(7);
    }

    fn drain_tx() {
        let channel = core_interface::can_tx_channel(BUS_ID).unwrap();
        while channel.try_receive().is_ok() {}
    }

    /// A car identified and approved, with one bonded phone.
    fn setup() {
        core_interface::init(PLATFORM_ID);
        core_interface::revoke_car_identity();
        core_interface::reset_can_access_for_tests();
        core_interface::reset_phone_trust_for_tests();
        core_interface::reset_phone_validation_for_tests();
        core_interface::reset_recent_commands_for_tests();
        reset_cluster_code_for_tests();
        core_interface::declare_validation_methods(&VALIDATION_METHODS);
        core_interface::set_random_source(random);
        core_interface::set_link_up(core_interface::Transport::Ble, true);
        while BLE_TX_CHANNEL.try_receive().is_ok() {}
        embassy_futures::block_on(core_interface::clear_paired_phones());
        embassy_futures::block_on(core_interface::add_paired_phone(PHONE));
        core_interface::confirm_car_identity();
        assert!(core_interface::confirm_car_fingerprint(b"hu-serial:TEST"));
        core_interface::approve_can_read_write().unwrap();
        core_interface::set_can_dev_write_unlocked(true).unwrap();
        core_interface::set_can_read_only(false);
        drain_tx();
    }

    fn command(action: Action) -> proto::CommandResponse {
        static NEXT: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(1);
        let msg = proto::AppToDevice {
            message_id: NEXT.fetch_add(1, core::sync::atomic::Ordering::Relaxed),
            platform_id: PLATFORM_ID,
            source_device_id: PHONE.to_vec(),
            payload: Some(Payload::SystemCommand(proto::SystemCommand {
                action: Some(action),
            })),
        };
        embassy_futures::block_on(handle_ble_message(msg));
        match BLE_TX_CHANNEL.try_receive().unwrap().payload {
            Some(proto::device_to_app::Payload::CommandResponse(resp)) => resp,
            other => panic!("{other:?}"),
        }
    }

    fn start() -> proto::CommandResponse {
        command(Action::StartPhoneValidation(
            proto::StartPhoneValidationCommand {
                method_id: METHOD_ID.into(),
            },
        ))
    }

    fn method_available() -> bool {
        let resp = command(Action::ListPhoneValidationMethods(
            proto::ListPhoneValidationMethodsCommand {},
        ));
        match resp.response_data {
            Some(proto::command_response::ResponseData::SystemResult(r)) => match r.result {
                Some(proto::system_command_result::Result::ValidationMethods(m)) => {
                    m.methods[0].available
                }
                other => panic!("{other:?}"),
            },
            other => panic!("{other:?}"),
        }
    }

    fn sent_frame() -> Option<CanFrame> {
        core_interface::can_tx_channel(BUS_ID)
            .unwrap()
            .try_receive()
            .ok()
            .map(|frame| *frame.frame())
    }

    #[test]
    fn the_method_needs_ready_and_a_source_with_a_text_channel() {
        setup();
        assert!(!method_available());
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert!(!method_available(), "not in Ready");
        let resp = start();
        assert!(!resp.success);
        assert!(
            resp.error_message.contains("current state"),
            "{}",
            resp.error_message
        );

        set_ready(true);
        assert!(method_available());
        track_cluster_code(MEDIA_STATUS_ID, &[0x14, 0x01, 0, 0, 0, 0, 0, 0]);
        assert!(!method_available(), "audio off");
        track_cluster_code(MEDIA_STATUS_ID, &[0x22, 0x01, 0, 0, 0, 0, 0, 0]);
        assert!(method_available(), "radio");
        assert!(start().success);
    }

    #[test]
    fn a_running_session_shows_its_code_until_it_ends() {
        setup();
        set_ready(true);
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert_eq!(sent_frame().map(|f| f.dlc), None, "no session");

        assert!(start().success);
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        let frame = sent_frame().expect("the first trigger frame");
        assert_eq!(frame.id, standard_id(0x4CE));
        assert_eq!(frame.data[1], 0x11);
        assert_eq!(frame.bus_id, BUS_ID);

        command(Action::CancelPhoneValidation(
            proto::CancelPhoneValidationCommand {},
        ));
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert!(sent_frame().is_none(), "the session ended");
    }

    #[test]
    fn the_display_pauses_outside_ready_or_while_read_only() {
        setup();
        set_ready(true);
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert!(start().success);

        set_ready(false);
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert!(sent_frame().is_none(), "out of Ready");
        assert!(
            core_interface::current_validation_challenge().is_some(),
            "the session keeps running"
        );

        set_ready(true);
        core_interface::set_can_dev_write_unlocked(false).unwrap();
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert!(sent_frame().is_none(), "read-only");

        core_interface::set_can_dev_write_unlocked(true).unwrap();
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert!(sent_frame().is_some(), "resumes");
    }

    #[test]
    fn typing_the_code_shown_validates_the_phone() {
        setup();
        set_ready(true);
        track_cluster_code(MEDIA_STATUS_ID, &MEDIA_STATUS);
        assert!(start().success);
        let code = core_interface::current_validation_challenge()
            .unwrap()
            .code
            .unwrap();
        let resp = command(Action::SubmitPhoneValidationCode(
            proto::SubmitPhoneValidationCodeCommand {
                code: core::str::from_utf8(&code).unwrap().into(),
            },
        ));
        assert!(resp.success, "{}", resp.error_message);
        assert!(core_interface::is_phone_validated(PHONE));
    }
}
