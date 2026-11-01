use alloc::vec;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, TrySendError};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use core::task::Poll;
use embassy_time::Instant;

use crate::channels::{
    BLE_STATE_TX, BLE_TX_CHANNEL, CMD_RESP_CHANNEL, MQTT_STATE_TX, MQTT_TX_CHANNEL,
    VEHICLE_STATE_CHANNEL,
};
use crate::proto;
use crate::types::{Transport, VehicleStatePayload};

// ── MQTT Throttle ─────────────────────────────────────────────────────────────

/// Seconds of silence from any MQTT client before entering idle rate.
const MQTT_ACTIVITY_WINDOW_S: u32 = 35;
/// Minimum gap between MQTT state sends while no client is active.
pub(crate) const MQTT_IDLE_RATE_S: u32 = 30;

/// Uptime seconds of the most recent inbound MQTT message (command or heartbeat).
static LAST_MQTT_RX_SECS: AtomicU32 = AtomicU32::new(0);
/// Uptime seconds of the most recent MQTT state message actually sent.
static LAST_MQTT_STATE_SECS: AtomicU32 = AtomicU32::new(0);

/// Called by `handle_mqtt_message` whenever any valid MQTT message arrives
/// (command, heartbeat, or any other payload). Keeps the activity window alive
/// so state updates continue to flow at full rate.
pub fn record_mqtt_activity(secs: u32) {
    LAST_MQTT_RX_SECS.store(secs, Ordering::Relaxed);
}

/// Whether an MQTT client has been active within the last [`MQTT_ACTIVITY_WINDOW_S`] seconds,
/// i.e. whether MQTT messages go out at the full rate or the idle rate.
pub(crate) fn is_mqtt_active(now_secs: u32) -> bool {
    now_secs.saturating_sub(LAST_MQTT_RX_SECS.load(Ordering::Relaxed)) <= MQTT_ACTIVITY_WINDOW_S
}

/// Resets MQTT throttle state to boot defaults. For use in tests only.
pub fn reset_mqtt_throttle_for_tests() {
    LAST_MQTT_RX_SECS.store(0, Ordering::Relaxed);
    LAST_MQTT_STATE_SECS.store(0, Ordering::Relaxed);
}

// ── Transport links ───────────────────────────────────────────────────────────

/// Whether a transport can deliver messages right now: a phone is connected over
/// BLE, or the MQTT session with the broker is up. Boards report it with
/// [`set_link_up`]. A message for a transport whose link is down is dropped, and
/// whatever is queued when a link goes down is cleared: it was meant for the peer
/// that left, never the next one.
struct Link {
    up: AtomicBool,
    /// Bumped each time the link goes down, so a send that waited for room can
    /// tell the peer it was meant for is gone, even if another one connected since.
    generation: AtomicU32,
}

static BLE_LINK: Link = Link {
    up: AtomicBool::new(false),
    generation: AtomicU32::new(0),
};
static MQTT_LINK: Link = Link {
    up: AtomicBool::new(false),
    generation: AtomicU32::new(0),
};

fn link(transport: Transport) -> &'static Link {
    match transport {
        Transport::Ble => &BLE_LINK,
        Transport::Mqtt => &MQTT_LINK,
    }
}

fn tx_channel(
    transport: Transport,
) -> &'static Channel<CriticalSectionRawMutex, proto::DeviceToApp, 4> {
    match transport {
        Transport::Ble => &BLE_TX_CHANNEL,
        Transport::Mqtt => &MQTT_TX_CHANNEL,
    }
}

/// Called by the board's transport driver when its link comes up (a phone
/// connected, the MQTT session is ready) and when it goes down. Going down clears
/// that transport's TX queue.
pub fn set_link_up(transport: Transport, up: bool) {
    let link = link(transport);
    if up {
        link.up.store(true, Ordering::Relaxed);
    } else {
        link.up.store(false, Ordering::Relaxed);
        link.generation.fetch_add(1, Ordering::Relaxed);
        while tx_channel(transport).try_receive().is_ok() {}
    }
}

/// Whether the board reported `transport`'s link as up.
pub fn is_link_up(transport: Transport) -> bool {
    link(transport).up.load(Ordering::Relaxed)
}

/// Queues `msg` on `transport`'s TX queue without waiting. Dropped if the link is
/// down or the queue is full. Returns whether it was queued.
pub(crate) fn try_send_to(transport: Transport, msg: proto::DeviceToApp) -> bool {
    is_link_up(transport) && tx_channel(transport).try_send(msg).is_ok()
}

/// Queues `msg` on `transport`'s TX queue, waiting for room while the link stays
/// up. Dropped if the link is down, or goes down before there is room. Returns
/// whether it was queued. A waiting send is woken by any receive, including the
/// clear in [`set_link_up`], so it can never outlive the peer it was meant for.
pub(crate) async fn send_to(transport: Transport, msg: proto::DeviceToApp) -> bool {
    let link = link(transport);
    let channel = tx_channel(transport);
    let generation = link.generation.load(Ordering::Relaxed);
    let mut msg = Some(msg);
    core::future::poll_fn(|cx| {
        loop {
            if !link.up.load(Ordering::Relaxed)
                || link.generation.load(Ordering::Relaxed) != generation
            {
                return Poll::Ready(false);
            }
            let Some(next) = msg.take() else {
                return Poll::Ready(false);
            };
            match channel.try_send(next) {
                Ok(()) => return Poll::Ready(true),
                Err(TrySendError::Full(back)) => msg = Some(back),
            }
            if channel.poll_ready_to_send(cx).is_pending() {
                return Poll::Pending;
            }
        }
    })
    .await
}

/// Resets both links to their boot state (down). For use in tests only.
pub fn reset_links_for_tests() {
    set_link_up(Transport::Ble, false);
    set_link_up(Transport::Mqtt, false);
}

// ── Response Router ───────────────────────────────────────────────────────────

/// Routes a single command response to BLE or MQTT based on the transport tag.
/// The response goes back on the transport the command came in on; if that link is
/// down, or goes down while the response waits for room, it is dropped (the app
/// re-reads the state). `timestamp_ms` is passed explicitly so callers (and tests)
/// control the clock.
pub async fn route_single_response(
    transport: Transport,
    response: proto::CommandResponse,
    timestamp_ms: u64,
) {
    let msg = proto::DeviceToApp {
        timestamp_ms,
        platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
        payload: Some(proto::device_to_app::Payload::CommandResponse(response)),
    };
    send_to(transport, msg).await;
}

/// Reads command responses from `CMD_RESP_CHANNEL` and routes each as a
/// `DeviceToApp(command_response)` to BLE or MQTT based on the transport tag.
///
/// `timestamp_ms` reflects device uptime. SNTP wall-clock sync is future work.
#[embassy_executor::task]
pub async fn route_responses_task() {
    let receiver = CMD_RESP_CHANNEL.receiver();
    loop {
        let (transport, response) = receiver.receive().await;
        route_single_response(transport, response, Instant::now().as_millis()).await;
    }
}

// ── State Publisher ───────────────────────────────────────────────────────────

/// The BLE state message for one update: the full state (basic + advanced).
/// Public so vehicle crates can check their worst-case state still fits in
/// `ble::BLE_MAX_MESSAGE_BYTES` once reassembled.
pub fn ble_state_update(
    timestamp_ms: u64,
    platform_id: u32,
    payload: VehicleStatePayload,
) -> proto::DeviceToApp {
    proto::DeviceToApp {
        timestamp_ms,
        platform_id,
        payload: Some(proto::device_to_app::Payload::StateUpdate(
            proto::StateUpdate {
                system_state: None,
                vehicle_state: Some(proto::VehicleState {
                    basic_state_bytes: payload.basic,
                    advanced_state_bytes: payload.advanced,
                }),
            },
        )),
    }
}

/// The last BLE state message published, so the BLE transport can resend it to
/// an app that subscribes after it was sent (`notify` drops it for an
/// unsubscribed peer).
static LATEST_BLE_STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<proto::DeviceToApp>>> =
    Mutex::new(RefCell::new(None));

/// The most recent BLE state update (full state, original timestamp), or
/// `None` if the vehicle hasn't published one since boot.
pub fn latest_ble_state() -> Option<proto::DeviceToApp> {
    LATEST_BLE_STATE.lock(|latest| latest.borrow().clone())
}

/// Forgets the cached BLE state update. For use in tests only.
pub fn reset_latest_ble_state_for_tests() {
    LATEST_BLE_STATE.lock(|latest| *latest.borrow_mut() = None);
}

/// Publishes a single vehicle state update.
/// - BLE receives the full state (basic + advanced).
/// - MQTT receives basic only (`advanced_state_bytes` is always empty).
///
/// `timestamp_ms` is passed explicitly so callers (and tests) control the clock.
///
/// State updates don't go through the TX queues: each transport has a single-value
/// slot (`BLE_STATE_TX`, `MQTT_STATE_TX`) that the newest update overwrites, since
/// the latest reading always supersedes older ones. This never blocks, so
/// `publish_state_task` can't stall `VEHICLE_STATE_CHANNEL` and every producer task
/// behind it (`state_update_task`, command handlers), and a state update can never
/// push a command response out of a queue.
pub async fn publish_single_state(payload: VehicleStatePayload, timestamp_ms: u64) {
    let pid = crate::PLATFORM_ID.load(Ordering::Relaxed);

    // BLE: full state (basic + advanced)
    let basic = payload.basic.clone();
    let ble_msg = ble_state_update(timestamp_ms, pid, payload);
    LATEST_BLE_STATE.lock(|latest| *latest.borrow_mut() = Some(ble_msg.clone()));
    BLE_STATE_TX.signal(ble_msg);

    // MQTT: basic only (advanced is BLE-exclusive), throttled when no client is active.
    let now_secs = (timestamp_ms / 1000) as u32;
    let last_rx = LAST_MQTT_RX_SECS.load(Ordering::Relaxed);
    let mqtt_active = now_secs.saturating_sub(last_rx) <= MQTT_ACTIVITY_WINDOW_S;
    let should_send_mqtt = if mqtt_active {
        true
    } else {
        let last_state = LAST_MQTT_STATE_SECS.load(Ordering::Relaxed);
        now_secs.saturating_sub(last_state) >= MQTT_IDLE_RATE_S
    };

    if should_send_mqtt {
        let mqtt_msg = proto::DeviceToApp {
            timestamp_ms,
            platform_id: pid,
            payload: Some(proto::device_to_app::Payload::StateUpdate(
                proto::StateUpdate {
                    system_state: None,
                    vehicle_state: Some(proto::VehicleState {
                        basic_state_bytes: basic,
                        advanced_state_bytes: vec![],
                    }),
                },
            )),
        };
        MQTT_STATE_TX.signal(mqtt_msg);
        LAST_MQTT_STATE_SECS.store(now_secs, Ordering::Relaxed);
    }
}

/// Reads `VehicleStatePayload` from `VEHICLE_STATE_CHANNEL` and publishes two
/// outbound `DeviceToApp(state_update)` messages per update:
/// - BLE: full `VehicleState` (basic + advanced bytes)
/// - MQTT: basic only (`advanced_state_bytes` omitted — BLE-exclusive)
///
/// `timestamp_ms` reflects device uptime. SNTP wall-clock sync is future work.
#[embassy_executor::task]
pub async fn publish_state_task() {
    let receiver = VEHICLE_STATE_CHANNEL.receiver();
    loop {
        let payload = receiver.receive().await;
        publish_single_state(payload, Instant::now().as_millis()).await;
    }
}
