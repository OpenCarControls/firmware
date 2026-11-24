use core_interface::{
    BLE_TX_CHANNEL, CAN_BUS_HEALTH_REPEAT_MS, CanRxCounts, MAX_CAN_BUSES, init, proto,
    publish_can_bus_health, report_can_bus_window, resend_can_bus_health,
    reset_can_bus_health_for_tests,
};
use proto::CanBusStatus as S;
use std::collections::BTreeMap;

fn boot() {
    init(0xDEAD_BEEF);
    reset_can_bus_health_for_tests();
    core_interface::set_link_up(core_interface::Transport::Ble, true);
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
}

fn frames(received: u32) -> CanRxCounts {
    CanRxCounts {
        received,
        matched: received / 2,
        ..CanRxCounts::new()
    }
}

fn sent() -> Option<BTreeMap<u32, proto::CanBusHealth>> {
    match BLE_TX_CHANNEL.try_receive().ok()?.payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => {
            assert!(update.vehicle_state.is_none());
            Some(update.system_state?.can_bus_health)
        }
        other => panic!("unexpected message {other:?}"),
    }
}

fn status(health: &proto::CanBusHealth) -> S {
    S::try_from(health.status.unwrap()).unwrap()
}

#[test]
fn nothing_is_published_before_a_bus_reports() {
    boot();
    assert!(!publish_can_bus_health(0));
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn a_report_is_published_with_totals_since_boot() {
    boot();
    report_can_bus_window(0, &frames(10), Some(true));
    report_can_bus_window(0, &frames(4), Some(true));
    let mut faulty = CanRxCounts::new();
    faulty.record_error();
    faulty.record_error();
    report_can_bus_window(1, &faulty, None);

    assert!(publish_can_bus_health(1_000));
    let health = sent().unwrap();
    assert_eq!(health.len(), 2);

    let bus0 = &health[&0];
    assert_eq!(status(bus0), S::Ok);
    assert_eq!(bus0.listen_only, Some(true));
    assert_eq!(bus0.frames_received, Some(14));
    assert_eq!(bus0.frames_matched, Some(7));
    assert_eq!(bus0.rx_errors, Some(0));
    assert_eq!(bus0.overruns, Some(0));
    assert_eq!(bus0.bus_offs, Some(0));

    let bus1 = &health[&1];
    assert_eq!(status(bus1), S::NoValidFrames);
    assert_eq!(bus1.listen_only, None);
    assert_eq!(bus1.rx_errors, Some(2));
}

#[test]
fn the_status_follows_the_last_window_only() {
    boot();
    let mut bus_off = frames(3);
    bus_off.record_bus_off();
    report_can_bus_window(0, &bus_off, Some(false));
    report_can_bus_window(0, &CanRxCounts::new(), Some(false));
    assert!(publish_can_bus_health(0));
    let bus0 = sent().unwrap()[&0];
    assert_eq!(status(&bus0), S::Silent);
    assert_eq!(bus0.bus_offs, Some(1));
    assert_eq!(bus0.frames_received, Some(3));
}

#[test]
fn a_status_change_is_published_at_once_and_counters_only_after_the_repeat() {
    boot();
    report_can_bus_window(0, &frames(10), Some(true));
    assert!(publish_can_bus_health(1_000));
    sent().unwrap();

    // Same status, new counters: held until the repeat interval.
    report_can_bus_window(0, &frames(10), Some(true));
    assert!(!publish_can_bus_health(2_000));
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
    assert!(publish_can_bus_health(1_000 + CAN_BUS_HEALTH_REPEAT_MS));
    assert_eq!(sent().unwrap()[&0].frames_received, Some(20));

    // A new status goes out at once.
    let now = 1_000 + CAN_BUS_HEALTH_REPEAT_MS + 100;
    report_can_bus_window(0, &CanRxCounts::new(), Some(true));
    assert!(publish_can_bus_health(now));
    assert_eq!(status(&sent().unwrap()[&0]), S::Silent);

    // So does a change of mode, and a bus that appears.
    report_can_bus_window(0, &CanRxCounts::new(), Some(false));
    assert!(publish_can_bus_health(now + 1));
    sent().unwrap();
    report_can_bus_window(1, &CanRxCounts::new(), Some(false));
    assert!(publish_can_bus_health(now + 2));
    assert_eq!(sent().unwrap().len(), 2);
}

#[test]
fn resend_publishes_again_even_if_unchanged() {
    boot();
    report_can_bus_window(0, &frames(1), Some(true));
    assert!(publish_can_bus_health(1_000));
    sent().unwrap();
    assert!(!publish_can_bus_health(1_100));
    resend_can_bus_health();
    assert!(publish_can_bus_health(1_100));
    assert!(sent().is_some());
}

#[test]
fn nothing_is_queued_while_ble_is_down_and_it_is_retried() {
    boot();
    report_can_bus_window(0, &frames(1), Some(true));
    core_interface::set_link_up(core_interface::Transport::Ble, false);
    assert!(!publish_can_bus_health(1_000));
    core_interface::set_link_up(core_interface::Transport::Ble, true);
    assert!(publish_can_bus_health(1_001));
    assert!(sent().is_some());
}

#[test]
fn a_bus_id_beyond_the_limit_is_ignored() {
    boot();
    report_can_bus_window(MAX_CAN_BUSES as u8, &frames(1), Some(true));
    assert!(!publish_can_bus_health(0));
}

#[test]
fn a_full_state_update_fits_a_ble_message() {
    use prost::Message;
    boot();
    let worst = CanRxCounts {
        received: u32::MAX,
        matched: u32::MAX,
        errors: u32::MAX,
        overruns: u32::MAX,
        bus_offs: u32::MAX,
    };
    for bus in 0..MAX_CAN_BUSES as u8 {
        report_can_bus_window(bus, &worst, Some(false));
    }
    let msg = core_interface::can_bus_health_state_update(u64::MAX);
    assert!(msg.encoded_len() <= core_interface::ble::BLE_MAX_MESSAGE_BYTES);
}
