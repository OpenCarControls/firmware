use core_interface::{
    init, publish_single_debug_batch, CanDebugFilter, CanRawCapture,
    BLE_TX_CHANNEL, MQTT_TX_CHANNEL,
    proto,
};
use embedded_can::{ExtendedId, Id, StandardId};

const PLATFORM_ID: u32 = 0xAB_CD_EF_01;
const TS: u64 = 99_000;

fn drain_tx_channels() {
    while BLE_TX_CHANNEL.try_receive().is_ok() {}
    while MQTT_TX_CHANNEL.try_receive().is_ok() {}
}

fn std_capture(bus_id: u8, raw_id: u16, data: &[u8], timestamp_ms: u64) -> CanRawCapture {
    let mut buf = [0u8; 8];
    buf[..data.len()].copy_from_slice(data);
    CanRawCapture {
        timestamp_ms,
        bus_id,
        id: Id::Standard(StandardId::new(raw_id).unwrap()),
        data: buf,
        dlc: data.len() as u8,
    }
}

fn ext_capture(bus_id: u8, raw_id: u32, data: &[u8]) -> CanRawCapture {
    let mut buf = [0u8; 8];
    buf[..data.len()].copy_from_slice(data);
    CanRawCapture {
        timestamp_ms: TS,
        bus_id,
        id: Id::Extended(ExtendedId::new(raw_id).unwrap()),
        data: buf,
        dlc: data.len() as u8,
    }
}

fn std_filter(raw_id: u16, mask: u32) -> CanDebugFilter {
    CanDebugFilter { can_id: raw_id as u32, is_extended_id: false, mask }
}

fn recv_debug_update() -> proto::CanDebugUpdate {
    let msg = BLE_TX_CHANNEL.try_receive().expect("expected message on BLE_TX_CHANNEL");
    match msg.payload {
        Some(proto::device_to_app::Payload::CanDebugUpdate(u)) => u,
        other => panic!("expected CanDebugUpdate, got {:?}", other),
    }
}

// ── publish_single_debug_batch ────────────────────────────────────────────────

#[test]
fn publish_batch_empty_no_output() {
    drain_tx_channels();
    init(PLATFORM_ID);
    embassy_futures::block_on(publish_single_debug_batch(&[], &[], 0, PLATFORM_ID, TS));
    // Nothing should have been sent.
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
    assert!(MQTT_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn publish_batch_sends_to_ble_only() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let cap = std_capture(0, 0x100, &[0x01], TS);
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[], 0, PLATFORM_ID, TS));
    assert!(BLE_TX_CHANNEL.try_receive().is_ok(), "expected BLE_TX message");
    assert!(MQTT_TX_CHANNEL.try_receive().is_err(), "MQTT_TX should be empty");
}

#[test]
fn publish_batch_frame_fields_correct() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let cap = std_capture(2, 0x3FF, &[0xAA, 0xBB], 12345);
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 1);
    let frame = &update.frames[0];
    assert_eq!(frame.can_id, 0x3FF);
    assert!(!frame.is_extended_id);
    assert_eq!(frame.bus_id, 2);
    assert_eq!(frame.timestamp_ms, 12345);
    assert_eq!(frame.data, vec![0xAA, 0xBB]);
}

#[test]
fn publish_batch_data_truncated_to_dlc() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // dlc=3 — only first 3 bytes should appear in the proto
    let cap = std_capture(0, 0x100, &[0x11, 0x22, 0x33], TS);
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames[0].data.len(), 3);
    assert_eq!(update.frames[0].data, vec![0x11, 0x22, 0x33]);
}

#[test]
fn publish_batch_blocklist_excludes_matched_frame() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let cap = std_capture(0, 0x100, &[0xFF], TS);
    let filter = std_filter(0x100, 0x7FF); // exact match
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[filter], 0, PLATFORM_ID, TS));
    // Frame matched the blocklist — nothing to send, dropped=0, so BLE_TX stays empty.
    assert!(BLE_TX_CHANNEL.try_receive().is_err());
}

#[test]
fn publish_batch_blocklist_mask_excludes_range() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // Filter: top 7 bits of 11-bit ID must be 0x100 → blocks 0x100..0x10F
    let filter = std_filter(0x100, 0x7F0);
    let frames = vec![
        std_capture(0, 0x100, &[1], TS),
        std_capture(0, 0x10F, &[2], TS),
        std_capture(0, 0x200, &[3], TS), // should pass — different top bits
    ];
    embassy_futures::block_on(publish_single_debug_batch(&frames, &[filter], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 1, "only 0x200 should pass");
    assert_eq!(update.frames[0].can_id, 0x200);
}

#[test]
fn publish_batch_non_matching_filter_passes_frame() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let cap = std_capture(0, 0x100, &[0xAB], TS);
    let filter = std_filter(0x200, 0x7FF); // targets 0x200, not 0x100
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[filter], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 1);
    assert_eq!(update.frames[0].can_id, 0x100);
}

#[test]
fn publish_batch_dropped_included_even_with_no_frames() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // No captures, but 3 dropped — batch should still be sent.
    embassy_futures::block_on(publish_single_debug_batch(&[], &[], 3, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 0);
    assert_eq!(update.dropped_frames, 3);
}

#[test]
fn publish_batch_extended_id_frame_fields_correct() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let cap = ext_capture(1, 0x1234_5678, &[0xDE, 0xAD]);
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 1);
    let frame = &update.frames[0];
    assert_eq!(frame.can_id, 0x1234_5678);
    assert!(frame.is_extended_id);
    assert_eq!(frame.data, vec![0xDE, 0xAD]);
}

#[test]
fn publish_batch_std_filter_does_not_block_extended_id_frame() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // Standard-ID filter targeting 0x100 must NOT block an extended-ID frame
    // with the same numeric ID — the id types are different.
    let cap = ext_capture(0, 0x100, &[0x01]);
    let filter = std_filter(0x100, 0x7FF); // is_extended_id: false
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[filter], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 1, "extended-ID frame must not be blocked by a standard-ID filter");
}

#[test]
fn publish_batch_extended_filter_does_not_block_standard_id_frame() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // Extended-ID filter must NOT block a standard-ID frame with the same numeric ID.
    let cap = std_capture(0, 0x100, &[0x01], TS);
    let filter = CanDebugFilter { can_id: 0x100, is_extended_id: true, mask: 0x1FFFFFFF };
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[filter], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 1, "standard-ID frame must not be blocked by an extended-ID filter");
}

#[test]
fn publish_batch_platform_id_in_output() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let cap = std_capture(0, 0x100, &[0x01], TS);
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[], 0, PLATFORM_ID, TS));
    let msg = BLE_TX_CHANNEL.try_receive().expect("expected DeviceToApp on BLE_TX");
    assert_eq!(msg.platform_id, PLATFORM_ID);
}

#[test]
fn publish_batch_zero_dlc_frame_produces_empty_data() {
    drain_tx_channels();
    init(PLATFORM_ID);
    let cap = std_capture(0, 0x100, &[], TS); // dlc = 0
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[], 0, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames[0].data.len(), 0, "zero-DLC frame must produce empty data field");
}

#[test]
fn publish_batch_all_filtered_with_drops_still_sends() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // Frame is blocked by filter AND there are dropped frames — batch must still be sent.
    let cap = std_capture(0, 0x100, &[0xFF], TS);
    let filter = std_filter(0x100, 0x7FF);
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &[filter], 5, PLATFORM_ID, TS));
    let update = recv_debug_update();
    assert_eq!(update.frames.len(), 0, "filtered frame should not appear");
    assert_eq!(update.dropped_frames, 5, "dropped count must be included");
}

#[test]
fn publish_batch_second_blocklist_entry_excludes_frame() {
    drain_tx_channels();
    init(PLATFORM_ID);
    // Two blocklist entries; frame matches the second one and must be excluded.
    let cap = std_capture(0, 0x300, &[0x01], TS);
    let filters = [
        std_filter(0x100, 0x7FF), // doesn't match 0x300
        std_filter(0x300, 0x7FF), // matches — should block
    ];
    embassy_futures::block_on(publish_single_debug_batch(&[cap], &filters, 0, PLATFORM_ID, TS));
    assert!(BLE_TX_CHANNEL.try_receive().is_err(), "frame matching second filter must be excluded");
}
