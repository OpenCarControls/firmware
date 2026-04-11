use core_interface::{CanFilter, CanFrame, passes_filter};
use embedded_can::{ExtendedId, Id, StandardId};

fn std_frame(bus_id: u8, raw_id: u16, dlc: u8) -> CanFrame {
    CanFrame {
        bus_id,
        id: Id::Standard(StandardId::new(raw_id).unwrap()),
        data: [0u8; 8],
        dlc,
    }
}

fn std_filter(bus_id: u8, raw_id: u16, mask: u32) -> CanFilter {
    CanFilter {
        bus_id,
        id: Id::Standard(StandardId::new(raw_id).unwrap()),
        mask,
    }
}

// ── Standard ID ───────────────────────────────────────────────────────────────

#[test]
fn exact_std_id_match_passes() {
    let frame = std_frame(0, 0x100, 2);
    let filters = [std_filter(0, 0x100, 0x7FF)];
    assert!(passes_filter(&frame, &filters));
}

#[test]
fn wrong_std_id_rejected() {
    let frame = std_frame(0, 0x101, 2);
    let filters = [std_filter(0, 0x100, 0x7FF)];
    assert!(!passes_filter(&frame, &filters));
}

#[test]
fn mask_zero_accepts_all_ids_on_matching_bus() {
    let frame = std_frame(0, 0x7FF, 0);
    let filters = [std_filter(0, 0x000, 0x000)];
    assert!(passes_filter(&frame, &filters));
}

#[test]
fn wrong_bus_id_rejected_even_if_id_matches() {
    let frame = std_frame(1, 0x100, 2);
    let filters = [std_filter(0, 0x100, 0x7FF)];
    assert!(!passes_filter(&frame, &filters));
}

#[test]
fn frame_matches_second_filter_in_list() {
    let frame = std_frame(0, 0x200, 1);
    let filters = [std_filter(0, 0x100, 0x7FF), std_filter(0, 0x200, 0x7FF)];
    assert!(passes_filter(&frame, &filters));
}

#[test]
fn empty_filter_list_rejects_everything() {
    let frame = std_frame(0, 0x100, 2);
    assert!(!passes_filter(&frame, &[]));
}

// ── Extended ID ───────────────────────────────────────────────────────────────

#[test]
fn exact_extended_id_match_passes() {
    let id = ExtendedId::new(0x1234_5678).unwrap();
    let frame = CanFrame { bus_id: 0, id: Id::Extended(id), data: [0u8; 8], dlc: 0 };
    let filter = CanFilter { bus_id: 0, id: Id::Extended(id), mask: u32::MAX };
    assert!(passes_filter(&frame, &[filter]));
}

#[test]
fn wrong_extended_id_rejected() {
    let frame_id = ExtendedId::new(0x1234_5678).unwrap();
    let filter_id = ExtendedId::new(0x1234_5679).unwrap();
    let frame = CanFrame { bus_id: 0, id: Id::Extended(frame_id), data: [0u8; 8], dlc: 0 };
    let filter = CanFilter { bus_id: 0, id: Id::Extended(filter_id), mask: u32::MAX };
    assert!(!passes_filter(&frame, &[filter]));
}

// ── Partial-mask matching ─────────────────────────────────────────────────────

#[test]
fn partial_mask_ignores_low_bits() {
    // Mask 0x7F0 checks only the top 7 of 11 bits: 0x100 and 0x10F both pass.
    let filters = [std_filter(0, 0x100, 0x7F0)];
    assert!(passes_filter(&std_frame(0, 0x100, 0), &filters));
    assert!(passes_filter(&std_frame(0, 0x10F, 0), &filters));
    assert!(!passes_filter(&std_frame(0, 0x200, 0), &filters));
}
