use core_interface::{DEDUPE_CAPACITY, DEDUPE_WINDOW_MS, RecentCommands};

const PHONE_A: &[u8] = b"ble:\x01\x02\x03\x04\x05\x06";
const PHONE_B: &[u8] = b"ble:\x0A\x0B\x0C\x0D\x0E\x0F";

#[test]
fn a_repeated_pair_is_a_duplicate() {
    let mut recent = RecentCommands::new();
    assert!(!recent.is_duplicate(PHONE_A, 7, 1_000));
    assert!(recent.is_duplicate(PHONE_A, 7, 2_000));
    assert!(recent.is_duplicate(PHONE_A, 7, 3_000));
}

#[test]
fn the_same_id_from_another_source_is_not() {
    let mut recent = RecentCommands::new();
    assert!(!recent.is_duplicate(PHONE_A, 7, 1_000));
    assert!(!recent.is_duplicate(PHONE_B, 7, 1_000));
    assert!(
        !recent.is_duplicate(&[], 7, 1_000),
        "MQTT has its own source"
    );
}

#[test]
fn a_command_is_forgotten_after_the_window() {
    let mut recent = RecentCommands::new();
    assert!(!recent.is_duplicate(PHONE_A, 7, 1_000));
    assert!(recent.is_duplicate(PHONE_A, 7, 1_000 + DEDUPE_WINDOW_MS - 1));
    assert!(!recent.is_duplicate(PHONE_A, 7, 1_000 + DEDUPE_WINDOW_MS));
}

#[test]
fn the_oldest_command_is_forgotten_first_when_full() {
    let mut recent = RecentCommands::new();
    for id in 1..=DEDUPE_CAPACITY as u64 {
        assert!(!recent.is_duplicate(PHONE_A, id, 1_000));
    }
    assert!(!recent.is_duplicate(PHONE_A, 100, 1_000));
    assert!(!recent.is_duplicate(PHONE_A, 1, 1_000), "pushed out by 100");
    assert!(recent.is_duplicate(PHONE_A, DEDUPE_CAPACITY as u64, 1_000));
}

#[test]
fn message_id_zero_is_never_a_duplicate() {
    let mut recent = RecentCommands::new();
    assert!(!recent.is_duplicate(PHONE_A, 0, 1_000));
    assert!(!recent.is_duplicate(PHONE_A, 0, 1_000));
}
