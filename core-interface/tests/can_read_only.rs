use core_interface::{is_can_read_only, set_can_read_only};

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Resets the flag to the boot default (true) so each test starts from a known
/// state regardless of what a prior test left behind. With --test-threads=1 the
/// tests run sequentially in the same process, sharing the global AtomicBool.
/// The car identity is confirmed so tests can unlock; the identity tests revoke it.
fn reset() {
    core_interface::confirm_car_identity();
    set_can_read_only(true);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

/// The factory default must be read-only (safe / no TX at boot).
#[test]
fn read_only_is_true_by_default() {
    reset();
    assert!(is_can_read_only());
}

/// After the vehicle crate validates the connected car it calls
/// set_can_read_only(false) to allow CAN TX.
#[test]
fn set_false_disables_read_only() {
    reset();
    set_can_read_only(false);
    assert!(!is_can_read_only());
    reset();
}

/// If the vehicle crate detects an error or inconsistent data, it can
/// re-engage the lock by calling set_can_read_only(true).
#[test]
fn set_true_re_engages_read_only() {
    reset();
    set_can_read_only(false);
    set_can_read_only(true);
    assert!(is_can_read_only());
}

/// Multiple transitions must always reflect the last written value.
#[test]
fn toggling_multiple_times_reflects_last_write() {
    reset();
    set_can_read_only(false);
    set_can_read_only(true);
    set_can_read_only(false);
    assert!(!is_can_read_only());
    reset();
}

/// Nothing can unlock the bus until the vehicle has confirmed the car's identity.
#[test]
fn unlock_is_refused_until_identity_is_confirmed() {
    core_interface::revoke_car_identity();
    assert!(!core_interface::is_car_identity_confirmed());
    assert!(set_can_read_only(false), "unlock must be refused");
    assert!(is_can_read_only());

    core_interface::confirm_car_identity();
    assert!(!set_can_read_only(false));
    assert!(!is_can_read_only());
    reset();
}

/// Revoking the identity re-locks a bus that was already unlocked.
#[test]
fn revoking_identity_relocks_the_bus() {
    reset();
    set_can_read_only(false);
    assert!(!is_can_read_only());
    core_interface::revoke_car_identity();
    assert!(is_can_read_only());
    assert!(set_can_read_only(false));
    reset();
}

#[test]
fn vin_must_be_well_formed_and_match_a_prefix() {
    use core_interface::vin_matches;
    let vin = b"KNAC381AAP5000001";
    assert!(vin_matches(vin, &["KNA", "KMH"]));
    assert!(vin_matches(vin, &["KNAC38"]));
    assert!(!vin_matches(vin, &["KMH", "5NM"]));
    assert!(!vin_matches(vin, &[]));
    assert!(!vin_matches(vin, &[""]));
    // Wrong length, lowercase, and the letters I, O, Q never appear in a VIN.
    assert!(!vin_matches(b"KNAC381AAP500000", &["KNA"]));
    assert!(!vin_matches(b"knac381aap5000001", &["kna"]));
    assert!(!vin_matches(b"KNAC381AAP500000I", &["KNA"]));
    assert!(!vin_matches(b"KNAC381AAP500000O", &["KNA"]));
    assert!(!vin_matches(&[0u8; 17], &["KNA"]));
}
