use core_interface::{is_can_read_only, set_can_read_only};

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Resets the flag to the boot default (true) so each test starts from a known
/// state regardless of what a prior test left behind. With --test-threads=1 the
/// tests run sequentially in the same process, sharing the global AtomicBool.
fn reset() {
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
