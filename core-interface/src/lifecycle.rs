use core::sync::atomic::{AtomicU32, Ordering};

static PAIRING_WINDOW_UNTIL_S: AtomicU32 = AtomicU32::new(0);

pub fn open_pairing_window_for(seconds: u32) {
    if seconds == 0 {
        return;
    }
    let now_s = embassy_time::Instant::now().as_secs();
    let deadline = now_s.saturating_add(seconds as u64);
    let deadline_u32 = if deadline > u32::MAX as u64 {
        u32::MAX
    } else {
        deadline as u32
    };
    PAIRING_WINDOW_UNTIL_S.store(deadline_u32, Ordering::Relaxed);
}

pub fn close_pairing_window() {
    PAIRING_WINDOW_UNTIL_S.store(0, Ordering::Relaxed);
}

pub fn is_pairing_window_open() -> bool {
    let now_s = embassy_time::Instant::now().as_secs();
    now_s < PAIRING_WINDOW_UNTIL_S.load(Ordering::Relaxed) as u64
}
