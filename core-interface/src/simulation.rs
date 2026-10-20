use portable_atomic::{AtomicBool, Ordering};

static SIMULATION_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn is_simulation_enabled() -> bool {
    SIMULATION_ENABLED.load(Ordering::Relaxed)
}

pub fn set_simulation_enabled(enabled: bool) {
    SIMULATION_ENABLED.store(enabled, Ordering::Relaxed);
}
