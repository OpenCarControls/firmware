//! What differs between modem models: power-on timings for now. A new model is a new
//! variant here (and its name in xtask's `[hardware.esp.modem].kind`).

/// A supported cellular modem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModemKind {
    /// SIMCom SIM7670G (LTE Cat-1 bis, GNSS), e.g. on the LilyGO T-SIM7670G-S3.
    Sim7670g,
}

/// Power-on and boot timings, from the module's hardware design notes and the board
/// vendor's examples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModemTimings {
    /// How long PWRKEY is held active to switch the modem on. Holding it much longer
    /// switches a running modem off.
    pub pwrkey_on_ms: u32,
    /// How long RESET is held active to restart a hung modem.
    pub reset_ms: u32,
    /// How long the modem may take to answer `AT` after power-on.
    pub boot_timeout_ms: u32,
    /// How long to wait for network registration before starting over.
    pub registration_timeout_ms: u32,
    /// How long the dial (`ATD*99#`) may take to answer `CONNECT`.
    pub dial_timeout_ms: u32,
}

impl ModemKind {
    pub const fn timings(self) -> ModemTimings {
        match self {
            ModemKind::Sim7670g => ModemTimings {
                pwrkey_on_ms: 100,
                reset_ms: 2600,
                boot_timeout_ms: 15_000,
                registration_timeout_ms: 180_000,
                dial_timeout_ms: 30_000,
            },
        }
    }
}
