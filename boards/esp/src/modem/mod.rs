//! Cellular (LTE) uplink: a modem on a UART, brought online with AT commands and then
//! run as a PPP link under `embassy-net`, so everything above the network stack (MQTT,
//! DNS) works as it does over WiFi. See docs/lte.md.
//!
//! The AT parsing and the serial channel are pure and host-tested; the power-on
//! sequence, the bring-up and the PPP session live in `hardware`.

pub mod at;
pub mod channel;
mod kind;

pub use kind::ModemKind;

#[cfg(feature = "hardware")]
mod hardware;
#[cfg(feature = "hardware")]
pub use hardware::*;

#[cfg(test)]
mod tests;
