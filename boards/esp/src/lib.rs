#![cfg_attr(not(test), no_std)]

use core_interface::CanFilter;
#[cfg(feature = "hardware")]
use core_interface::{CanFrame, CanRawCapture, CAN_DEBUG_RX_CHANNEL, CAN_RX_CHANNEL, CAN_TX_CHANNEL};
#[cfg(feature = "hardware")]
use embedded_can::{Id, StandardId};

#[cfg(feature = "hardware")]
use embedded_can::Frame as EmbeddedFrame;
#[cfg(feature = "hardware")]
use embedded_hal_bus::spi::ExclusiveDevice;
#[cfg(feature = "hardware")]
use esp_hal::{
    delay::Delay,
    gpio::{Input, Output},
    gpio::interconnect::{PeripheralInput, PeripheralOutput},
    spi::master::Spi,
    twai::{self, Twai, TwaiMode},
};
#[cfg(feature = "hardware")]
pub use mcp2515::{CanSpeed, McpSpeed};
#[cfg(feature = "hardware")]
use mcp2515::{
    error::Error as McpError,
    filter::{RxFilter, RxMask},
    frame::CanFrame as McpFrame,
    regs::OpMode,
    MCP2515,
};

// ── Type Aliases ─────────────────────────────────────────────────────────────

#[cfg(feature = "hardware")]
/// Concrete TWAI driver type (Blocking mode → implements Send, safe for core 1).
pub type TwaiDriver = Twai<'static, esp_hal::Blocking>;

#[cfg(feature = "hardware")]
/// Concrete MCP2515 driver type (blocking SPI, INT-pin driven).
pub type Mcp2515Driver = MCP2515<ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>>;

#[cfg(feature = "hardware")]
/// The INT input pin type used for MCP2515 interrupt-driven RX.
pub type CanIntPin = Input<'static>;

// ── Board Entry Point ─────────────────────────────────────────────────────────

#[cfg(feature = "hardware")]
pub fn start(spawner: &embassy_executor::Spawner) {
    spawner.spawn(core_interface::process_ble_commands_task()).unwrap();
    spawner.spawn(core_interface::process_mqtt_commands_task()).unwrap();
    spawner.spawn(core_interface::route_responses_task()).unwrap();
    spawner.spawn(core_interface::publish_state_task()).unwrap();
    spawner.spawn(core_interface::publish_can_debug_task()).unwrap();
}

// ── CAN ID helpers ────────────────────────────────────────────────────────────

// ── TWAI (built-in CAN) ───────────────────────────────────────────────────────
#[cfg(feature = "hardware")]/// Initialises the TWAI peripheral with an accept-all hardware filter; actual
/// frame selection is done in software inside `run_twai_loop`.
pub fn init_twai(
    peripheral: impl twai::Instance + 'static,
    rx_pin: impl PeripheralInput<'static>,
    tx_pin: impl PeripheralOutput<'static>,
    _filters: &[CanFilter],
) -> TwaiDriver {
    // Mask = 0 in esp-hal TWAI means "don't care" for every bit → accept all.
    let accept_all = twai::filter::SingleStandardFilter::new_from_code_mask(
        unsafe { twai::StandardId::new_unchecked(0) },
        unsafe { twai::StandardId::new_unchecked(0) },
        false, false,
        [0, 0], [0, 0],
    );
    let mut cfg = twai::TwaiConfiguration::new(
        peripheral,
        rx_pin,
        tx_pin,
        esp_hal::twai::BaudRate::B500K,
        TwaiMode::Normal,
    );
    cfg.set_filter(accept_all);
    cfg.start()
}

#[cfg(feature = "hardware")]
/// Bidirectional TWAI loop — runs forever on core 1.
///
/// Uses non-blocking nb polling on RX (1 ms yield when no frame is available)
/// and drains the TX channel between polls.
pub async fn run_twai_loop(driver: TwaiDriver, bus_id: u8, filters: &'static [CanFilter]) {
    use embassy_time::{Duration, Timer};

    let (mut rx, mut tx) = driver.split();
    loop {
        // Drain all available received frames
        loop {
            match rx.receive() {
                Ok(frame) => {
                    let core_frame = twai_to_core_frame(frame, bus_id);
                    if core_interface::can_debug_wants_bus(bus_id) {
                        let cap = CanRawCapture {
                            timestamp_ms: embassy_time::Instant::now().as_millis(),
                            bus_id,
                            id: core_frame.id,
                            data: core_frame.data,
                            dlc: core_frame.dlc,
                        };
                        if CAN_DEBUG_RX_CHANNEL.sender().try_send(cap).is_err() {
                            core_interface::increment_can_debug_dropped();
                        }
                    }
                    if core_interface::passes_filter(&core_frame, filters) {
                        CAN_RX_CHANNEL.sender().send(core_frame).await;
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(_) => break, // bus error; keep going
            }
        }

        // Drain outbound TX channel for this bus
        while let Ok(outbound) = CAN_TX_CHANNEL.receiver().try_receive() {
            if outbound.bus_id == bus_id {
                // Drop silently when in read-only mode; do not transmit on the bus.
                if !core_interface::is_can_read_only() {
                    if let Some(f) = core_to_twai_frame(&outbound) {
                        // nb::block! would stall the executor; spin until TX buffer free
                        loop {
                            match tx.transmit(&f) {
                                Ok(_) | Err(nb::Error::Other(_)) => break,
                                Err(nb::Error::WouldBlock) => Timer::after(Duration::from_micros(100)).await,
                            }
                        }
                    }
                }
            } else {
                let _ = CAN_TX_CHANNEL.sender().try_send(outbound);
            }
        }

        // Yield 1 ms before next poll
        Timer::after(Duration::from_millis(1)).await;
    }
}

#[cfg(feature = "hardware")]
fn twai_to_core_frame(frame: twai::EspTwaiFrame, bus_id: u8) -> CanFrame {
    let id = EmbeddedFrame::id(&frame);
    let dlc = EmbeddedFrame::dlc(&frame) as u8;
    let raw = EmbeddedFrame::data(&frame);
    let mut data = [0u8; 8];
    data[..raw.len()].copy_from_slice(raw);
    CanFrame { bus_id, id, data, dlc }
}

#[cfg(feature = "hardware")]
fn core_to_twai_frame(frame: &CanFrame) -> Option<twai::EspTwaiFrame> {
    let data = &frame.data[..frame.dlc as usize];
    // esp_hal::twai::Id implements From<embedded_can::Id>
    twai::EspTwaiFrame::new(frame.id, data)
}

// ── MCP2515 (SPI CAN) ─────────────────────────────────────────────────────────

/// Computes the two combined RX buffer masks required by the MCP2515 from a
/// `CanFilter` slice for a specific `bus_id`.
///
/// - `mask0` covers RXB0 filters (slots 0–1, up to 2 filters).
/// - `mask1` covers RXB1 filters (slots 2–5, up to 4 more filters).
///
/// Each mask is OR-folded from the individual filter masks so that a hardware
/// RXB mask bit is 1 (must match) only when every filter in that buffer agrees
/// the bit must match.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) fn compute_mcp_masks(filters: &[CanFilter], bus_id: u8) -> (u32, u32) {
    let mut it = filters.iter().filter(|f| f.bus_id == bus_id);
    let mask0 = it.by_ref().take(2).fold(0u32, |acc, f| acc | f.mask);
    let mask1 = it.take(4).fold(0u32, |acc, f| acc | f.mask);
    (mask0, mask1)
}

#[cfg(feature = "hardware")]
const RX_FILTERS: [RxFilter; 6] = [
    RxFilter::F0, RxFilter::F1, RxFilter::F2,
    RxFilter::F3, RxFilter::F4, RxFilter::F5,
];

#[cfg(feature = "hardware")]
/// Initialises an MCP2515 via blocking SPI. Programs hardware RX filters/masks.
/// Returns `(driver, int_pin)`. Pass both to `run_mcp2515_loop`.
pub fn init_mcp2515(
    spi_peri: impl esp_hal::spi::master::Instance + 'static,
    sck_pin: impl PeripheralOutput<'static>,
    mosi_pin: impl PeripheralOutput<'static>,
    miso_pin: impl PeripheralInput<'static>,
    cs_pin: Output<'static>,
    filters: &[CanFilter],
    bus_id: u8,
    can_speed: CanSpeed,
    mcp_speed: McpSpeed,
    int_pin: CanIntPin,
) -> (Mcp2515Driver, CanIntPin) {
    let spi_bus = Spi::new(spi_peri, esp_hal::spi::master::Config::default())
        .expect("SPI init failed")
        .with_sck(sck_pin)
        .with_mosi(mosi_pin)
        .with_miso(miso_pin);
    let spi_dev = ExclusiveDevice::new(spi_bus, cs_pin, Delay::new())
        .expect("SPI device init failed");

    let mut mcp = MCP2515::new(spi_dev);
    mcp.init(&mut Delay::new(), mcp2515::Settings { mode: OpMode::Normal, can_speed, mcp_speed, clkout_en: false }).unwrap();

    // Collect up to 6 filter IDs for this bus on the stack.
    let mut ids: [Option<Id>; 6] = [None; 6];
    let mut count = 0usize;
    for f in filters.iter().filter(|f| f.bus_id == bus_id).take(6) {
        ids[count] = Some(f.id);
        count += 1;
    }

    // Combined mask for RXB0 (slots 0..2) and RXB1 (slots 2..6).
    let (mask0, mask1) = compute_mcp_masks(filters, bus_id);

    if let Some(sid) = StandardId::new((mask0 & 0x7FF) as u16) {
        mcp.set_mask(RxMask::Mask0, Id::Standard(sid)).ok();
    }
    if let Some(sid) = StandardId::new((mask1 & 0x7FF) as u16) {
        mcp.set_mask(RxMask::Mask1, Id::Standard(sid)).ok();
    }
    for i in 0..count {
        if let Some(id) = ids[i] {
            mcp.set_filter(RX_FILTERS[i], id).ok();
        }
    }

    (mcp, int_pin)
}

#[cfg(feature = "hardware")]
/// Bidirectional MCP2515 loop — runs forever on core 1.
pub async fn run_mcp2515_loop(
    mut driver: Mcp2515Driver,
    mut int_pin: CanIntPin,
    bus_id: u8,
    filters: &'static [CanFilter],
) {
    let mut was_debug_active = false;
    loop {
        // ── MCP2515 hardware mask switching on debug enable/disable ───────
        let debug_now = core_interface::is_can_debug_active();
        if debug_now && !was_debug_active {
            // Accept all frames in hardware so the debug tap sees raw traffic.
            driver.set_mask(RxMask::Mask0, Id::Standard(unsafe { StandardId::new_unchecked(0) })).ok();
            driver.set_mask(RxMask::Mask1, Id::Standard(unsafe { StandardId::new_unchecked(0) })).ok();
        } else if !debug_now && was_debug_active {
            // Restore vehicle-specific hardware masks.
            let (mask0, mask1) = compute_mcp_masks(filters, bus_id);
            if let Some(sid) = StandardId::new((mask0 & 0x7FF) as u16) {
                driver.set_mask(RxMask::Mask0, Id::Standard(sid)).ok();
            }
            if let Some(sid) = StandardId::new((mask1 & 0x7FF) as u16) {
                driver.set_mask(RxMask::Mask1, Id::Standard(sid)).ok();
            }
        }
        was_debug_active = debug_now;

        int_pin.wait_for_falling_edge().await;

        loop {
            match driver.read_message() {
                Ok(frame) => {
                    let core_frame = mcp_to_core_frame(frame, bus_id);
                    if core_interface::can_debug_wants_bus(bus_id) {
                        let cap = CanRawCapture {
                            timestamp_ms: embassy_time::Instant::now().as_millis(),
                            bus_id,
                            id: core_frame.id,
                            data: core_frame.data,
                            dlc: core_frame.dlc,
                        };
                        if CAN_DEBUG_RX_CHANNEL.sender().try_send(cap).is_err() {
                            core_interface::increment_can_debug_dropped();
                        }
                    }
                    if core_interface::passes_filter(&core_frame, filters) {
                        CAN_RX_CHANNEL.sender().send(core_frame).await;
                    }
                }
                Err(McpError::NoMessage) | Err(_) => break,
            }
        }

        while let Ok(outbound) = CAN_TX_CHANNEL.receiver().try_receive() {
            if outbound.bus_id == bus_id {
                // Drop silently when in read-only mode; do not transmit on the bus.
                if !core_interface::is_can_read_only() {
                    if let Some(f) = core_to_mcp_frame(&outbound) {
                        let _ = driver.send_message(f);
                    }
                }
            } else {
                let _ = CAN_TX_CHANNEL.sender().try_send(outbound);
            }
        }
    }
}

#[cfg(feature = "hardware")]
fn mcp_to_core_frame(frame: McpFrame, bus_id: u8) -> CanFrame {
    let id = EmbeddedFrame::id(&frame);
    let dlc = EmbeddedFrame::dlc(&frame) as u8;
    let raw = EmbeddedFrame::data(&frame);
    let mut data = [0u8; 8];
    data[..raw.len()].copy_from_slice(raw);
    CanFrame { bus_id, id, data, dlc }
}

#[cfg(feature = "hardware")]
fn core_to_mcp_frame(frame: &CanFrame) -> Option<McpFrame> {
    McpFrame::new(frame.id, &frame.data[..frame.dlc as usize])
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{Id, StandardId};

    fn filter(bus_id: u8, raw_id: u16, mask: u32) -> CanFilter {
        CanFilter {
            bus_id,
            id: Id::Standard(StandardId::new(raw_id).unwrap()),
            mask,
        }
    }

    // ── compute_mcp_masks ─────────────────────────────────────────────────────

    #[test]
    fn empty_filters_produces_zero_masks() {
        let (m0, m1) = compute_mcp_masks(&[], 0);
        assert_eq!(m0, 0);
        assert_eq!(m1, 0);
    }

    #[test]
    fn single_filter_sets_mask0_only() {
        let filters = [filter(0, 0x100, 0x7FF)];
        let (m0, m1) = compute_mcp_masks(&filters, 0);
        assert_eq!(m0, 0x7FF);
        assert_eq!(m1, 0);
    }

    #[test]
    fn two_filters_ored_into_mask0() {
        // 0x700 | 0x0FF = 0x7FF
        let filters = [filter(0, 0x100, 0x700), filter(0, 0x200, 0x0FF)];
        let (m0, m1) = compute_mcp_masks(&filters, 0);
        assert_eq!(m0, 0x7FF);
        assert_eq!(m1, 0);
    }

    #[test]
    fn third_filter_goes_into_mask1() {
        let filters = [
            filter(0, 0x100, 0x7FF),
            filter(0, 0x200, 0x7FF),
            filter(0, 0x300, 0x70F),
        ];
        let (m0, m1) = compute_mcp_masks(&filters, 0);
        assert_eq!(m0, 0x7FF); // OR of first two
        assert_eq!(m1, 0x70F); // third filter alone
    }

    #[test]
    fn filters_for_other_bus_excluded() {
        let filters = [
            filter(0, 0x100, 0x7FF),
            filter(1, 0x200, 0x7FF), // bus 1 — must be ignored
        ];
        let (m0, m1) = compute_mcp_masks(&filters, 0);
        assert_eq!(m0, 0x7FF); // only bus 0 filter
        assert_eq!(m1, 0);
        let (m0, m1) = compute_mcp_masks(&filters, 1);
        assert_eq!(m0, 0x7FF); // only bus 1 filter
        assert_eq!(m1, 0);
    }

    #[test]
    fn six_filters_fill_both_mask_buffers() {
        // mask0 covers slots 0..1, mask1 covers slots 2..5
        let filters: Vec<CanFilter> = (0u16..6)
            .map(|i| filter(0, 0x100 + i * 0x10, if i < 2 { 0x700 } else { 0x0F0 }))
            .collect();
        let (m0, m1) = compute_mcp_masks(&filters, 0);
        assert_eq!(m0, 0x700); // OR of first two (identical masks)
        assert_eq!(m1, 0x0F0); // OR of slots 2..5 (identical masks)
    }
}
