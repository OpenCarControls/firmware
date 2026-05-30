//! CAN bus drivers for the ESP32 board.
//!
//! Two driver implementations are provided:
//! - **TWAI** – the ESP32's built-in CAN peripheral, polled with `nb` non-blocking reads.
//! - **MCP2515** – external SPI CAN controller, interrupt-driven via a falling-edge INT pin.
//!
//! Both share the same channel contract: `CAN_RX_CHANNEL` (board → vehicle) and
//! `CAN_TX_CHANNEL` (vehicle → board). All frame filtering against the vehicle's
//! `CanFilter` list is done in software after the hardware delivers the frame.

use core_interface::CanFilter;

#[cfg(feature = "hardware")]
use core_interface::{
    CAN_DEBUG_RX_CHANNEL, CAN_RX_CHANNEL, CAN_TX_CHANNEL, CanFrame, CanRawCapture,
};
#[cfg(feature = "hardware")]
use embedded_can::Frame as EmbeddedFrame;
#[cfg(feature = "hardware")]
use embedded_can::{Id, StandardId};
#[cfg(feature = "hardware")]
use embedded_hal_bus::spi::ExclusiveDevice;
#[cfg(feature = "hardware")]
use esp_hal::{
    delay::Delay,
    gpio::interconnect::{PeripheralInput, PeripheralOutput},
    gpio::{Input, Output},
    spi::master::Spi,
    twai::{self, Twai, TwaiMode},
};
#[cfg(feature = "hardware")]
pub use mcp2515::{CanSpeed, McpSpeed};
#[cfg(feature = "hardware")]
use mcp2515::{
    MCP2515,
    filter::{RxFilter, RxMask},
    frame::CanFrame as McpFrame,
    regs::OpMode,
};

#[cfg(feature = "hardware")]
/// Concrete TWAI driver type (Blocking mode -> implements Send, safe for core 1).
pub type TwaiDriver = Twai<'static, esp_hal::Blocking>;

#[cfg(feature = "hardware")]
/// Concrete MCP2515 driver type (blocking SPI, INT-pin driven).
pub type Mcp2515Driver =
    MCP2515<ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>>;

#[cfg(feature = "hardware")]
/// The INT input pin type used for MCP2515 interrupt-driven RX.
pub type CanIntPin = Input<'static>;

/// Computes the two combined RX buffer masks required by the MCP2515 from a
/// `CanFilter` slice for a specific `bus_id`.
///
/// - `mask0` covers RXB0 filters (slots 0-1, up to 2 filters).
/// - `mask1` covers RXB1 filters (slots 2-5, up to 4 more filters).
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
    RxFilter::F0,
    RxFilter::F1,
    RxFilter::F2,
    RxFilter::F3,
    RxFilter::F4,
    RxFilter::F5,
];

#[cfg(feature = "hardware")]
/// Initialises the TWAI peripheral with an accept-all hardware filter; actual
/// frame selection is done in software inside `run_twai_loop`.
pub fn init_twai(
    peripheral: impl twai::Instance + 'static,
    rx_pin: impl PeripheralInput<'static>,
    tx_pin: impl PeripheralOutput<'static>,
    _filters: &[CanFilter],
) -> TwaiDriver {
    // Mask = 0 in esp-hal TWAI means "don't care" for every bit -> accept all.
    let accept_all = twai::filter::SingleStandardFilter::new_from_code_mask(
        unsafe { twai::StandardId::new_unchecked(0) },
        unsafe { twai::StandardId::new_unchecked(0) },
        false,
        false,
        [0, 0],
        [0, 0],
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
/// Bidirectional TWAI loop - runs forever on core 1.
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
                    let core_frame = to_core_frame(frame, bus_id);
                    try_send_can_debug_capture(bus_id, &core_frame);
                    if core_interface::passes_filter(&core_frame, filters) {
                        CAN_RX_CHANNEL.sender().send(core_frame).await;
                    }
                }
                Err(_) => break, // WouldBlock (no frame) or bus error; either way keep going
            }
        }

        // Drain outbound TX channel for this bus
        // Frames for other buses are put back on the shared channel so their
        // respective bus loops can pick them up. try_send is intentional — if the
        // channel is momentarily full the frame is dropped rather than stalling.
        while let Ok(outbound) = CAN_TX_CHANNEL.receiver().try_receive() {
            if outbound.bus_id == bus_id {
                // Drop silently when in read-only mode; do not transmit on the bus.
                if !core_interface::is_can_read_only() {
                    if let Some(f) = core_to_twai_frame(&outbound) {
                        // nb::block! would stall the executor; spin until TX buffer free
                        loop {
                            match tx.transmit(&f) {
                                Ok(_) | Err(nb::Error::Other(_)) => break,
                                Err(nb::Error::WouldBlock) => {
                                    Timer::after(Duration::from_micros(100)).await
                                }
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
fn core_to_twai_frame(frame: &CanFrame) -> Option<twai::EspTwaiFrame> {
    let data = &frame.data[..frame.dlc as usize];
    // esp_hal::twai::Id implements From<embedded_can::Id>
    twai::EspTwaiFrame::new(frame.id, data)
}

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
    let spi_dev =
        ExclusiveDevice::new(spi_bus, cs_pin, Delay::new()).expect("SPI device init failed");

    let mut mcp = MCP2515::new(spi_dev);
    mcp.init(
        &mut Delay::new(),
        mcp2515::Settings {
            mode: OpMode::Normal,
            can_speed,
            mcp_speed,
            clkout_en: false,
        },
    )
    .unwrap();

    // Collect up to 6 filter IDs for this bus on the stack.
    let mut ids: [Option<Id>; 6] = [None; 6];
    for (slot, f) in filters
        .iter()
        .filter(|f| f.bus_id == bus_id)
        .take(6)
        .enumerate()
    {
        ids[slot] = Some(f.id);
    }

    // Program hardware RX masks (reuse the same logic used during debug-mode transitions)
    // then program the individual filter slots.
    set_mcp_vehicle_masks(&mut mcp, filters, bus_id);
    for (slot, id) in ids.iter().enumerate() {
        if let Some(id) = id {
            mcp.set_filter(RX_FILTERS[slot], *id).ok();
        }
    }

    (mcp, int_pin)
}

#[cfg(feature = "hardware")]
/// Bidirectional MCP2515 loop — runs forever on core 1.
///
/// Waits for the INT pin to fall (MCP2515 asserts it on any RX or TX event), then
/// drains all available received frames and the outbound TX channel.
///
/// Hardware RX masks are swapped to accept-all when CAN debug mode is active so the
/// debug tap can observe every frame on the bus, and restored when debug is disabled.
pub async fn run_mcp2515_loop(
    mut driver: Mcp2515Driver,
    mut int_pin: CanIntPin,
    bus_id: u8,
    filters: &'static [CanFilter],
) {
    let mut was_debug_active = false;
    loop {
        // MCP2515 hardware mask switching on debug enable/disable.
        let debug_now = core_interface::is_can_debug_active();
        if debug_now && !was_debug_active {
            // Accept all frames in hardware so the debug tap sees raw traffic.
            set_mcp_accept_all_masks(&mut driver);
        } else if !debug_now && was_debug_active {
            // Restore vehicle-specific hardware masks.
            set_mcp_vehicle_masks(&mut driver, filters, bus_id);
        }
        was_debug_active = debug_now;

        int_pin.wait_for_falling_edge().await;

        loop {
            match driver.read_message() {
                Ok(frame) => {
                    let core_frame = to_core_frame(frame, bus_id);
                    try_send_can_debug_capture(bus_id, &core_frame);
                    if core_interface::passes_filter(&core_frame, filters) {
                        CAN_RX_CHANNEL.sender().send(core_frame).await;
                    }
                }
                Err(_) => break, // NoMessage (FIFO empty) or hardware error; either way done
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
fn core_to_mcp_frame(frame: &CanFrame) -> Option<McpFrame> {
    McpFrame::new(frame.id, &frame.data[..frame.dlc as usize])
}

/// Converts any [`EmbeddedFrame`]-implementing hardware frame into the
/// crate-shared [`CanFrame`] type. Used by both the TWAI and MCP2515 drivers.
#[cfg(feature = "hardware")]
fn to_core_frame<F: EmbeddedFrame>(frame: F, bus_id: u8) -> CanFrame {
    let id = frame.id();
    let dlc = frame.dlc() as u8;
    let raw = frame.data();
    let mut data = [0u8; 8];
    data[..raw.len()].copy_from_slice(raw);
    CanFrame {
        bus_id,
        id,
        data,
        dlc,
    }
}

/// Switches both MCP2515 RX buffer masks to accept-all (mask bits = 0).
/// Called when CAN debug mode is enabled so the debug tap can observe every
/// frame on the bus before the software filter runs.
#[cfg(feature = "hardware")]
fn set_mcp_accept_all_masks(driver: &mut Mcp2515Driver) {
    // StandardId 0 is always valid (fits in 11 bits); no unsafe needed.
    let id_zero = Id::Standard(StandardId::new(0).unwrap());
    driver.set_mask(RxMask::Mask0, id_zero).ok();
    driver.set_mask(RxMask::Mask1, id_zero).ok();
}

/// Restores the MCP2515 RX buffer masks derived from the vehicle's `CanFilter` list.
/// Called when CAN debug mode is disabled to return the hardware to selective
/// filtering and reduce CPU load from unwanted frames reaching the software path.
#[cfg(feature = "hardware")]
fn set_mcp_vehicle_masks(driver: &mut Mcp2515Driver, filters: &[CanFilter], bus_id: u8) {
    let (mask0, mask1) = compute_mcp_masks(filters, bus_id);
    if let Some(sid) = StandardId::new((mask0 & 0x7FF) as u16) {
        driver.set_mask(RxMask::Mask0, Id::Standard(sid)).ok();
    }
    if let Some(sid) = StandardId::new((mask1 & 0x7FF) as u16) {
        driver.set_mask(RxMask::Mask1, Id::Standard(sid)).ok();
    }
}

/// Forwards `frame` to the CAN debug channel if the debug tap is currently
/// watching `bus_id`. Uses `try_send` (non-blocking) so the CAN driver is never
/// stalled by a slow debug consumer; dropped frames are counted so the app can
/// report the loss.
#[cfg(feature = "hardware")]
fn try_send_can_debug_capture(bus_id: u8, frame: &CanFrame) {
    if !core_interface::can_debug_wants_bus(bus_id) {
        return;
    }

    let cap = CanRawCapture {
        timestamp_ms: embassy_time::Instant::now().as_millis(),
        bus_id,
        id: frame.id,
        data: frame.data,
        dlc: frame.dlc,
    };
    if CAN_DEBUG_RX_CHANNEL.sender().try_send(cap).is_err() {
        core_interface::increment_can_debug_dropped();
    }
}

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
        assert_eq!(m0, 0x7FF);
        assert_eq!(m1, 0x70F);
    }

    #[test]
    fn filters_for_other_bus_excluded() {
        let filters = [filter(0, 0x100, 0x7FF), filter(1, 0x200, 0x7FF)];
        let (m0, m1) = compute_mcp_masks(&filters, 0);
        assert_eq!(m0, 0x7FF);
        assert_eq!(m1, 0);

        let (m0, m1) = compute_mcp_masks(&filters, 1);
        assert_eq!(m0, 0x7FF);
        assert_eq!(m1, 0);
    }

    #[test]
    fn six_filters_fill_both_mask_buffers() {
        let filters: Vec<CanFilter> = (0u16..6)
            .map(|i| filter(0, 0x100 + i * 0x10, if i < 2 { 0x700 } else { 0x0F0 }))
            .collect();
        let (m0, m1) = compute_mcp_masks(&filters, 0);
        assert_eq!(m0, 0x700);
        assert_eq!(m1, 0x0F0);
    }
}
