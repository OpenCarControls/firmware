#![no_std]

use core_interface::{CanFilter, CanFrame, CAN_RX_CHANNEL, CAN_TX_CHANNEL};
use embedded_can::{Frame as EmbeddedFrame, Id, StandardId};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    delay::Delay,
    gpio::{Input, Output},
    gpio::interconnect::{PeripheralInput, PeripheralOutput},
    spi::master::Spi,
    twai::{self, Twai, TwaiMode},
};
pub use mcp2515::{CanSpeed, McpSpeed};
use mcp2515::{
    error::Error as McpError,
    filter::{RxFilter, RxMask},
    frame::CanFrame as McpFrame,
    regs::OpMode,
    MCP2515,
};

// ── Type Aliases ─────────────────────────────────────────────────────────────

/// Concrete TWAI driver type (Blocking mode → implements Send, safe for core 1).
pub type TwaiDriver = Twai<'static, esp_hal::Blocking>;

/// Concrete MCP2515 driver type (blocking SPI, INT-pin driven).
pub type Mcp2515Driver = MCP2515<ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>>;

/// The INT input pin type used for MCP2515 interrupt-driven RX.
pub type CanIntPin = Input<'static>;

// ── Board Entry Point ─────────────────────────────────────────────────────────

pub fn start(spawner: &embassy_executor::Spawner) {
    spawner.spawn(core_interface::process_ble_commands_task()).unwrap();
    spawner.spawn(core_interface::process_mqtt_commands_task()).unwrap();
    spawner.spawn(core_interface::route_responses_task()).unwrap();
    spawner.spawn(core_interface::publish_state_task()).unwrap();
}

// ── CAN ID helpers ────────────────────────────────────────────────────────────

fn filter_id_raw(f: &CanFilter) -> u32 {
    match f.id {
        Id::Standard(sid) => sid.as_raw() as u32,
        Id::Extended(eid) => eid.as_raw(),
    }
}

fn id_raw(id: Id) -> u32 {
    match id {
        Id::Standard(sid) => sid.as_raw() as u32,
        Id::Extended(eid) => eid.as_raw(),
    }
}

fn passes_software_filter(frame: &CanFrame, filters: &[CanFilter]) -> bool {
    let frame_raw = id_raw(frame.id);
    filters
        .iter()
        .filter(|f| f.bus_id == frame.bus_id)
        .any(|f| (frame_raw & f.mask) == (filter_id_raw(f) & f.mask))
}

// ── TWAI (built-in CAN) ───────────────────────────────────────────────────────

/// Initialises the TWAI peripheral with an accept-all hardware filter; actual
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
                    if passes_software_filter(&core_frame, filters) {
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
                if let Some(f) = core_to_twai_frame(&outbound) {
                    // nb::block! would stall the executor; spin until TX buffer free
                    loop {
                        match tx.transmit(&f) {
                            Ok(_) | Err(nb::Error::Other(_)) => break,
                            Err(nb::Error::WouldBlock) => Timer::after(Duration::from_micros(100)).await,
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

fn twai_to_core_frame(frame: twai::EspTwaiFrame, bus_id: u8) -> CanFrame {
    let id = EmbeddedFrame::id(&frame);
    let dlc = EmbeddedFrame::dlc(&frame) as u8;
    let raw = EmbeddedFrame::data(&frame);
    let mut data = [0u8; 8];
    data[..raw.len()].copy_from_slice(raw);
    CanFrame { bus_id, id, data, dlc }
}

fn core_to_twai_frame(frame: &CanFrame) -> Option<twai::EspTwaiFrame> {
    let data = &frame.data[..frame.dlc as usize];
    // esp_hal::twai::Id implements From<embedded_can::Id>
    twai::EspTwaiFrame::new(frame.id, data)
}

// ── MCP2515 (SPI CAN) ─────────────────────────────────────────────────────────

const RX_FILTERS: [RxFilter; 6] = [
    RxFilter::F0, RxFilter::F1, RxFilter::F2,
    RxFilter::F3, RxFilter::F4, RxFilter::F5,
];

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
    // Mask bit=1 means the bit must match. Start with all-don't-care (0).
    let mask0 = filters.iter().filter(|f| f.bus_id == bus_id).take(2)
        .fold(0u32, |acc, f| acc | f.mask);
    let mask1 = filters.iter().filter(|f| f.bus_id == bus_id).skip(2).take(4)
        .fold(0u32, |acc, f| acc | f.mask);

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

/// Bidirectional MCP2515 loop — runs forever on core 1.
pub async fn run_mcp2515_loop(
    mut driver: Mcp2515Driver,
    mut int_pin: CanIntPin,
    bus_id: u8,
    filters: &'static [CanFilter],
) {
    loop {
        int_pin.wait_for_falling_edge().await;

        loop {
            match driver.read_message() {
                Ok(frame) => {
                    let core_frame = mcp_to_core_frame(frame, bus_id);
                    if passes_software_filter(&core_frame, filters) {
                        CAN_RX_CHANNEL.sender().send(core_frame).await;
                    }
                }
                Err(McpError::NoMessage) | Err(_) => break,
            }
        }

        while let Ok(outbound) = CAN_TX_CHANNEL.receiver().try_receive() {
            if outbound.bus_id == bus_id {
                if let Some(f) = core_to_mcp_frame(&outbound) {
                    let _ = driver.send_message(f);
                }
            } else {
                let _ = CAN_TX_CHANNEL.sender().try_send(outbound);
            }
        }
    }
}

fn mcp_to_core_frame(frame: McpFrame, bus_id: u8) -> CanFrame {
    let id = EmbeddedFrame::id(&frame);
    let dlc = EmbeddedFrame::dlc(&frame) as u8;
    let raw = EmbeddedFrame::data(&frame);
    let mut data = [0u8; 8];
    data[..raw.len()].copy_from_slice(raw);
    CanFrame { bus_id, id, data, dlc }
}

fn core_to_mcp_frame(frame: &CanFrame) -> Option<McpFrame> {
    McpFrame::new(frame.id, &frame.data[..frame.dlc as usize])
}
