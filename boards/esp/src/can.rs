//! CAN bus drivers for the ESP32 board.
//!
//! Two driver implementations are provided:
//! - **TWAI** – the ESP32's built-in CAN peripheral, interrupt-driven (esp-hal async mode).
//! - **MCP2515** – external SPI CAN controller, interrupt-driven via a falling-edge INT pin.
//!
//! Both share the same channel contract: `CAN_RX_CHANNEL` (board → vehicle) and
//! `CAN_TX_CHANNELS[bus_id]` (vehicle → board, one queue per bus). All frame filtering
//! against the vehicle's `CanFilter` list is done in software after the hardware delivers
//! the frame.

use core_interface::{CanFilter, CanFrame};
use embedded_can::Frame as EmbeddedFrame;

/// Converts any [`EmbeddedFrame`]-implementing hardware frame into the
/// crate-shared [`CanFrame`] type. Used by both the TWAI and MCP2515 drivers.
///
/// Returns `None` for remote (RTR) frames. A remote frame asks another node to send
/// the frame with that ID: it has a DLC but no data, so passed on it would look like
/// a data frame full of zeros and be decoded as real values (doors locked, speed 0).
/// `CanFrame` has no RTR flag, so they are dropped here. They could still be useful
/// later, e.g. to record every frame in CAN logging; that needs an RTR flag on
/// `CanFrame` (and `CanLogFrame`) first.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
fn to_core_frame<F: EmbeddedFrame>(frame: F, bus_id: u8) -> Option<CanFrame> {
    if frame.is_remote_frame() {
        return None;
    }
    let id = frame.id();
    let dlc = frame.dlc() as u8;
    let raw = frame.data();
    let mut data = [0u8; 8];
    data[..raw.len()].copy_from_slice(raw);
    Some(CanFrame {
        bus_id,
        id,
        data,
        dlc,
    })
}

#[cfg(feature = "hardware")]
mod hardware {
    use super::{CanFilter, to_core_frame};
    use core_interface::{CAN_RX_CHANNEL, CanFrame, CanReadOnlyReceiver};
    use embedded_can::Frame as EmbeddedFrame;
    use embedded_can::{Id, StandardId};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use esp_hal::{
        delay::Delay,
        gpio::interconnect::{PeripheralInput, PeripheralOutput},
        gpio::{AnyPin, Input, Output},
        spi::master::Spi,
        twai::{self, EspTwaiError, Twai, TwaiConfiguration, TwaiMode},
    };
    pub use mcp2515::{CanSpeed, McpSpeed};
    use mcp2515::{
        MCP2515,
        filter::{RxFilter, RxMask},
        frame::CanFrame as McpFrame,
        regs::OpMode,
    };

    /// TWAI peripheral and pins, owned by `run_twai_loop`. The driver is rebuilt from
    /// these (via `reborrow()`) whenever the operating mode has to change, because esp-hal
    /// only accepts the mode at construction. All fields are `Send`, so this can be
    /// handed to core 1; the async driver (and its interrupt) is created on whichever
    /// core runs the loop.
    pub struct TwaiDriver {
        twai: twai::AnyTwai<'static>,
        rx_pin: AnyPin<'static>,
        tx_pin: AnyPin<'static>,
    }

    /// Concrete MCP2515 driver type (blocking SPI, INT-pin driven).
    pub type Mcp2515Driver =
        MCP2515<ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>>;

    /// The INT input pin type used for MCP2515 interrupt-driven RX.
    pub type CanIntPin = Input<'static>;

    const RX_FILTERS: [RxFilter; 6] = [
        RxFilter::F0,
        RxFilter::F1,
        RxFilter::F2,
        RxFilter::F3,
        RxFilter::F4,
        RxFilter::F5,
    ];

    /// Takes ownership of the TWAI peripheral and its pins. The controller is configured
    /// and started by `run_twai_loop`, not here.
    pub fn init_twai(
        peripheral: impl Into<twai::AnyTwai<'static>>,
        rx_pin: impl Into<AnyPin<'static>>,
        tx_pin: impl Into<AnyPin<'static>>,
        _filters: &[CanFilter],
    ) -> TwaiDriver {
        TwaiDriver {
            twai: peripheral.into(),
            rx_pin: rx_pin.into(),
            tx_pin: tx_pin.into(),
        }
    }

    /// Builds a TWAI configuration in `mode` with an accept-all hardware filter; actual
    /// frame selection is done in software inside `run_twai_session`.
    fn configure_twai(
        driver: &mut TwaiDriver,
        mode: TwaiMode,
    ) -> TwaiConfiguration<'_, esp_hal::Blocking> {
        // Mask = 0 in esp-hal TWAI means "don't care" for every bit -> accept all.
        let accept_all = twai::filter::SingleStandardFilter::new_from_code_mask(
            unsafe { twai::StandardId::new_unchecked(0) },
            unsafe { twai::StandardId::new_unchecked(0) },
            false,
            false,
            [0, 0],
            [0, 0],
        );
        let mut cfg = TwaiConfiguration::new(
            driver.twai.reborrow(),
            driver.rx_pin.reborrow(),
            driver.tx_pin.reborrow(),
            esp_hal::twai::BaudRate::B500K,
            mode,
        );
        cfg.set_filter(accept_all);
        cfg
    }

    /// Bidirectional TWAI loop - runs forever (core 1 on dual-core chips, core 0 otherwise).
    ///
    /// While CAN read-only mode is on, the controller runs in hardware **listen-only**
    /// mode: it never drives the bus (no ACKs, no error frames, no TX). When the flag
    /// changes, the controller is reset and restarted in the matching mode.
    ///
    /// Also restarts the controller after a bus-off (with a 1 s back-off).
    pub async fn run_twai_loop(mut driver: TwaiDriver, bus_id: u8, filters: &'static [CanFilter]) {
        let mut read_only_changed = core_interface::can_read_only_receiver()
            .expect("more CAN bus drivers than MAX_CAN_BUSES");
        let mut stats = TwaiRxStats::new();
        let mut next_report = embassy_time::Instant::now() + RX_STATS_INTERVAL;
        loop {
            let listen_only = core_interface::is_can_read_only();
            let mode = if listen_only {
                TwaiMode::ListenOnly
            } else {
                TwaiMode::Normal
            };
            log::info!(
                "CAN bus {}: TWAI started in {} mode",
                bus_id,
                if listen_only { "listen-only" } else { "normal" }
            );
            let mut twai = configure_twai(&mut driver, mode).into_async().start();
            run_twai_session(
                &mut twai,
                bus_id,
                filters,
                listen_only,
                &mut read_only_changed,
                &mut stats,
                &mut next_report,
            )
            .await;
            // Dropping `twai` resets the peripheral before it is reconfigured.
        }
    }

    /// Runs one TWAI session in a fixed mode. Returns when the CAN read-only flag no longer
    /// matches `listen_only`, or after a bus-off, so the caller can rebuild the controller.
    ///
    /// RX is interrupt-driven: the esp-hal TWAI ISR moves frames from the small hardware
    /// FIFO into a 32-frame software queue, so frames are not lost while the executor is
    /// busy with WiFi/BLE work (which matters on single-core chips like the ESP32-C3).
    /// The loop sleeps until a frame arrives, a TX frame is queued for this bus, the
    /// read-only flag changes, or the next RX stats report is due.
    #[allow(clippy::too_many_arguments)]
    async fn run_twai_session(
        twai: &mut Twai<'_, esp_hal::Async>,
        bus_id: u8,
        filters: &'static [CanFilter],
        listen_only: bool,
        read_only_changed: &mut CanReadOnlyReceiver,
        stats: &mut TwaiRxStats,
        next_report: &mut embassy_time::Instant,
    ) {
        use embassy_futures::select::{Either4, select4};
        use embassy_time::{Duration, Instant, Timer};

        let tx_channel =
            core_interface::can_tx_channel(bus_id).expect("CAN bus id exceeds MAX_CAN_BUSES");

        loop {
            if core_interface::is_can_read_only() != listen_only {
                return;
            }

            let mut pending_tx = None;
            match select4(
                twai.receive_async(),
                tx_channel.receive(),
                read_only_changed.changed(),
                Timer::at(*next_report),
            )
            .await
            {
                Either4::First(Ok(frame)) => {
                    crate::status_leds::pulse_can();
                    let Some(core_frame) = to_core_frame(frame, bus_id) else {
                        continue;
                    };
                    core_interface::log_can_frame(&core_frame);
                    let matched = core_interface::passes_filter(&core_frame, filters);
                    stats.record_frame(matched);
                    // The vehicle doesn't see frames while SavvyCAN has the buses.
                    if matched && !core_interface::is_can_logging() {
                        CAN_RX_CHANNEL.sender().send(core_frame).await;
                    }
                }
                Either4::First(Err(EspTwaiError::BusOff)) => {
                    stats.record_bus_off();
                    log::warn!(
                        "CAN bus {}: TWAI bus-off, restarting controller in 1s",
                        bus_id
                    );
                    Timer::after(Duration::from_secs(1)).await;
                    return;
                }
                Either4::First(Err(EspTwaiError::EmbeddedHAL(twai::ErrorKind::Overrun))) => {
                    stats.record_overrun();
                    log::warn!("CAN bus {}: TWAI RX overrun, frames dropped", bus_id);
                    twai.clear_receive_fifo();
                }
                Either4::First(Err(e)) => {
                    stats.record_error(e);
                    log::debug!("CAN bus {}: TWAI RX error: {:?}", bus_id, e);
                }
                Either4::Second(outbound) => pending_tx = Some(outbound),
                // The flag is re-checked at the top of the loop.
                Either4::Third(_) => {}
                Either4::Fourth(()) => {
                    log_rx_stats(bus_id, listen_only, &stats.take());
                    *next_report = Instant::now() + RX_STATS_INTERVAL;
                }
            }

            // `select4` polls RX first, so a busy bus could starve the TX arm; drain TX
            // here every iteration regardless of which arm fired.
            while let Some(outbound) = pending_tx.take().or_else(|| tx_channel.try_receive().ok()) {
                // Drop silently when in read-only mode; do not transmit on the bus.
                // Both checks matter: the flag may have flipped since this session
                // started, and the controller is only rebuilt at the top of the loop.
                if !listen_only && !core_interface::is_can_read_only() {
                    transmit_twai(twai, bus_id, &outbound).await;
                }
            }
        }
    }

    /// How often each CAN bus logs its RX counters, including when nothing was received.
    const RX_STATS_INTERVAL: embassy_time::Duration = embassy_time::Duration::from_secs(5);

    type TwaiRxStats = crate::can::RxStats<EspTwaiError>;

    /// Logs one RX stats window. A silent bus logs zeros, so "no data" is visible on the
    /// serial console; errors without frames usually mean a wrong bitrate or wiring.
    fn log_rx_stats(bus_id: u8, listen_only: bool, stats: &TwaiRxStats) {
        let mode = if listen_only { "listen-only" } else { "normal" };
        let secs = RX_STATS_INTERVAL.as_secs();
        match stats.last_error {
            Some(e) => log::info!(
                "CAN bus {} ({}), last {}s: {} frames, {} matched filters, {} RX errors (last: {:?}), {} overruns, {} bus-offs",
                bus_id,
                mode,
                secs,
                stats.received,
                stats.matched,
                stats.errors,
                e,
                stats.overruns,
                stats.bus_offs
            ),
            None => log::info!(
                "CAN bus {} ({}), last {}s: {} frames, {} matched filters, 0 RX errors, {} overruns, {} bus-offs",
                bus_id,
                mode,
                secs,
                stats.received,
                stats.matched,
                stats.overruns,
                stats.bus_offs
            ),
        }
    }

    async fn transmit_twai(twai: &mut Twai<'_, esp_hal::Async>, bus_id: u8, outbound: &CanFrame) {
        use embassy_time::{Duration, with_timeout};

        if let Some(f) = core_to_twai_frame(outbound) {
            // Bounded wait: without another node to ACK, a frame never
            // completes. Dropping the future aborts the transmission.
            match with_timeout(Duration::from_millis(100), twai.transmit_async(&f)).await {
                Ok(Ok(())) => crate::status_leds::pulse_can(),
                Ok(Err(e)) => log::debug!("CAN bus {}: TWAI TX error: {:?}", bus_id, e),
                Err(_) => log::debug!("CAN bus {}: TWAI TX timed out", bus_id),
            }
        }
    }

    fn core_to_twai_frame(frame: &CanFrame) -> Option<twai::EspTwaiFrame> {
        let data = &frame.data[..frame.dlc as usize];
        // esp_hal::twai::Id implements From<embedded_can::Id>
        twai::EspTwaiFrame::new(frame.id, data)
    }

    /// Initialises an MCP2515 via blocking SPI. Programs hardware RX filters/masks.
    /// The chip is left in listen-only mode; `run_mcp2515_loop` applies the mode that
    /// matches the CAN read-only flag. Returns `(driver, int_pin)`. Pass both to
    /// `run_mcp2515_loop`.
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
                mode: OpMode::ListenOnly,
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

    /// Bidirectional MCP2515 loop — runs forever on core 1.
    ///
    /// Waits for the INT pin to fall (MCP2515 asserts it on any RX or TX event), a TX frame
    /// to be queued for this bus, a read-only flag change, or at most 10 ms, then drains
    /// all available received frames and the outbound TX channel.
    ///
    /// While CAN read-only mode is on, the chip runs in hardware **listen-only** mode: it
    /// never drives the bus (no ACKs, no error frames, no TX). The mode follows the flag.
    ///
    /// Hardware RX masks are swapped to accept-all when CAN debug mode is active so the
    /// debug tap can observe every frame on the bus, and restored when debug is disabled.
    pub async fn run_mcp2515_loop(
        mut driver: Mcp2515Driver,
        mut int_pin: CanIntPin,
        bus_id: u8,
        filters: &'static [CanFilter],
    ) {
        use embassy_futures::select::select4;
        use embassy_time::{Duration, Timer};

        let tx_channel =
            core_interface::can_tx_channel(bus_id).expect("CAN bus id exceeds MAX_CAN_BUSES");
        let mut read_only_changed = core_interface::can_read_only_receiver()
            .expect("more CAN bus drivers than MAX_CAN_BUSES");

        let mut was_logging = false;
        let mut listen_only = core_interface::is_can_read_only();
        set_mcp_operating_mode(&mut driver, bus_id, listen_only);
        loop {
            if core_interface::is_can_read_only() != listen_only {
                listen_only = !listen_only;
                set_mcp_operating_mode(&mut driver, bus_id, listen_only);
            }

            // MCP2515 hardware mask switching when SavvyCAN logging starts/stops.
            let logging_now = core_interface::is_can_logging();
            if logging_now && !was_logging {
                // Accept all frames in hardware so the logger sees raw traffic.
                set_mcp_accept_all_masks(&mut driver);
            } else if !logging_now && was_logging {
                // Restore vehicle-specific hardware masks.
                set_mcp_vehicle_masks(&mut driver, filters, bus_id);
            }
            was_logging = logging_now;

            // INT is level-triggered: if it is already low (frames left pending), no edge
            // arrives, so the timeout also guarantees RX is serviced and the logging-mask
            // state above is re-evaluated.
            let mut pending_tx = match select4(
                int_pin.wait_for_falling_edge(),
                Timer::after(Duration::from_millis(10)),
                tx_channel.receive(),
                read_only_changed.changed(),
            )
            .await
            {
                embassy_futures::select::Either4::Third(outbound) => Some(outbound),
                _ => None,
            };

            loop {
                match driver.read_message() {
                    Ok(frame) => {
                        crate::status_leds::pulse_can();
                        let Some(core_frame) = to_core_frame(frame, bus_id) else {
                            continue;
                        };
                        core_interface::log_can_frame(&core_frame);
                        if core_interface::passes_filter(&core_frame, filters)
                            && !core_interface::is_can_logging()
                        {
                            CAN_RX_CHANNEL.sender().send(core_frame).await;
                        }
                    }
                    Err(_) => break, // NoMessage (FIFO empty) or hardware error; either way done
                }
            }

            while let Some(outbound) = pending_tx.take().or_else(|| tx_channel.try_receive().ok()) {
                // Drop silently when in read-only mode; do not transmit on the bus.
                // Both checks matter: the flag may have flipped since the mode was
                // last applied at the top of the loop.
                if !listen_only && !core_interface::is_can_read_only() {
                    if let Some(f) = core_to_mcp_frame(&outbound) {
                        if driver.send_message(f).is_ok() {
                            crate::status_leds::pulse_can();
                        }
                    }
                }
            }
        }
    }

    /// Switches the MCP2515 between listen-only (CAN read-only mode) and normal mode.
    fn set_mcp_operating_mode(driver: &mut Mcp2515Driver, bus_id: u8, listen_only: bool) {
        let mode = if listen_only {
            OpMode::ListenOnly
        } else {
            OpMode::Normal
        };
        match driver.set_mode(mode) {
            Ok(()) => log::info!(
                "CAN bus {}: MCP2515 in {} mode",
                bus_id,
                if listen_only { "listen-only" } else { "normal" }
            ),
            Err(_) => log::error!("CAN bus {}: failed to set MCP2515 mode", bus_id),
        }
    }

    fn core_to_mcp_frame(frame: &CanFrame) -> Option<McpFrame> {
        McpFrame::new(frame.id, &frame.data[..frame.dlc as usize])
    }

    /// Switches both MCP2515 RX buffer masks to accept-all (mask bits = 0).
    /// Called when SavvyCAN logging starts so the logger can observe every frame on
    /// the bus before the software filter runs.
    fn set_mcp_accept_all_masks(driver: &mut Mcp2515Driver) {
        // StandardId 0 is always valid (fits in 11 bits); no unsafe needed.
        let id_zero = Id::Standard(StandardId::new(0).unwrap());
        driver.set_mask(RxMask::Mask0, id_zero).ok();
        driver.set_mask(RxMask::Mask1, id_zero).ok();
    }

    /// Restores the MCP2515 RX buffer masks derived from the vehicle's `CanFilter` list.
    /// Called when SavvyCAN logging stops to return the hardware to selective
    /// filtering and reduce CPU load from unwanted frames reaching the software path.
    fn set_mcp_vehicle_masks(driver: &mut Mcp2515Driver, filters: &[CanFilter], bus_id: u8) {
        let (mask0, mask1) = super::compute_mcp_masks(filters, bus_id);
        if let Some(sid) = StandardId::new((mask0 & 0x7FF) as u16) {
            driver.set_mask(RxMask::Mask0, Id::Standard(sid)).ok();
        }
        if let Some(sid) = StandardId::new((mask1 & 0x7FF) as u16) {
            driver.set_mask(RxMask::Mask1, Id::Standard(sid)).ok();
        }
    }
}

#[cfg(feature = "hardware")]
pub use hardware::*;

/// RX counters behind the periodic CAN health log. `take` returns them and starts a fresh
/// window, so each log line covers one reporting interval. `E` is the driver's error type.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
#[derive(Debug, PartialEq)]
pub(crate) struct RxStats<E> {
    pub received: u32,
    pub matched: u32,
    pub errors: u32,
    pub overruns: u32,
    pub bus_offs: u32,
    pub last_error: Option<E>,
}

#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
impl<E> RxStats<E> {
    pub const fn new() -> Self {
        Self {
            received: 0,
            matched: 0,
            errors: 0,
            overruns: 0,
            bus_offs: 0,
            last_error: None,
        }
    }

    pub fn record_frame(&mut self, matched: bool) {
        self.received = self.received.saturating_add(1);
        if matched {
            self.matched = self.matched.saturating_add(1);
        }
    }

    pub fn record_error(&mut self, error: E) {
        self.errors = self.errors.saturating_add(1);
        self.last_error = Some(error);
    }

    pub fn record_overrun(&mut self) {
        self.overruns = self.overruns.saturating_add(1);
    }

    pub fn record_bus_off(&mut self) {
        self.bus_offs = self.bus_offs.saturating_add(1);
    }

    pub fn take(&mut self) -> Self {
        core::mem::replace(self, Self::new())
    }
}

/// Number of RX acceptance filters in the MCP2515 hardware.
pub const MCP2515_HW_FILTERS: usize = 6;

/// Number of `filters` that apply to `bus_id`. `const` so the generated `main.rs` can
/// refuse to build an MCP2515 bus with more filters than the chip has (see
/// `xtask_builder.rs`).
pub const fn bus_filter_count(filters: &[CanFilter], bus_id: u8) -> usize {
    let mut count = 0;
    let mut i = 0;
    while i < filters.len() {
        if filters[i].bus_id == bus_id {
            count += 1;
        }
        i += 1;
    }
    count
}

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

    #[test]
    fn rx_stats_count_frames_and_errors() {
        let mut stats = RxStats::<&str>::new();
        stats.record_frame(true);
        stats.record_frame(false);
        stats.record_error("bit");
        stats.record_error("stuff");
        stats.record_overrun();
        stats.record_bus_off();
        assert_eq!(
            stats,
            RxStats {
                received: 2,
                matched: 1,
                errors: 2,
                overruns: 1,
                bus_offs: 1,
                last_error: Some("stuff"),
            }
        );
    }

    #[test]
    fn rx_stats_take_starts_a_new_window() {
        let mut stats = RxStats::<&str>::new();
        stats.record_frame(true);
        stats.record_error("bit");
        let report = stats.take();
        assert_eq!(report.received, 1);
        assert_eq!(report.last_error, Some("bit"));
        assert_eq!(stats, RxStats::new());
    }

    #[test]
    fn rx_stats_saturate_instead_of_wrapping() {
        let mut stats = RxStats::<&str>::new();
        stats.received = u32::MAX;
        stats.record_frame(false);
        assert_eq!(stats.received, u32::MAX);
    }

    /// Minimal stand-in for a driver's frame type.
    struct TestFrame {
        id: Id,
        data: heapless::Vec<u8, 8>,
        /// Set for remote frames, which have a DLC but no data.
        remote_dlc: Option<usize>,
    }

    impl EmbeddedFrame for TestFrame {
        fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
            Some(Self {
                id: id.into(),
                data: heapless::Vec::from_slice(data).ok()?,
                remote_dlc: None,
            })
        }
        fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
            Some(Self {
                id: id.into(),
                data: heapless::Vec::new(),
                remote_dlc: Some(dlc),
            })
        }
        fn is_extended(&self) -> bool {
            matches!(self.id, Id::Extended(_))
        }
        fn is_remote_frame(&self) -> bool {
            self.remote_dlc.is_some()
        }
        fn id(&self) -> Id {
            self.id
        }
        fn dlc(&self) -> usize {
            self.remote_dlc.unwrap_or(self.data.len())
        }
        fn data(&self) -> &[u8] {
            &self.data
        }
    }

    #[test]
    fn to_core_frame_copies_id_and_pads_short_payloads() {
        let id = Id::Standard(StandardId::new(0x123).unwrap());
        let frame = to_core_frame(TestFrame::new(id, &[0xAA, 0xBB, 0xCC]).unwrap(), 2).unwrap();
        assert_eq!(frame.bus_id, 2);
        assert_eq!(frame.id, id);
        assert_eq!(frame.dlc, 3);
        assert_eq!(frame.data, [0xAA, 0xBB, 0xCC, 0, 0, 0, 0, 0]);
    }

    /// A remote frame on BODY_STATUS must not become an all-zero data frame.
    #[test]
    fn to_core_frame_drops_remote_frames() {
        let id = Id::Standard(StandardId::new(1041).unwrap());
        assert!(to_core_frame(TestFrame::new_remote(id, 8).unwrap(), 0).is_none());
    }

    #[test]
    fn bus_filter_count_counts_only_that_bus() {
        let filters = [
            filter(0, 0x100, 0x7FF),
            filter(1, 0x200, 0x7FF),
            filter(0, 0x300, 0x7FF),
        ];
        assert_eq!(bus_filter_count(&filters, 0), 2);
        assert_eq!(bus_filter_count(&filters, 1), 1);
        assert_eq!(bus_filter_count(&filters, 2), 0);
    }
}
