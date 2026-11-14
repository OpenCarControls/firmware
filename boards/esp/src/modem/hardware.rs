//! LTE uplink on ESP: powers the modem on, brings it online with AT commands, then runs
//! PPP on the same UART and feeds it to an `embassy-net` stack. Any failure ends in
//! starting over from "does the modem answer `AT`?".
//!
//! Sleep isn't handled yet. The pieces it will need are kept separate: DTR (held
//! active here so the modem stays awake), the ring pin (reserved in the config), and
//! the phases of `lte_task`, which a sleeping phase will join. See docs/lte.md.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::select::{Either, select};
use embassy_net::{ConfigV4, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{ErrorType, Read, ReadReady, Write};
use esp_hal::Async;
use esp_hal::gpio::interconnect::{PeripheralInput, PeripheralOutput};
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::uart::{
    Config as UartConfig, Instance as UartInstance, RxConfig, RxError, TxError, Uart, UartRx,
    UartTx,
};
use static_cell::StaticCell;

use super::at::{self, Final, Registration, SimState};
use super::channel::AtChannel;
use super::kind::{ModemKind, ModemTimings};
use crate::network::{NET_RESOURCES_SOCKETS, NetStack};
use crate::status_leds;

/// The UART the modem is on.
pub type ModemUart = Uart<'static, Async>;

/// Read buffer between the UART and the AT/PPP layers.
const RX_BUFFER: usize = 512;

type Channel = AtChannel<ModemPort, RX_BUFFER>;

/// Bytes received from the modem, waiting for the AT or PPP layer. The UART's hardware
/// FIFO holds only 128 bytes (about 11 ms at 115200 baud), and nothing else reads it while
/// PPP writes a packet (a full one takes about 130 ms) or the AT layer sleeps, so
/// `modem_rx_task` moves everything here as it arrives. The board has no RTS/CTS to make
/// the modem wait.
static MODEM_RX: Pipe<CriticalSectionRawMutex, 4096> = Pipe::new();

/// The RX FIFO level that wakes `modem_rx_task`: well below the 128-byte FIFO, so a
/// busy executor has several milliseconds before bytes are lost. Bytes below it arrive
/// on the RX idle timeout.
const RX_FIFO_THRESHOLD: u16 = 64;

/// When `modem_rx_task` last received a byte, in seconds since boot.
static LAST_RX_S: AtomicU32 = AtomicU32::new(0);

fn now_s() -> u32 {
    Instant::now().as_secs() as u32
}

/// How long PPP may take to come up after `CONNECT`.
const PPP_UP_TIMEOUT_S: u64 = 30;

/// Silence from the modem that ends a PPP session. A dropped data call can leave the
/// session waiting forever (the modem's `NO CARRIER` is text PPP ignores), and the MQTT
/// keepalive (about once a minute) gets an answer on a live link.
const PPP_SILENCE_LIMIT_S: u32 = 180;

/// The modem's serial port as the AT and PPP layers see it: reads come from
/// `MODEM_RX`, writes go to the UART.
pub struct ModemPort {
    tx: UartTx<'static, Async>,
}

impl ErrorType for ModemPort {
    type Error = TxError;
}

impl Read for ModemPort {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        Ok(MODEM_RX.read(buf).await)
    }
}

impl embedded_io::Read for ModemPort {
    /// Only what is already buffered; 0 when there is nothing.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(MODEM_RX.try_read(buf).unwrap_or(0))
    }
}

impl ReadReady for ModemPort {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(!MODEM_RX.is_empty())
    }
}

impl Write for ModemPort {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush_async().await
    }
}

/// Moves received bytes from the UART to `MODEM_RX` as they arrive. Never waits on the
/// reader: if `MODEM_RX` is full (nothing has read it for a while, e.g. after the
/// bring-up stopped), the newest bytes are dropped and counted.
#[embassy_executor::task]
async fn modem_rx_task(mut rx: UartRx<'static, Async>) {
    let mut buf = [0u8; 128];
    let mut dropped: u32 = 0;
    loop {
        match rx.read_async(&mut buf).await {
            Ok(n) => {
                LAST_RX_S.store(now_s(), Ordering::Relaxed);
                let mut data = &buf[..n];
                while !data.is_empty() {
                    match MODEM_RX.try_write(data) {
                        Ok(written) => data = &data[written..],
                        Err(_) => {
                            if dropped == 0 {
                                log::warn!("LTE: modem RX buffer full, dropping input");
                            }
                            dropped = dropped.saturating_add(data.len() as u32);
                            break;
                        }
                    }
                }
                if dropped != 0 && MODEM_RX.is_empty() {
                    log::warn!("LTE: dropped {} byte(s) from the modem", dropped);
                    dropped = 0;
                }
            }
            // Lost bytes: PPP drops the damaged frame and TCP resends it; the AT layer
            // times out and retries. Logged so a bench test shows it.
            Err(RxError::FifoOverflowed) => log::warn!("LTE: UART RX FIFO overflowed"),
            Err(e) => log::warn!("LTE: UART RX error: {:?}", e),
        }
    }
}

/// PPP packet queues (each slot holds one 1500-byte packet).
const PPP_RX_PACKETS: usize = 4;
const PPP_TX_PACKETS: usize = 4;

/// The PDP context the dial uses.
const PDP_CID: u8 = 1;

/// Opens the modem's UART.
pub fn new_modem_uart(
    uart: impl UartInstance + 'static,
    tx: impl PeripheralOutput<'static>,
    rx: impl PeripheralInput<'static>,
    baudrate: u32,
) -> ModemUart {
    let config = UartConfig::default()
        .with_baudrate(baudrate)
        .with_rx(RxConfig::default().with_fifo_full_threshold(RX_FIFO_THRESHOLD));
    Uart::new(uart, config)
        .expect("modem UART config")
        .with_tx(tx)
        .with_rx(rx)
        .into_async()
}

/// A modem control line with its active level (boards put transistors on some lines,
/// which inverts them).
pub struct ControlPin {
    pin: Output<'static>,
    active_high: bool,
}

impl ControlPin {
    /// Takes the pin and drives it to its inactive level.
    pub fn new(pin: impl OutputPin + 'static, active_high: bool) -> Self {
        let inactive = if active_high { Level::Low } else { Level::High };
        Self {
            pin: Output::new(pin, inactive, OutputConfig::default()),
            active_high,
        }
    }

    fn set_active(&mut self, active: bool) {
        self.pin.set_level(if active == self.active_high {
            Level::High
        } else {
            Level::Low
        });
    }

    async fn pulse(&mut self, ms: u32) {
        self.set_active(true);
        Timer::after(Duration::from_millis(ms.into())).await;
        self.set_active(false);
    }
}

/// The modem's control lines this board wires to the ESP. Any may be missing.
pub struct ModemPins {
    /// Switches the modem's supply.
    pub power_enable: Option<ControlPin>,
    /// The modem's power key.
    pub pwrkey: Option<ControlPin>,
    /// The modem's reset line.
    pub reset: Option<ControlPin>,
    /// DTR, active = "host awake": while active the modem may not sleep.
    pub dtr: Option<ControlPin>,
}

/// The LTE settings from `[network.lte]`.
pub struct LteSettings {
    pub kind: ModemKind,
    pub apn: &'static str,
    /// PAP credentials for PPP; most networks ignore them.
    pub username: &'static str,
    pub password: &'static str,
    /// Empty when the SIM has no PIN.
    pub sim_pin: &'static str,
}

/// Creates the LTE network stack and spawns the tasks that drive it. Returns the
/// stack for the MQTT driver; it is configured whenever the PPP link is up.
pub fn init_lte(
    spawner: &embassy_executor::Spawner,
    uart: ModemUart,
    pins: ModemPins,
    settings: LteSettings,
) -> &'static NetStack {
    static PPP_STATE: StaticCell<embassy_net_ppp::State<PPP_RX_PACKETS, PPP_TX_PACKETS>> =
        StaticCell::new();
    static RESOURCES: StaticCell<StackResources<NET_RESOURCES_SOCKETS>> = StaticCell::new();
    static STACK: StaticCell<NetStack> = StaticCell::new();

    let (device, ppp_runner) = embassy_net_ppp::new(PPP_STATE.init(embassy_net_ppp::State::new()));
    // The address comes from PPP (IPCP) once the link is up, not from DHCP.
    let mut seed = [0u8; 8];
    esp_hal::rng::Rng::new().read(&mut seed);
    let (stack, net_runner) = embassy_net::new(
        device,
        embassy_net::Config::default(),
        RESOURCES.init(StackResources::new()),
        u64::from_le_bytes(seed),
    );
    let stack = STACK.init(stack);

    let (rx, tx) = uart.split();
    spawner.spawn(modem_rx_task(rx).unwrap());
    spawner.spawn(ppp_net_task(net_runner).unwrap());
    spawner.spawn(lte_task(ModemPort { tx }, pins, settings, ppp_runner, *stack).unwrap());
    stack
}

#[embassy_executor::task]
async fn ppp_net_task(mut runner: Runner<'static, embassy_net_ppp::Device<'static>>) {
    runner.run().await;
}

/// Why a bring-up stopped.
enum BringUpError {
    /// Start over (after a pause).
    Retry,
    /// Retrying can't help (or would harm, like a wrong PIN locking the SIM). The task
    /// stops until the next boot.
    Fatal(&'static str),
}

#[embassy_executor::task]
async fn lte_task(
    port: ModemPort,
    mut pins: ModemPins,
    settings: LteSettings,
    mut ppp: embassy_net_ppp::Runner<'static>,
    stack: Stack<'static>,
) {
    let timings = settings.kind.timings();
    let mut at = Channel::new(port);
    // A wrong PIN is entered at most once per boot: three wrong ones lock the SIM.
    let mut pin_refused = false;

    if let Some(power) = &mut pins.power_enable {
        power.set_active(true);
    }
    // Keep the modem awake; sleep will release DTR.
    if let Some(dtr) = &mut pins.dtr {
        dtr.set_active(true);
    }

    loop {
        status_leds::set_flag(status_leds::FLAG_LTE, false);

        if !ensure_responsive(&mut at, &mut pins, &timings).await {
            log::warn!("LTE: modem not answering, retrying in 10s");
            Timer::after(Duration::from_secs(10)).await;
            continue;
        }

        match bring_up(&mut at, &settings, &timings, &mut pin_refused).await {
            Ok(()) => {}
            Err(BringUpError::Retry) => {
                Timer::after(Duration::from_secs(10)).await;
                continue;
            }
            Err(BringUpError::Fatal(reason)) => loop {
                log::error!("LTE: stopped until reboot: {}", reason);
                Timer::after(Duration::from_secs(600)).await;
            },
        }

        log::info!("LTE: dialled, starting PPP");
        let config = embassy_net_ppp::Config {
            username: settings.username.as_bytes(),
            password: settings.password.as_bytes(),
        };
        LAST_RX_S.store(now_s(), Ordering::Relaxed);
        let session = ppp.run(at.data_mode(), config, |ipv4| {
            let Some(address) = ipv4.address else {
                log::warn!("LTE: PPP up without an address");
                return;
            };
            let mut dns_servers = heapless::Vec::new();
            for dns in ipv4.dns_servers.iter().flatten() {
                let _ = dns_servers.push(*dns);
            }
            log::info!("LTE: online, address {}, DNS {:?}", address, dns_servers);
            stack.set_config_v4(ConfigV4::Static(StaticConfigV4 {
                // Point-to-point: everything goes to the peer.
                address: Ipv4Cidr::new(address, 0),
                gateway: None,
                dns_servers,
            }));
            status_leds::set_flag(status_leds::FLAG_LTE, true);
        });
        // Dropping the PPP session (the watchdog won) ends it and marks the link down.
        let ended = select(session, watch_ppp(stack)).await;
        stack.set_config_v4(ConfigV4::None);
        status_leds::set_flag(status_leds::FLAG_LTE, false);
        match ended {
            Either::First(Err(embassy_net_ppp::RunError::Terminated)) => {
                log::warn!("LTE: PPP ended by the network")
            }
            Either::First(Err(embassy_net_ppp::RunError::Eof)) => {
                log::warn!("LTE: modem closed the link")
            }
            Either::First(Err(e)) => log::warn!("LTE: PPP failed: {:?}", e),
            Either::First(Ok(never)) => match never {},
            Either::Second(reason) => log::warn!("LTE: ending PPP: {}", reason),
        }
        leave_data_mode(&mut at).await;
        Timer::after(Duration::from_secs(2)).await;
    }
}

/// Returns when a PPP session looks dead: not up soon after `CONNECT`, or no data from
/// the modem for `PPP_SILENCE_LIMIT_S`.
async fn watch_ppp(stack: Stack<'static>) -> &'static str {
    if with_timeout(
        Duration::from_secs(PPP_UP_TIMEOUT_S),
        stack.wait_config_up(),
    )
    .await
    .is_err()
    {
        return "the link didn't come up after CONNECT";
    }
    loop {
        Timer::after(Duration::from_secs(10)).await;
        let silent = now_s().saturating_sub(LAST_RX_S.load(Ordering::Relaxed));
        if silent >= PPP_SILENCE_LIMIT_S {
            return "no data from the modem";
        }
    }
}

/// Sends `cmd` and waits up to `timeout_ms` for its final result. `None` on a timeout
/// or a serial error.
async fn command(
    at: &mut Channel,
    cmd: &str,
    timeout_ms: u32,
    on_line: impl FnMut(&str),
) -> Option<Final> {
    match with_timeout(
        Duration::from_millis(timeout_ms.into()),
        at.command(cmd, on_line),
    )
    .await
    {
        Ok(Ok(result)) => Some(result),
        Ok(Err(e)) => {
            log::warn!("LTE: serial error on '{}': {:?}", cmd, e);
            None
        }
        Err(_) => {
            log::debug!("LTE: no answer to '{}'", cmd);
            None
        }
    }
}

/// Sends `AT` until the modem answers `OK` or `timeout_ms` passes.
async fn probe(at: &mut Channel, timeout_ms: u32) -> bool {
    let deadline = Instant::now() + Duration::from_millis(timeout_ms.into());
    while Instant::now() < deadline {
        if command(at, "AT", 1000, |_| {}).await == Some(Final::Ok) {
            return true;
        }
        Timer::after(Duration::from_millis(200)).await;
    }
    false
}

/// Makes sure the modem answers AT commands, switching it on (or restarting it) if not.
/// A modem that kept running while the ESP restarted answers at once and is left alone.
async fn ensure_responsive(at: &mut Channel, pins: &mut ModemPins, t: &ModemTimings) -> bool {
    if probe(at, 2000).await {
        return true;
    }
    log::info!("LTE: powering the modem on");
    if let Some(reset) = &mut pins.reset {
        reset.pulse(t.reset_ms).await;
    }
    if let Some(pwrkey) = &mut pins.pwrkey {
        pwrkey.pulse(t.pwrkey_on_ms).await;
    }
    probe(at, t.boot_timeout_ms).await
}

/// From a responsive modem to `CONNECT`: SIM, PDP context, registration, dial.
async fn bring_up(
    at: &mut Channel,
    settings: &LteSettings,
    t: &ModemTimings,
    pin_refused: &mut bool,
) -> Result<(), BringUpError> {
    // Echo off, numeric error codes. Neither is required, so their result is ignored.
    let _ = command(at, "ATE0", 1000, |_| {}).await;
    let _ = command(at, "AT+CMEE=1", 1000, |_| {}).await;

    unlock_sim(at, settings, pin_refused).await?;

    let pdp = at::pdp_context(PDP_CID, settings.apn)
        .map_err(|_| BringUpError::Fatal("the APN can't be sent to the modem"))?;
    if command(at, &pdp, 5000, |_| {}).await != Some(Final::Ok) {
        log::warn!("LTE: setting the APN failed");
        return Err(BringUpError::Retry);
    }

    wait_registered(at, t).await?;

    match command(at, at::DIAL_PPP, t.dial_timeout_ms, |_| {}).await {
        Some(Final::Connect) => Ok(()),
        other => {
            log::warn!("LTE: dial failed: {:?}", other);
            Err(BringUpError::Retry)
        }
    }
}

async fn sim_state(at: &mut Channel) -> Option<SimState> {
    let mut state = None;
    let result = command(at, "AT+CPIN?", 5000, |line| {
        if let Some(s) = at::parse_cpin(line) {
            state = Some(s);
        }
    })
    .await;
    if result != Some(Final::Ok) {
        // `+CME ERROR: 10` is "SIM not inserted"; the SIM can also still be starting.
        log::warn!("LTE: SIM not ready ({:?}); is a SIM inserted?", result);
    }
    state
}

async fn unlock_sim(
    at: &mut Channel,
    settings: &LteSettings,
    pin_refused: &mut bool,
) -> Result<(), BringUpError> {
    match sim_state(at).await {
        Some(SimState::Ready) => Ok(()),
        None => Err(BringUpError::Retry),
        Some(SimState::Locked) => Err(BringUpError::Fatal(
            "the SIM is locked (PUK); unlock it in a phone",
        )),
        Some(SimState::PinRequired) => {
            if settings.sim_pin.is_empty() {
                return Err(BringUpError::Fatal(
                    "the SIM needs a PIN: set [network.lte].sim_pin",
                ));
            }
            if *pin_refused {
                return Err(BringUpError::Fatal(
                    "the SIM refused [network.lte].sim_pin; not retried so the SIM doesn't lock",
                ));
            }
            let cmd = at::enter_pin(settings.sim_pin)
                .map_err(|_| BringUpError::Fatal("[network.lte].sim_pin isn't 4-8 digits"))?;
            match command(at, &cmd, 5000, |_| {}).await {
                Some(Final::Ok) => {
                    log::info!("LTE: SIM unlocked");
                    // The SIM takes a moment to be ready; the next attempt checks it.
                    Timer::after(Duration::from_secs(3)).await;
                    Err(BringUpError::Retry)
                }
                Some(_) => {
                    *pin_refused = true;
                    Err(BringUpError::Fatal(
                        "the SIM refused [network.lte].sim_pin; not retried so the SIM doesn't lock",
                    ))
                }
                // No answer: unknown whether it counted. Check the state again first.
                None => Err(BringUpError::Retry),
            }
        }
    }
}

/// Polls the LTE registration until the modem is registered (home or roaming).
async fn wait_registered(at: &mut Channel, t: &ModemTimings) -> Result<(), BringUpError> {
    let deadline = Instant::now() + Duration::from_millis(t.registration_timeout_ms.into());
    let mut last = None;
    let mut polls: u32 = 0;
    while Instant::now() < deadline {
        let mut reg = None;
        let _ = command(at, "AT+CEREG?", 2000, |line| {
            if let Some(r) = at::parse_registration(line, "+CEREG") {
                reg = Some(r);
            }
        })
        .await;
        if let Some(r) = reg {
            if r.is_registered() {
                log::info!("LTE: registered ({:?})", r);
                return Ok(());
            }
            if last != Some(r) {
                log::info!("LTE: registration: {:?}", r);
                if r == Registration::Denied {
                    log::warn!("LTE: the network denied registration; check the SIM's plan");
                }
                last = Some(r);
            }
        }
        // Signal strength now and then, for bench debugging.
        if polls.is_multiple_of(10) {
            let _ = command(at, "AT+CSQ", 2000, |line| {
                if let Some(dbm) = at::parse_csq_dbm(line) {
                    log::info!("LTE: signal {:?} dBm", dbm);
                }
            })
            .await;
        }
        polls += 1;
        Timer::after(Duration::from_secs(2)).await;
    }
    log::warn!(
        "LTE: not registered after {} s",
        t.registration_timeout_ms / 1000
    );
    Err(BringUpError::Retry)
}

/// Gets back to command mode after PPP: the `+++` escape (guarded by a second of
/// silence on each side), then hangs up. If the modem already left data mode, the
/// escape is just a line it rejects.
async fn leave_data_mode(at: &mut Channel) {
    Timer::after(Duration::from_millis(1100)).await;
    if let Err(e) = at.write_raw(b"+++").await {
        log::warn!("LTE: serial error leaving data mode: {:?}", e);
    }
    Timer::after(Duration::from_millis(1100)).await;
    let _ = command(at, "ATH", 3000, |_| {}).await;
}
