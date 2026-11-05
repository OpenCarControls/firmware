# LTE uplink

An ESP board with a cellular modem can reach the MQTT broker over LTE instead of WiFi. The modem is a plain data pipe: the firmware brings it online with AT commands, then runs PPP over the same UART and hands the link to `embassy-net`. Everything above the network stack (the MQTT driver, DNS, TLS once it exists) works the same over LTE as over WiFi.

Related docs: [Architecture](architecture.md) · [Hardware setup](hardware-setup.md)

## Configuration

The settings are split the same way as WiFi and CAN: the board's wiring goes in the hardware profile, the SIM's settings in `config.toml`.

```toml
# hardware profile (e.g. hardware-profiles/lilygo-t-sim7670g-s3.toml): how the modem is wired
[hardware.esp.modem]
kind = "sim7670g"        # the modem model; see "Supported modems"
uart_tx_pin = 11         # ESP → modem RXD
uart_rx_pin = 10         # modem TXD → ESP
baudrate = 115200        # default
pwrkey_pin = 18          # optional
pwrkey_level = "high"    # the level at the ESP pin that presses the key (default "high")
reset_pin = 17           # optional
reset_level = "low"      # default "low"
# power_enable_pin = 4   # optional, for boards that switch the modem's supply
# power_enable_level = "high"
dtr_pin = 9              # optional, held low so the modem stays awake
ring_pin = 3             # optional, reserved for wake-up (not used yet)

# config.toml: the SIM
[network.wifi]
enabled = false          # one uplink per build

[network.lte]
enabled = true
apn = "your.carrier.apn"
# username = ""          # PAP credentials, if the carrier needs them
# password = ""
# sim_pin = "1234"       # only for a SIM with a PIN
```

xtask checks the combination before building:

*   A build has one uplink: WiFi and LTE can't both be enabled.
*   `[network.lte]` needs a `[hardware.esp.modem]`; unknown keys in that section are errors, so a typo in a pin name doesn't go unnoticed.
*   The APN must be printable ASCII without `"` (it is quoted in an AT command), and the SIM PIN 4 to 8 digits.
*   The SavvyCAN logger needs WiFi: on a carrier network nothing can connect to the device.
*   The PC board refuses `[network.lte]`: on Linux the OS brings the modem up (ModemManager, NetworkManager) and the firmware uses it like any other network.

The modem's UART is `UART1`. `board-esp` keeps the control lines (`ControlPin`) with their active level, because boards often put a transistor on PWRKEY or RESET, which inverts them.

## Supported boards and modems

| Modem | `kind` | Boards |
|---|---|---|
| SIMCom SIM7670G (LTE Cat-1 bis) | `sim7670g` | LilyGO T-SIM7670G-S3 (`lilygo-t-sim7670g-s3` profile) |

The LilyGO profile has no CAN bus: the board has no transceiver, so wire one to free GPIOs and add a `[[hardware.esp.can_buses]]` entry in `config.toml`. LilyGO also sells a "SIMCom standard interface" variant of the board with different pins; the profile is not for that one.

**Adding a modem** is a new `ModemKind` variant in `boards/esp/src/modem/kind.rs` (its power-on timings), its name in `modem_kind_ident` in `boards/esp/xtask_builder.rs`, and any AT differences in the bring-up. **Adding a board** with a supported modem is only a hardware profile.

## How it works

`init_lte` (`boards/esp/src/modem/hardware.rs`) creates an `embassy-net` stack on an `embassy-net-ppp` device and spawns `lte_task`, which loops through:

1.  **Make sure the modem answers `AT`.** A modem that kept running while the ESP restarted answers at once and is left alone. Otherwise the task pulses RESET, then PWRKEY, and waits for the modem to boot.
2.  **Bring it up:** echo off, the SIM (`AT+CPIN?`, entering the PIN if needed), the PDP context with the APN (`AT+CGDCONT`), then polls the LTE registration (`AT+CEREG?`) until the modem is registered at home or roaming, logging the signal strength (`AT+CSQ`) now and then.
3.  **Dial** `ATD*99#`. On `CONNECT` the UART carries PPP. The address and DNS servers the network assigns (IPCP) become the stack's static config, and the MQTT driver, waiting for the network, connects.
4.  **When PPP ends**, the stack's config is cleared (the MQTT session drops and waits), the task leaves data mode (`+++`, then `ATH`) and starts over at 1.

Any failure goes back to 1 after a pause. Two failures stop the task until the next boot, because retrying can't help: a SIM locked with its PUK, and a SIM PIN the SIM refused. A wrong PIN is entered once per boot at most: three wrong PINs lock the SIM, so fix `sim_pin` before rebooting the device again.

The AT layer (`at.rs`: reply parsing and the commands sent; `channel.rs`: the line reader on top of a buffered UART) is generic over the serial port and host-tested against a scripted modem. Reads keep their state in the channel rather than in the future, so a command dropped on a timeout loses no bytes, and input nobody read is discarded before each command so a late reply can't answer the next one.

The `wireless` status LED is lit while the PPP link is up (`FLAG_LTE`).

## Limits

*   **Security over a public network:** the broker is on the internet once the device uses LTE, but MQTT over TLS (`mqtts://`, mTLS) isn't implemented yet. Inbound MQTT commands are already refused until they are authenticated (see [Architecture](architecture.md#channels)), but the published state isn't encrypted.
*   **IPv4 only:** the PDP context is `"IP"`.
*   **Data rate:** PPP at 115200 baud gives about 11 KB/s, plenty for MQTT.

## Sleep and remote wake-up (not implemented)

The car's 12 V budget will need the device to sleep while parked. Carriers put the device behind NAT, so nothing on the internet can open a connection to it. A message can only reach a sleeping device in two ways:

*   **Over a connection it keeps open.** The MQTT session stays up while the ESP light-sleeps (RAM, and so the TCP state, is kept) and the modem sleeps with DTR released (`AT+CSCLK=1`). When data arrives, the modem pulses its ring indicator, which wakes the ESP with a GPIO interrupt. The keepalive must stay below the carrier's NAT timeout (often a few minutes to about 30). Whether the SIM7670G pulses RI for incoming data in PPP mode, and not only for SMS and AT notifications, still has to be checked on the bench.
*   **SMS wake-up.** The ESP deep-sleeps with almost everything off. An SMS makes the modem pulse RI, whatever PPP does. The ESP wakes, reconnects and receives the queued command, which needs a persistent MQTT session (`clean_start` off, QoS 1). It costs a server-side SMS gateway and a few seconds of latency.

What is in place for that: the ring and DTR lines are part of the modem config (on the LilyGO board the ring line is GPIO 3, an RTC GPIO that can wake the ESP from deep sleep), DTR is driven by the firmware, and `lte_task` is organised in phases a sleeping phase can join. GNSS (the SIM7670G has it) would need the AT channel while PPP runs, through the modem's multiplexer (`AT+CMUX`); that is also for later.
