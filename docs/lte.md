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

## Why PPP and not the modem's MQTT client

Most LTE modems have their own MQTT client, driven by AT commands (SIMCom `AT+CMQTT*`, Quectel `AT+QMT*`, u-blox `AT+UMQTT*`). The firmware uses PPP instead:

*   **One driver for every modem.** The PPP dial (`ATD*99#`) is standard across vendors, while each vendor's MQTT commands differ. Supporting another modem is then mostly its power-on timings.
*   **One MQTT implementation.** WiFi and LTE share `mqtt_driver_task` and its resilience rules (see [Architecture](architecture.md#mqtt-driver-resilience)), and everything above the network stack (DNS, TLS once it exists) works the same on both.
*   **TLS stays on the ESP.** WiFi builds need TLS on the ESP anyway, so the modem's TLS would only save RAM on LTE builds. It would also keep the client's private key on the modem, readable by whoever can reach its UART, and tie the TLS versions and cipher suites to the modem's firmware.
*   **No modem MQTT limits:** these clients speak MQTT 3.1.1 (the firmware uses v5), cap the payload size, and return binary payloads inside AT notifications.

What the modem's client would give: the ESP could deep-sleep while the modem keeps the session alive, and the AT channel would stay free while connected. If the sleep experiments below show that remote wake-up only works reliably that way, the client can be added as another MQTT driver for that modem: it would read and write the same `MQTT_RX_CHANNEL` / `MQTT_TX_CHANNEL`, and the power-on, SIM and registration steps would stay as they are.

## Sleep and remote wake-up (not implemented)

The car's 12 V budget will need the device to sleep while parked. Carriers put the device behind NAT, so nothing on the internet can open a connection to it. A message can only reach a sleeping device in two ways:

*   **Over a connection it keeps open.** The MQTT session stays up while the ESP light-sleeps (RAM, and so the TCP state, is kept) and the modem sleeps with DTR released (`AT+CSCLK=1`). When data arrives, the modem pulses its ring indicator, which wakes the ESP with a GPIO interrupt. The keepalive must stay below the carrier's NAT timeout (often a few minutes to about 30).
*   **SMS wake-up.** The ESP deep-sleeps and an SMS makes the modem pulse RI. This is ruled out: many IoT SIMs are data-only.

The first way is open questions with PPP. On SIMCom modems DTR controls both sleep (`AT+CSCLK`) and data mode (`AT&D`), so releasing DTR to let the modem sleep can end the PPP session; keeping both needs `AT&D0`, or the modem's multiplexer (`AT+CMUX`) to keep an AT channel next to PPP. It is also unknown whether the SIM7670G pulses RI for incoming PPP data, and not only for SMS and AT notifications. With the modem's MQTT client, holding back the notification of an incoming message and pulsing RI while asleep is the documented behaviour. Two bench experiments, from an AT console with RI (GPIO 3 on the LilyGO board) watched, decide between them:

1.  **PPP:** with a data session up and `AT&D0`, release DTR. Does the modem sleep, keep the session, and pulse RI on incoming data?
2.  **Modem MQTT:** connect and subscribe with `AT+CMQTT*`, set `AT+CSCLK=1`, release DTR and publish to the topic from the broker. Does RI pulse, and does the message arrive once DTR is asserted again?

If only the second works, the LTE MQTT driver for that modem moves to its MQTT client (see above).

Either way, what is in place: the ring and DTR lines are part of the modem config (on the LilyGO board the ring line is GPIO 3, an RTC GPIO that can wake the ESP from deep sleep), DTR is driven by the firmware, and `lte_task` is organised in phases a sleeping phase can join. GNSS (the SIM7670G has it) would need the AT channel while PPP runs, through the modem's multiplexer (`AT+CMUX`); that is also for later.
