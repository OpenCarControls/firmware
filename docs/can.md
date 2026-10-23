# CAN bus

How CAN frames move between hardware, the board crates and vehicle crates. See [Architecture](architecture.md) for the channel contract in general.

## Shared types and channels
Defined in `core-interface`:

| Channel | Direction | Type |
|---|---|---|
| `CAN_RX_CHANNEL` | board → vehicle | `CanFrame` |
| `CAN_TX_CHANNELS[bus_id]` | vehicle → board | `CanFrame` (one queue per bus, `MAX_CAN_BUSES` = 4) |

**`CanFrame`** carries `{ bus_id: u8, id: embedded_can::Id, data: [u8; 8], dlc: u8 }`. `bus_id` is a 0-based index matching the order of `[[hardware.esp.can_buses]]` / `[[hardware.pc.can_buses]]` entries in `config.toml`.

Vehicles queue frames with `core_interface::send_can_frame(frame).await`, which picks the channel from `frame.bus_id` (and drops the frame if the bus does not exist). Each board driver drains only its own bus's channel, so drivers never inspect or re-queue each other's frames.

**`CanFilter`** carries `{ bus_id: u8, id: embedded_can::Id, mask: u32 }`. Vehicle crates export a `pub static CAN_FILTERS: &[CanFilter]` that xtask passes to the board's init functions. A frame passes if `(frame_id_raw & mask) == (filter_id_raw & mask)`.

## Filter strategy
Hardware filters are set to accept-all where driver constraints apply (TWAI on ESP). All matching is done in software inside the driver loop against the vehicle-supplied filter list. Remote (RTR) frames are dropped before filtering: `CanFrame` has no RTR flag, so they would look like all-zero data frames. MCP2515 also programs hardware RX masks/filters from the same list as a first-pass optimisation.

**Known MCP2515 issue (pre-existing):** the MCP2515 only accepts writes to its mask/filter registers in Configuration mode, but `init_mcp2515` and the logging mask switching write them in an operating mode, so they have no effect and the chip effectively accepts all frames (the software filter does the real work). Fixing this naively would break vehicles with more than 6 filters per bus (the chip has 6 filter slots and egmp uses 33), so it needs a fallback to accept-all in that case. Until then, the build refuses an MCP2515 bus whose vehicle has more than 6 filters for it: the generated `main.rs` asserts `bus_filter_count(..) <= MCP2515_HW_FILTERS` at compile time.

## ESP two-core split

| Core | Tasks |
|---|---|
| Core 1 (`esp_rtos::start_second_core`) | `run_twai_loop`, `run_mcp2515_loop` — all CAN I/O |
| Core 0 (`#[esp_rtos::main]` executor) | BLE/MQTT comms, vehicle logic, `core-interface` plumbing |

## ESP driver modes

### TWAI (built-in)
`init_twai` returns a `TwaiDriver` that just owns the TWAI peripheral and pins (`AnyTwai`/`AnyPin`, all `Send`, so it can cross to core 1). esp-hal only takes the operating mode at construction, so `run_twai_loop` builds a `TwaiConfiguration` from `reborrow()`s of those, calls `.into_async().start()` on the core it runs on (binding the TWAI interrupt there; `esp_hal::Async` itself is `!Send`), and rebuilds it whenever the mode has to change.

*   RX is interrupt-driven (esp-hal ISR → 32-frame queue), which keeps frames from being lost on single-core chips while WiFi/BLE are busy.
*   The session loop sleeps until a frame arrives, a TX frame is queued for this bus, the read-only flag changes, or the next RX stats report is due. TX waits are bounded to 100 ms, RX overruns clear the FIFO, and bus-off rebuilds the controller after 1 s.
*   Every 5 s each TWAI bus logs one `info` line with the frames received, how many matched the vehicle filters, RX errors (with the last error kind), overruns and bus-offs since the previous line. A silent bus logs zeros, so the serial console shows whether the car is sending anything. RX errors with no frames usually mean a wrong bitrate, a CAN FD bus or a wiring fault. MCP2515 buses don't have this log yet.
*   The blocking `nb` path was dropped because esp-hal's blocking `receive()` never releases the FIFO after an overrun, so RX stays dead permanently.

### MCP2515 (SPI)
`MCP2515<ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>>`. RX is interrupt-driven via `int_pin.wait_for_falling_edge().await` (async GPIO, which IS `Send`). `CanSpeed` / `McpSpeed` are re-exported from `board_esp` so the generated `main.rs` can reference them without a direct `mcp2515` dep.

## PC driver
`socket_can_task` (pool of 4) opens a non-blocking `SocketCAN` socket per bus and polls it every 1 ms via Embassy timer. Outbound frames are drained from this bus's `CAN_TX_CHANNELS` entry after each RX sweep. If the interface cannot be opened (e.g. `vcan0` does not exist), the task logs a warning and retries every 5 seconds — it does **not** panic, so MQTT, BLE, and the simulated state heartbeat continue running normally.

## xtask CAN codegen
In `xtask_builder.rs` per board:

*   Reads `[[hardware.<board>.can_buses]]` entries from `config.toml`.
*   For each entry emits peripheral/pin construction code + an `init_*` call into `{CAN_HARDWARE_INIT}`.
*   Emits the corresponding `s.spawn(run_*_loop(...))` call into `{CORE1_TASK_SPAWNS}` (ESP) or `spawner.spawn(socket_can_task(...))` (PC).
*   MCP2515 entries require `interface = "mcp2515"`, `cs_pin`, `clk_pin`, `mosi_pin`, `miso_pin`, `int_pin`, `can_speed`, and `mcp_speed`. The SPI peripheral is auto-assigned by xtask (starting from SPI2).
*   TWAI entries require `interface = "twai"`, `tx_pin`, and `rx_pin`, and take an optional `bitrate` in bit/s (125000, 250000, 500000 (default) or 1000000). The TWAI peripheral is hardcoded as `TWAI0`. Pin mapping matches config field names directly: `rx_pin` is passed as RX and `tx_pin` as TX to `board_esp::init_twai(...)`.

## Bitrate and CAN-FD
A vehicle declares what it expects of each bus in its `Cargo.toml`, one `[[package.metadata.opencar.can_buses]]` entry per bus in bus order:

```toml
[[package.metadata.opencar.can_buses]]
bitrate = 500000   # bit/s
fd = false         # optional; true for a CAN-FD bus
```

xtask compares `bitrate` with the board config (`bitrate` for TWAI, `can_speed` for MCP2515) and fails the build on a mismatch, so a wrong bitrate is caught before it reaches a car. PC/SocketCAN buses aren't checked: their bitrate is set on the Linux interface. A vehicle that declares nothing gets no check.

The TWAI and MCP2515 are classic-CAN controllers: in normal mode they would error-frame every CAN-FD frame. A bus declared `fd = true` therefore **never leaves listen-only mode**, even after `set_can_read_only(false)`: xtask emits `core_interface::declare_can_fd_bus(bus_id)` at boot, and the board drivers use `is_can_bus_read_only(bus_id)` for both the controller mode and the TX drop. Vehicle code that unlocks the other buses needs no special case; frames queued for an FD bus are dropped.

## Read-only mode

**Purpose:** keep the device completely passive on the bus until the vehicle crate has positively identified the connected car. Where the controller supports it, read-only mode is enforced in hardware (listen-only mode): the device sends no frames, no ACKs and no error frames, so even a wrong bitrate cannot disturb the car's bus.

**Flag:** `core-interface` — `static CAN_READ_ONLY: AtomicBool = AtomicBool::new(true)`. Defaults to `true` (locked) at boot so no frame can reach the bus before validation.

**Public API (in `core-interface`):**
*   `pub fn is_can_read_only() -> bool` — vehicle tasks call this to check whether TX is currently blocked. Also `true` while a CAN logging session is active or the car's identity is unconfirmed; both are checked on every read, so a racing unlock can't leave the bus open without a confirmed identity.
*   `pub fn set_can_read_only(enabled: bool) -> bool` — vehicle tasks call this to unlock (`false`) after validating inbound CAN frames confirm the correct car, or re-lock (`true`) if an error or inconsistent data is detected at any time. Unlocking is **refused** (the bus stays read-only and the call returns `true`) until the car's identity has been confirmed, see below. Returns the resulting state.
*   `confirm_car_identity()` / `revoke_car_identity()` / `is_car_identity_confirmed()` — the identity gate. `revoke_car_identity()` also re-locks the bus.
*   `vin_matches(vin, prefixes)` — checks a VIN the car reported against the VINs a vehicle supports (well-formed 17 characters, then a prefix match: 3 characters for a manufacturer, longer for a model).

### Enforcement
Two layers:

1.  **Hardware listen-only (ESP):** the board CAN loops are woken by `can_read_only_receiver()` (a `Watch` raised by `set_can_read_only`) and re-check `is_can_read_only()` to keep the controller's mode in sync. The MCP2515 loop additionally wakes every 10 ms.
    *   TWAI: `run_twai_loop` rebuilds the driver in `TwaiMode::ListenOnly` / `TwaiMode::Normal` (esp-hal also applies the ESP32/S3/C3 errata workaround that forces the controller error-passive in listen-only mode). The rebuild resets the controller, so frames arriving during the switch are lost.
    *   MCP2515: `init_mcp2515` leaves the chip in `OpMode::ListenOnly`, and `run_mcp2515_loop` calls `set_mode(ListenOnly | Normal)` on each change.
    *   Neither controller is ever in normal mode before its loop has checked the flag.
2.  **Software TX drop (all boards):** the board TX drain loops discard every frame addressed to their `bus_id` while the flag is set, and also while the hardware mode has not yet caught up with a flip of the flag. Vehicle code is free to push frames to the TX channels at any time. Sites:
    *   `board-pc` — `socket_can_task` TX arm (before `socket.write_frame()`)
    *   `board-esp` — `run_twai_session` TX arm (before `twai.transmit_async()`)
    *   `board-esp` — `run_mcp2515_loop` TX arm (before `driver.send_message()`)

**PC / SocketCAN:** only the software drop applies. Listen-only is a property of the Linux CAN interface, configured with root privileges while the link is down (`ip link set can0 type can bitrate 500000 listen-only on`), and `vcan` does not support it. Configure it on the interface if a PC is attached to a real car.

### Car identity
On every boot the device is passive until the vehicle has checked that it is plugged into a car it supports. `core-interface` enforces this: `set_can_read_only(false)` does nothing until the vehicle calls `confirm_car_identity()`, which starts unset at every boot. The vehicle decides how to confirm, from what the car itself reports; the VIN is the intended source (`vin_matches` with the VIN prefixes the vehicle supports). If the identity later changes (a different VIN, inconsistent data) the vehicle calls `revoke_car_identity()`, which also re-locks the bus.

E-GMP does not confirm an identity yet, so it never unlocks the bus. The VIN has to come from a passive broadcast: reading it with a diagnostic request would need a transmit, which read-only mode forbids. Which E-GMP message carries it is still to be found in a CAN log.

### Intended vehicle workflow
1.  Boot: `CAN_READ_ONLY = true`, identity unconfirmed. Vehicle receives CAN frames via `can_rx_task`.
2.  Vehicle reads the car's VIN from the frames and checks it with `vin_matches`; it also validates frame IDs, data ranges, or handshake frames against its expected car profile.
3.  On success: `confirm_car_identity()`, then `set_can_read_only(false)` — CAN TX is now live.
4.  On any subsequent error or unexpected data: `revoke_car_identity()` re-engages the lock.

## SavvyCAN logging

For reverse engineering, the firmware can stream every frame on every bus to [SavvyCAN](https://www.savvycan.com/) over WiFi, using its GVRET network protocol. It is receive-only.

**Enable it** in `config.toml` with `[network.can_logger] enabled = true` (default off; `port` defaults to 23). The ESP board needs WiFi enabled too. In SavvyCAN add a network connection of type GVRET with the device's IP and that port. The port has no authentication and is open to the whole network, which is why it is off by default.

**What it does while a client is connected:**
*   The buses are forced read-only, whatever the vehicle asks: `is_can_read_only()` is true and `set_can_read_only(false)` is ignored. The buses are also relocked when the session starts and stay locked afterwards, so the vehicle has to validate the car again before it can transmit.
*   The board drivers stop forwarding frames to the vehicle (`CAN_RX_CHANNEL`), so the vehicle tasks are idle. BLE and MQTT keep working.
*   Frames SavvyCAN asks to transmit are parsed and discarded.
*   The MCP2515 hardware masks are switched to accept-all so the logger sees every frame.

Logging stops as soon as the client disconnects, errors, or stops reading for 5 s. On ESP, TCP keep-alive notices a client that vanished. One client at a time; a second connection waits until the first is gone.

**Where the frames go:** the drivers call `core_interface::log_can_frame` for every received frame, before vehicle filtering. It uses `try_send` on the dedicated `CAN_LOG_CHANNEL` (256 frames), so a slow client drops frames (counted and logged when the session ends) and never stalls the CAN loop or the BLE/MQTT/vehicle queues. Frames on a bus the board doesn't have are ignored.

**Protocol:** `core-interface/src/can_log.rs` holds the parser, the encoder and the session loop (generic over `embedded_io_async` streams); each board only provides the TCP listener (`can_logger_task`). Supported commands are binary-mode frames, time sync, keep-alive, number of buses, bus parameters (always listen-only, speed reported as 0) and device info. The bitrate is set at build time, so SavvyCAN's bus setup is accepted and ignored.

## E-GMP vehicle state
`EgmpCarState` fields are all `None` until the matching CAN frame has been decoded, and `None` fields are omitted from the published proto, so the app never sees invented values. The debug simulation seeds its own defaults when it is switched on.

`gear` comes from `DRIVE_MODE_STATUS`, which reads Park with the car off, asleep or charging. `power_state` combines the BCM's accessory and ignition inputs (`BCM_02_200ms`: off, accessory, on) with the ready bit in `DRIVE_MODE_STATUS`, since the ignition inputs read the same when on and ready. `is_driving` is ready and not in Park, so it stays true while stopped in traffic; the speed alone drops to 0 at every stop.

The charge port state combines four frames: the door (`CHARGE_PORT_DOOR`), the plug (`CHARGE_INLET`), whether energy is flowing (`CHARGING_STATUS`) and the charger type with the DC sequence stage (`CHARGER_SESSION`). With no cable it is the door state. E-GMP cars try DC first at every plug-in, so the charger type is only taken once it is certain (AC reported, or a DC stage an AC charger never reaches) and kept until unplug; until then the state is `Connected`, which lasts about 7 s on an AC charger. On DC, stages 2 to 6 without energy flowing (the handshake with the charger, 20 to 60 s) are `DcNegotiating`; `DcConnected` is the cable still in after a session ends. Error and V2L states aren't decoded.

`battery_voltage` and `instantaneous_power_watt` come from the high-voltage battery voltage (`BATTERY_INFO_567`) and current (`BATTERY_INFO_764`), which arrive in separate messages; the power is published once both are known. The car reports the current positive when discharging, so the power is negated to match the proto (positive when charging). `power_flow_watt` comes from `CHARGING_POWER`, which is only set on DC; while AC charging it is the battery power instead (what reaches the battery, a little under what the charger supplies), clamped at 0 since negative means V2L.

`battery_range_km` is the range the cluster shows (`CLUSTER_RANGE`), which is for the active drive mode only; the car doesn't send the other modes' ranges. Range, odometer and speed stay in km and km/h when the car displays miles; `distance_unit` says how the car displays them (`CLUSTER_DISPLAY_UNITS`). `time_remaining_minutes` is the time to the charge limit for the charge in progress, AC or DC (`CHARGING_TIME`, 0 when not charging). `time_to_80_percent_minutes` comes from `CHARGE_TIME_ESTIMATE`, the car's DC fast-charge estimate, which is sent all the time, so it is only taken while a DC charger is plugged in. The charge limits come from `CHARGE_LIMITS`.

Temperatures are always published in Celsius, with `temperature_unit` saying how the car displays them. While it displays Fahrenheit, `AMBIENT_TEMPERATURE` reads Not_Available and the outside temperature comes from `ODOMETER` in Fahrenheit instead, converted back. `indoor_temperature` is the climate's cabin sensor (`CABIN_TEMPERATURE`). Window states come from `BCM_15_200ms`, which gives each window's position in four steps (closed, then three open steps); the front windows publish that step as `*_window_position`. The EV6 reports its rear windows as not available there, so those fields stay `None`.

Only signals checked on a real car are decoded; the DBC comments say how each was checked (CAN logs with timed notes, or live). Climate on/off, windshield defog and the target temperature come from the climate controller's `DATC_*` messages and `CLIMATE_TEMPERATURE`, the steering wheel heater from `ICU_08_200ms`, and the seat heating levels from `DRIVER_SEAT_HEATING` and `PASSENGER_SEAT_HEATING`. Values that mean "not available" (car off) leave the last known state, so the target temperature doesn't drop to the OFF value.

`preconditioning_state` comes from `BMS_PRECOND_STATUS`: Preparing from the request, Heating once the battery heater switches on (when the cluster shows the battery icon, about a minute later but not at a fixed delay, so don't count down to it). The value the car sends while off, asleep or charging maps to Unspecified, not Off.

The battery minimum and maximum temperatures and the 12 V battery voltage (`auxiliary_battery_voltage`) come from `BATTERY_INFO_764`, with the high-voltage current. The 12 V voltage and the tire pressures are tracked but not published yet: they don't fit one BLE message until state updates can be fragmented (see [BLE](ble.md#message-size)). The 12 V voltage reads 0 for a frame while the car switches off, which keeps the last value.

`regen_mode` and `regen_level` are tracked in `EgmpCarState` but not published (they aren't in the proto yet; they are kept for a later feature that sets the regen). `regen_mode` combines the Auto bit (`REGEN_STATUS`) with the i-Pedal bit (`I_PEDAL_STATUS`), which arrive in separate messages. `regen_level` is the level from `REGEN_STATUS`, 0 to 4 (4 is the maximum, always in i-Pedal). In Auto the car moves it smoothly between steps, so it is rounded to the nearest one; the cluster numbers the Auto steps one higher (1 to 5). It reads Not_Available with the car off and while smart cruise drives the car, which keeps the last level.

The tire pressures come from `TIRE_PRESSURES` in whole psi, converted to kPa; that is with the cluster set to psi, and the kPa setting hasn't been captured yet. They read Not_Available until the sensors report, a minute or two into a drive, which keeps the last pressures. Front left and rear left were checked against the cluster; front right and rear right are assumed from the usual order, the one exception to decoding only checked signals, until a capture sets one of them apart.
