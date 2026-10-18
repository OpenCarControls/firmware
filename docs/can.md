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
Hardware filters are set to accept-all where driver constraints apply (TWAI on ESP). All matching is done in software inside the driver loop against the vehicle-supplied filter list. MCP2515 also programs hardware RX masks/filters from the same list as a first-pass optimisation.

**Known MCP2515 issue (pre-existing):** the MCP2515 only accepts writes to its mask/filter registers in Configuration mode, but `init_mcp2515` and the debug mask switching write them in an operating mode, so they have no effect and the chip effectively accepts all frames (the software filter does the real work). Fixing this naively would break vehicles with more than 6 filters per bus (the chip has 6 filter slots and egmp uses 17), so it needs a fallback to accept-all in that case.

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
*   TWAI entries require `interface = "twai"`, `tx_pin`, and `rx_pin`. The TWAI peripheral is hardcoded as `TWAI0`. Pin mapping matches config field names directly: `rx_pin` is passed as RX and `tx_pin` as TX to `board_esp::init_twai(...)`.

## Read-only mode

**Purpose:** keep the device completely passive on the bus until the vehicle crate has positively identified the connected car. Where the controller supports it, read-only mode is enforced in hardware (listen-only mode): the device sends no frames, no ACKs and no error frames, so even a wrong bitrate cannot disturb the car's bus.

**Flag:** `core-interface` — `static CAN_READ_ONLY: AtomicBool = AtomicBool::new(true)`. Defaults to `true` (locked) at boot so no frame can reach the bus before validation.

**Public API (in `core-interface`):**
*   `pub fn is_can_read_only() -> bool` — vehicle tasks call this to check whether TX is currently blocked.
*   `pub fn set_can_read_only(enabled: bool)` — vehicle tasks call this to unlock (`false`) after validating inbound CAN frames confirm the correct car, or re-lock (`true`) if an error or inconsistent data is detected at any time.

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

### Intended vehicle workflow
1.  Boot: `CAN_READ_ONLY = true`. Vehicle receives CAN frames via `can_rx_task`.
2.  Vehicle validates frame IDs, data ranges, or handshake frames against its expected car profile.
3.  On success: `set_can_read_only(false)` — CAN TX is now live.
4.  On any subsequent error or unexpected data: `set_can_read_only(true)` re-engages the lock.

## E-GMP vehicle state
`EgmpCarState` fields are all `None` until the matching CAN frame has been decoded, and `None` fields are omitted from the published proto, so the app never sees invented values. The debug simulation seeds its own defaults when it is switched on.

`is_driving` is derived from `speed > 0`. The charge port state combines four frames: the door (`CHARGE_PORT_DOOR`), the plug (`CHARGE_INLET`), whether energy is flowing (`CHARGING_STATUS`) and the charger type with the DC sequence stage (`CHARGER_SESSION`). With no cable it is the door state. E-GMP cars try DC first at every plug-in, so the charger type is only taken once it is certain (AC reported, or a DC stage an AC charger never reaches) and kept until unplug; until then the state is `Connected`, which lasts about 7 s on an AC charger. Error, DC negotiating and V2L states aren't decoded. Window states come from `BCM_15_200ms`; the EV6 reports its rear windows as not available there, so those fields stay `None`.

The DBC comments say which signals were checked against CAN logs or a live car. Signals whose mapping is not yet verified (climate target temperature) are only decoded and filtered when the `untested-signals` cargo feature of `egmp-controller` is on; it is off by default. xtask does not currently forward this feature, so enable it by hand in `cars/egmp/Cargo.toml` while verifying a mapping.
