# CAN bus

How CAN frames move between hardware, the board crates and vehicle crates. See [Architecture](architecture.md) for the channel contract in general.

## Shared types and channels
Defined in `core-interface`:

| Channel | Direction | Type |
|---|---|---|
| `CAN_RX_CHANNEL` | board → vehicle | `CanFrame` |
| `CAN_TX_CHANNELS[bus_id]` | vehicle → board | `CanTxFrame` (a `CanFrame` with its [TX class](#tx-classes); one queue per bus, `MAX_CAN_BUSES` = 4) |

**`CanFrame`** carries `{ bus_id: u8, id: embedded_can::Id, data: [u8; 8], dlc: u8 }`. `bus_id` is a 0-based index matching the order of `[[hardware.esp.can_buses]]` / `[[hardware.pc.can_buses]]` entries in `config.toml`.

Vehicles queue frames with `core_interface::send_can_frame(CanTxFrame::platform(frame)).await` (or `CanTxFrame::command(..)`, see [TX classes](#tx-classes)), which picks the channel from the frame's `bus_id` (and drops the frame if the bus does not exist). Each board driver drains only its own bus's channel, so drivers never inspect or re-queue each other's frames.

**`CanFilter`** carries `{ bus_id: u8, id: embedded_can::Id, mask: u32 }`. Vehicle crates export a `pub static CAN_FILTERS: &[CanFilter]` that xtask passes to the board's init functions. A frame passes if `(frame_id_raw & mask) == (filter_id_raw & mask)`.

## Filter strategy
Hardware filters are set to accept-all where driver constraints apply (TWAI on ESP). All matching is done in software inside the driver loop against the vehicle-supplied filter list. Remote (RTR) frames are dropped before filtering: `CanFrame` has no RTR flag, so they would look like all-zero data frames. MCP2515 also programs hardware RX masks/filters from the same list as a first-pass optimisation. The two RX buffer masks are the AND of the masks of the filters in each buffer (a bit must match only if every filter in the buffer needs it), so the hardware never rejects a frame the software filter accepts. They fall back to accept-all when any filter for the bus is extended (the masks are written as standard IDs) or there are more filters than the chip's 6 slots (`compute_mcp_masks`).

**Known MCP2515 issue (pre-existing):** the MCP2515 only accepts writes to its mask/filter registers in Configuration mode, but `init_mcp2515` and the logging mask switching write them in an operating mode, so they have no effect and the chip effectively accepts all frames (the software filter does the real work). Fixing this naively would break vehicles with more than 6 filters per bus (the chip has 6 filter slots and egmp uses far more), so it needs a fallback to accept-all in that case. Until then, the build refuses an MCP2515 bus whose vehicle has more than 6 filters for it: the generated `main.rs` asserts `bus_filter_count(..) <= MCP2515_HW_FILTERS` at compile time.

## ESP two-core split

| Core | Tasks |
|---|---|
| Core 1 (`esp_rtos::start_second_core`) | `run_twai_loop`, `run_mcp2515_loop` — all CAN I/O |
| Core 0 (`#[esp_rtos::main]` executor) | BLE/MQTT comms, vehicle logic, `core-interface` plumbing |

## ESP driver modes

### TWAI (built-in)
`init_twai` returns a `TwaiDriver` that just owns the TWAI peripheral and pins (`AnyTwai`/`AnyPin`, all `Send`, so it can cross to core 1). esp-hal only takes the operating mode at construction, so `run_twai_loop` builds a `TwaiConfiguration` from `reborrow()`s of those, calls `.into_async().start()` on the core it runs on (binding the TWAI interrupt there; `esp_hal::Async` itself is `!Send`), and rebuilds it whenever the mode has to change.

*   RX is interrupt-driven (esp-hal ISR → 32-frame queue), which keeps frames from being lost on single-core chips while WiFi/BLE are busy.
*   The session loop sleeps until a frame arrives, a TX frame is queued for this bus, the read-only flag changes, or the next RX stats report is due. TX waits are bounded to 100 ms, RX overruns clear the FIFO, and bus-off rebuilds the controller after 1 s. A bus-off also relocks every bus (`set_can_read_only(true)`), so the vehicle has to unlock again (see [Read-write policy](#read-write-policy)).
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

**Purpose:** keep the device completely passive on the bus until the vehicle crate has positively identified the connected car and the [read-write policy](#read-write-policy) allows it. Where the controller supports it, read-only mode is enforced in hardware (listen-only mode): the device sends no frames, no ACKs and no error frames, so even a wrong bitrate cannot disturb the car's bus. The one exception before the car is identified is the [identity probe](#identity-probe).

**Flag:** `core-interface` — `static CAN_READ_ONLY: AtomicBool = AtomicBool::new(true)`. Defaults to `true` (locked) at boot so no frame can reach the bus before validation.

**Public API (in `core-interface`):**
*   `pub fn is_can_read_only() -> bool` — vehicle tasks call this to check whether TX is currently blocked. Also `true` while a CAN logging session is active or any condition of the [read-write policy](#read-write-policy) is unmet (car identity, owner approval, dev lock); all of it is checked on every read, so a racing unlock can't leave the bus open with a condition unmet.
*   `pub fn set_can_read_only(enabled: bool) -> bool` — vehicle tasks call this to unlock (`false`) after validating inbound CAN frames confirm the correct car, or re-lock (`true`) if an error or inconsistent data is detected at any time. Unlocking is **refused** (the latch stays set) until the car's identity has been confirmed in both stages, see below. An accepted unlock still waits for the owner's approval and the dev lock. Returns the resulting state (`is_can_read_only()`).
*   `confirm_car_identity()` / `revoke_car_identity()` / `is_car_identity_confirmed()` — the identity gate, first stage (the platform). `revoke_car_identity()` forgets both stages and re-locks the bus.
*   `confirm_car_fingerprint(fingerprint)` / `is_car_fingerprint_confirmed()` — the second stage (the exact car), see below.
*   `vin_matches(vin, prefixes)` — checks a VIN the car reported against the VINs a vehicle supports (well-formed 17 characters, then a prefix match: 3 characters for a manufacturer, longer for a model).

### Enforcement
Two layers:

1.  **Hardware listen-only (ESP):** the board CAN loops are woken by `can_read_only_receiver()` (a `Watch` raised by `set_can_read_only` and by an identity probe window opening or closing) and re-check `is_can_bus_listen_only(bus_id)` to keep the controller's mode in sync: listen-only while the bus is read-only (`is_can_bus_read_only(bus_id)`), unless a probe window is open on it. The MCP2515 loop additionally wakes every 10 ms.
    *   TWAI: `run_twai_loop` rebuilds the driver in `TwaiMode::ListenOnly` / `TwaiMode::Normal` (esp-hal also applies the ESP32/S3/C3 errata workaround that forces the controller error-passive in listen-only mode). The rebuild resets the controller, so frames arriving during the switch are lost.
    *   MCP2515: `init_mcp2515` leaves the chip in `OpMode::ListenOnly`, and `run_mcp2515_loop` calls `set_mode(ListenOnly | Normal)` on each change.
    *   Neither controller is ever in normal mode before its loop has checked the flag.
2.  **Software TX drop (all boards):** the board TX drain loops ask `allow_can_tx(bus_id, &frame)` for every frame and discard it unless the bus is read-write or the frame is one an open probe window allows (and, for a command frame, its phone is still validated, see [TX classes](#tx-classes)), and also while the hardware mode has not yet caught up. Vehicle code is free to push frames to the TX channels at any time. Sites:
    *   `board-pc` — `socket_can_task` TX arm (before `socket.write_frame()`)
    *   `board-esp` — `run_twai_session` TX arm (before `twai.transmit_async()`)
    *   `board-esp` — `run_mcp2515_loop` TX arm (before `driver.send_message()`)

**PC / SocketCAN:** only the software drop applies. Listen-only is a property of the Linux CAN interface, configured with root privileges while the link is down (`ip link set can0 type can bitrate 500000 listen-only on`), and `vcan` does not support it. Configure it on the interface if a PC is attached to a real car.

### Car identity
On every boot the device is passive until the vehicle has checked that it is plugged into a car it supports, and which car. `core-interface` enforces this in two stages, both unset at every boot, and `set_can_read_only(false)` does nothing until both are confirmed:

1.  **Platform.** `confirm_car_identity()`: the vehicle recognised its own platform on the bus (it fingerprints the bus, e.g. "this is an E-GMP bus"), so the firmware matches the car. Commands written for one make never run on another.
2.  **The exact car.** `confirm_car_fingerprint(fingerprint)`, only accepted after the first stage: bytes that are stable for this one car and differ between cars of the same model. The VIN where the bus carries it (`vin_matches` checks it against the VIN prefixes the vehicle supports), otherwise other identifiers unique to the car (ECU serial numbers, for example), never values that change like the odometer. The owner's approval is bound to it, so a configuration made for one car never runs on another, even the same model.

The vehicle decides how to confirm both, from what the car itself reports; `core-interface` never interprets the fingerprint. If the identity later changes (a different VIN, inconsistent data) the vehicle calls `revoke_car_identity()`, which forgets both stages and re-locks the bus. If a repair changes what the fingerprint is made of (a replaced ECU), the car reads as a different car and the owner has to approve it again; that fails safe.

**E-GMP** confirms the platform passively (`cars/egmp/src/platform.rs`). Hyundai, Kia and Genesis protect many frames with a CRC-16 that also covers the frame ID (opendbc's `hkg_can_fd_checksum`), so a frame from another make passes it by chance about once in 65,536. The platform is confirmed once three of the checksummed IDs it knows (the climate group, DISPLAY_SETTINGS_472 and its neighbours) have each passed twice, and the two EV-only frames (BATTERY_INFO_764 and REGEN_STATUS) have been seen. Frames that are all zeros or carry `FF FF` as the checksum, which senders send before they have a value, are ignored. The check keeps running on every frame: three failures in a row on one of those IDs revoke the identity, and it then needs the full evidence again. It recognises a recent Hyundai/Kia/Genesis EV, not E-GMP specifically; those cars share these IDs, and the owner still approves the exact car. Cars whose M-CAN carries these frames split into 29-bit pieces instead (seen on an Ioniq 6 and a GV60, probably with a newer head unit) are not supported and don't pass. None of the checksummed frames are sent with the car off and only the doors used, so the platform is confirmed once the car powers up. The bus going to sleep doesn't revoke it: the device can't move to another car without losing power, and a reboot resets the identity.

E-GMP then identifies the exact car with the head unit's serial number (`cars/egmp/src/identity.rs`), on every car: North American cars don't broadcast the VIN on M-CAN. It reads it through the [identity probe](#identity-probe), UDS `22 F18C` on 0x780 with the answer on 0x788. The head unit of an EV6 answers in the default session whenever the bus is awake (Ready, ACC, or off with the bus still awake). The two frames sent are the request and the flow control, `03 22 F1 8C 00 00 00 00` and `30 00 00 00 00 00 00 00`. The identification is driven by received frames, so it only moves on while the bus is busy: a request sent on a sleeping EV6 bus woke the car. An attempt that gets no answer or a refusal closes its window and is retried after 2, 5, 15, then 30 s, until `core-interface` refuses more attempts (`MAX_PROBE_ATTEMPTS` per boot); the bus then stays read-only until a reboot. The fingerprint is `hu-serial:` followed by the serial without its trailing padding (`00`, spaces or `FF`), so it can never match a VIN-based one. E-GMP then calls `set_can_read_only(false)`; the bus leaves read-only mode once the owner has approved this car (and, in a development build, the dev lock is unlocked). If the identity is revoked later, the serial is read again once the platform matches again. A replaced head unit makes it a different car, which the owner approves again.

virtual-car has no real car to recognise: it identifies a simulated car with a fixed fingerprint at boot (`identify_simulated_car()`) and asks to unlock, so its commands work once the owner has approved it (and, in a development build, unlocked the dev lock) and the phone is validated. Every virtual-car device is the same car to the policy. It is for development only; don't run it on a device plugged into a real car.

virtual-car's simulated [phone validation](ble.md#phone-trust) methods, on bus 0 (`vcan0` on PC):

*   `virtual_car.cluster_code` (code entry, needs the approval): while the session runs, the code is sent every second on 0x500 as a platform frame (6 ASCII digits, then two zeros), as a car shows it on the cluster, and logged once (`the cluster shows the validation code ...`) for a PC without `vcan0`.
*   `virtual_car.button` (in-car action): a 0x400 frame with byte 0 = `01` (`cansend vcan0 400#01`) presses the button.

Its door lock sends a command frame on 0x310 (byte 0 = `01` to lock), so `candump vcan0` shows that command frames only go out for a validated phone.

### Identity probe
Some cars only give away what identifies the exact car when asked with a diagnostic request (E-GMP: the head unit's serial number, `22 F18C` on 0x780). The identity probe is the only way to transmit before the car is identified, and it is kept as narrow as possible:

*   **Declared at build time.** The vehicle declares an `IdentityProbe`: one classic-CAN bus and the exact frames it may send, each with its ID, 11-bit (`standard_id`) or 29-bit (`extended_id`), at most `MAX_PROBE_FRAMES`. E-GMP's is `uds_single_frame(&[0x22, 0xF1, 0x8C], 0x00)` and `iso_tp_flow_control(0x00)`, both on 0x780. A request broadcast to every ECU (OBD's 0x7DF) declares its flow control on the answering ECU's own ID instead.
*   **A window per attempt.** `open_identity_probe(&PROBE)` returns a `ProbeWindow` and is refused unless the platform is confirmed and the exact car isn't, no logging session runs, the bus received a frame in the last `BUS_AWAKE_MS` (a frame sent on a sleeping E-GMP bus woke the car), no other window is open, and fewer than `MAX_PROBE_ATTEMPTS` windows were opened since boot. CAN-FD buses are refused.
*   **Exact frames, once each.** While the window is open, the bus's controller leaves listen-only mode (it then also acknowledges the car's frames, as any node does), and `allow_can_tx` lets each declared frame through at most once, on its own ID, byte for byte, with 8 data bytes. Everything else is still dropped, and `is_can_read_only()` stays `true`.
*   **Short.** The window closes when it is dropped, after `PROBE_WINDOW_MS`, when the exact car is confirmed, when a logging session starts, or on any relock (`set_can_read_only(true)`, a bus-off). The controller then returns to listen-only mode.

The vehicle waits `PROBE_SETTLE_MS` after opening the window, for the controller to leave listen-only, then sends the request. `UdsRequest` (`isotp.rs`) is the matching ISO-TP client, for one single-frame request (UDS `22`, OBD `09`, any service whose positive answer is the service byte plus 0x40 followed by the request's parameters) with normal addressing on classic CAN. It does no I/O: the vehicle feeds it the frames from the response ID, sends its flow control when it returns `SendFlowControl`, and gives up after `timeout_ms()` without a frame. It handles single and multi-frame answers, consecutive-frame order, negative responses and "response pending" (`7F xx 78`), and pads the frames it builds with the byte the vehicle chooses (some ECUs expect `AA` or `55` instead of `00`). Extended addressing, ISO-TP over CAN-FD and multi-frame requests aren't supported.

### Intended vehicle workflow
1.  Boot: `CAN_READ_ONLY = true`, identity unconfirmed, dev lock locked; the board restores the owner's approval from storage. Vehicle receives CAN frames via `can_rx_task`.
2.  Vehicle fingerprints the bus (frame IDs, data ranges, handshake frames against its expected car profile) and calls `confirm_car_identity()`.
3.  Vehicle identifies the exact car (the VIN, checked with `vin_matches`, or other unique identifiers, read passively or through the [identity probe](#identity-probe)) and calls `confirm_car_fingerprint(..)`, then `set_can_read_only(false)`. CAN TX is live once the owner has approved this car and, in a development build, the dev lock is unlocked; the vehicle doesn't need to ask again when that happens.
4.  On any subsequent error or unexpected data: `revoke_car_identity()` re-engages the lock.

## TX classes
Every frame a vehicle sends has a class, and the TX channels only carry `CanTxFrame`s, which can only be built with one of two constructors (`can_tx.rs`), so the class of every frame is visible where it is built:

*   **Platform frames** (`CanTxFrame::platform(frame)`): the platform's own traffic that doesn't act on the car, such as the identity request or text on the cluster. They need read-write mode (the [read-write policy](#read-write-policy)), or an open [identity probe](#identity-probe) window for the probe's exact frames. Review every `CanTxFrame::platform` call to check the frame really doesn't act on the car.
*   **Command frames** (`CanTxFrame::command(&authorization, frame)`): part of a vehicle command (lock the doors, open the trunk). They need read-write mode and the phone that sent the command to be [validated](ble.md#phone-trust) for this car. The `CommandAuthorization` comes with the command's `InboundCommand`, and only the dispatcher creates one, for a command from a validated phone. `allow_can_tx` checks the phone's validation again when the frame is sent, so a frame queued before the validation ended is dropped. Command frames never go through a probe window.

`CommandAuthorization::for_tests` lets vehicle crate tests build one; it only exists with the `test-utils` feature, which only dev-dependencies enable.

## Read-write policy

The device may transmit on the car's buses (read-write mode) only while **all** of these hold. `is_can_read_only()` derives the answer from them every time it is read (`can_access.rs`), so an unmet condition always reads as read-only:

1.  **The car is identified**: both stages of [Car identity](#car-identity), platform and exact car.
2.  **The owner approved read-write mode for this car.** Given once, after setup, from a bonded phone over BLE (`SetCanReadWriteApprovalCommand`), after the app has shown the risk; once a phone is validated for this car, only a validated phone can change it (see [Phone trust](ble.md#phone-trust)). Refused while the car isn't identified, since it binds to the identified car.
3.  **Development builds only: the dev lock is unlocked** (`SetCanDevWriteUnlockCommand`, from a bonded phone over BLE). It is locked at every boot and never stored, so a developer unlocks it deliberately each time. Release builds (`cargo xtask build --production`, which only the release workflow passes) have no dev lock and refuse the command; every other build is a development build, so a missing flag only adds a condition.

On top of that, the vehicle must have asked to unlock (`set_can_read_only(false)`), and no SavvyCAN logging session may be active.

Read-write mode only lets the platform's own frames out. A vehicle command's frames also need the sending phone to be validated in the car ([TX classes](#tx-classes)), so the approval alone never lets anything act on the car.

**The approval** is stored as `SHA-256(domain ‖ platform_id ‖ fingerprint)` (`CarDigest`), so no VIN is stored, and it only applies while the identified car has the same digest. It survives reboots and pairing, removing or clearing phones; phones connecting or disconnecting change nothing. It stops applying when the device is moved to another car (the app shows "approved for another car" and the owner can approve this one, which replaces it), and it is cleared by revoking it from the app or by erasing the flash. Firmware for another platform hashes a different `platform_id`, so it never inherits an approval either. The ESP keeps it in flash next to the BLE bonds (see [BLE](ble.md#persistence-power-cycle)); the PC board in the file `[hardware.pc] can_rw_approval_file`. Until the board has restored it at boot, the device counts as not approved. `core-interface` hands every change to the board with `can_read_write_approval_to_store()`.

**Who can change it:** only a bonded phone, over BLE, and once a phone is validated for the identified car only a validated one (`REJECTED_NOT_VALIDATED` otherwise). The dispatcher checks the transport-stamped `source_device_id` against the paired-phone registry (a phone that isn't bonded can still write while the pairing window is open) and answers with a `CommandResponse` (failed with the reason when refused). MQTT drops every `SystemCommand`, so it can never approve, revoke or unlock. Approving a car other than the one the validations were made in erases every validation.

**What relocks the bus:**

| Event | Effect |
|---|---|
| Lost identity (`revoke_car_identity()`) | Relocked; the vehicle has to identify the car and unlock again |
| Bus fault (TWAI bus-off) | Relocked (`set_can_read_only(true)`); the vehicle has to unlock again |
| SavvyCAN session | Forced read-only while connected, and relocked afterwards |
| Approval revoked, or the identified car isn't the approved one | Read-only at once; read-write again as soon as the approval matches |
| Dev lock locked | Read-only at once; read-write again as soon as it is unlocked |
| Reboot | Latch, identity and dev lock reset; the approval is kept |

The approval and the dev lock are read-time conditions rather than a relock of the latch: the owner can lock and unlock without the vehicle having to validate the car again, and withdrawing either still closes the bus immediately.

**What the app sees:** `SystemState.can_access` (`CanAccessState`): platform confirmed, car identified, the approval compared with this car (`NotApproved`, `Approved`, `OtherCar`, `AwaitingCar`), whether this build has the dev lock and whether it is unlocked, logging, and the effective `read_only`. It is BLE only, sent in a `StateUpdate` of its own (no vehicle state, so the vehicle's message size is unaffected) by `publish_can_access_task` whenever it changes (checked at least once a second), every `CAN_ACCESS_REPEAT_MS` (5 s) while it doesn't, and again when the app subscribes. The repeat covers a message lost as the app subscribes and a missed change notification, so the app never relies on one delivery. The same update carries the connected phone's `SystemState.phone_access` (its trust level and validation session, see [Phone trust](ble.md#phone-trust)), which also triggers it when it changes. The fingerprint and the VIN are never sent.

**Vehicle commands while read-only:** `basic_command_bytes` and `advanced_command_bytes` are refused by `core-interface`'s dispatcher (BLE, and MQTT once it accepts commands) with a failed `CommandResponse` while `is_can_read_only()` holds, before they reach the vehicle. The message gives the first unmet condition in words (`can_read_only_reason()`: logging, platform not recognised, car not identified, not approved or approved for another car, dev lock locked, or the vehicle not having enabled control). So every vehicle, the simulator included, reports a read-only device truthfully instead of claiming a command it never sent worked. A read-write device then refuses them from a phone that isn't validated for this car (`REJECTED_NOT_VALIDATED`, see [Phone trust](ble.md#phone-trust)). System commands (the approval, the dev lock, phone management) are not gated. The vehicle's own traffic, such as an identification request, doesn't go through app commands and isn't affected.

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

The tire pressures come from `TIRE_PRESSURES`, converted to kPa. The car sends them in the unit set for the cluster (whole psi, kPa / 5 or bar x 10), which `TIRE_PRESSURE_UNIT` gives in a separate message; until it has arrived the pressures aren't decoded. When the unit changes, the two messages can arrive in either order, so one frame (200 ms) may be converted with the old unit. They read Not_Available until the sensors report, a minute or two into a drive, which keeps the last pressures. Front left and rear left were checked against the cluster; front right and rear right are assumed from the usual order, the one exception to decoding only checked signals, until a capture sets one of them apart.

## E-GMP cluster code
E-GMP validates a phone with a code on the instrument cluster, `egmp.cluster_code` (`cars/egmp/src/cluster_code.rs`, see [Phone trust](ble.md#phone-trust)). It uses the popup the cluster shows for a new track, as [wicant-i-precondition](https://github.com/L1Z3/wicant-i-precondition) does. Behaviour checked on an EV6 with a tap on M-CAN (no bridge):

*   **The popup.** MEDIA_STATUS (0x4CE) with byte 1 at `0x11` opens it for 4 s: 1 s of the source's name ("Android Auto", "FM 99.5"), then 3 s of the text the head unit sends over ISO-TP. Real popups are the same. The device sends copies of the head unit's own frame, three of them per trigger, then the text about 5 ms later.
*   **Keeping it open.** A trigger within the 4 s extends the popup, without showing the name again. A trigger every 3.5 s held it open with no limit seen (43 s) and no flicker. The trigger waits for the next MEDIA_STATUS (about every 185 ms), so a gap can reach about 3.7 s.
*   **The dots.** Text alone updates an open popup, but doesn't extend or reopen it. Three texts per trigger step the dots at the end of the code (`Code: 482 913 .`, `..`, `...`) evenly, to show the code is live. The last text comes well before the next trigger: with text sent just before each trigger, every 2 s, the popup closed after 6 s and didn't come back. The cause isn't known.
*   **The source.** The text goes on the playing source's ISO-TP channel, read from MEDIA_STATUS for every text. Android Auto (0x6D2, flow control on 0x6B2) and radio (0x6DC, 0x6BB) were checked; the other channels come from wicant-i-precondition. The text is UTF-16LE padded with `AA`, like the head unit's. A title or station text sent at the same time can overwrite ours or break a transfer; the next text corrects it.
*   **Audio off.** With no source (0x14) there is no way to show the code on a tap. Copies of MEDIA_STATUS claiming a source make the cluster flip between that source and "off", so the popup flickers at about 4 Hz and the text never shows. Only a bridge could replace the head unit's frame. So the method is only available with a source that has a text channel, which also means the popup's name and icon can't be chosen.

The method is available in Ready with such a source (`ValidationMethod::is_available`). The display follows the running session: it stops when the session ends, pauses while the method isn't available or the device is read-only, and starts a new popup when it can again. The session keeps running meanwhile, up to its normal timeout. The display is driven by received frames, so nothing goes out on a sleeping bus. All its frames are platform frames, so it needs read-write mode.
