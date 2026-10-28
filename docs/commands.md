# Vehicle commands

How a vehicle crate answers a command from the app, and the rule every command follows before it builds a CAN frame. See [Architecture](architecture.md) for the channels a command travels through and [CAN bus](can.md#read-write-policy) for the read-write policy (car identity, owner approval, dev lock).

## The requirements rule
Every command runs a check before its frames are built. The check is generic and lives in `core-interface` (`requirements.rs`); each vehicle declares what its commands need.

*   **`CommandRequirements`** is what a command needs: how fresh the vehicle's state must be (`max_state_age`) plus a list of **`Requirement`**s. A requirement is a condition on the car that the vehicle has already evaluated from its own state: `Some(true)` holds, `Some(false)` doesn't, `None` the state isn't known yet. Each of the two failing outcomes carries the message the app shows. An unknown condition rejects the command, so a missing signal never lets a command through.
*   **`Snapshot`** is the gate's input: why the device is read-only, if it is (`can_read_only_reason()`), and the time since the vehicle last heard from the car (`ActivityClock`). `Snapshot::capture` reads the live values; tests build one by hand.
*   **`CommandRequirements::check(&snapshot)`** returns the first reason the command can't run, in this order:
    1.  the device is read-only, with the reason from the read-write policy (car not identified, not approved, dev lock locked, logging active, ...);
    2.  no data from the car since boot, or the last data is older than `max_state_age` (the car is asleep or the bus is silent; see firmware#11);
    3.  each requirement, in the order declared.

    Freshness comes before the requirements on purpose: a "parked" fact read from stale state must not be trusted.
*   The vehicle feeds the `ActivityClock` by touching it for every CAN frame that matched its filters. It is a "time since the last matching frame" input, not a per-signal age: a live bus does not prove the specific signal a requirement uses is recent.

### Always answered
A command always gets a `CommandResponse`, including when it is rejected because the device is read-only. A rejection is `success: false` with the `COMMAND_STATUS_CODE_REJECTED_UNAVAILABLE` status code and a message that says why. The dispatcher's read-only refusal uses the same code. Vehicles answer through `respond_to_command`, which queues a rejection with `try_send` (`try_send_response`): nothing was sent to the car, so a dropped rejection only costs the app a retry, and waiting on a full `CMD_RESP_CHANNEL` could stall the handler behind the router and the BLE/MQTT drivers (see the `try_send` rules in [Architecture](architecture.md#channel-flow)). The response of a command that did run still uses the blocking send so its result isn't lost.

The BLE and MQTT dispatcher already refuses vehicle commands while the device is read-only, before they reach the vehicle (see [CAN bus](can.md#read-write-policy)). The check repeats that against the live flag, because a command can wait in the queue while the policy changes, so the two use the same wording.

### In a vehicle crate
A handler decodes the command, calls the vehicle's admission function (for E-GMP `commands::admit_basic` / `admit_advanced`), and only if that passes goes on to build frames. It returns a `CommandError` (`Rejected` or `Failed`) and the handler task answers with `respond_to_command`. The requirements are plain functions of the vehicle state, so they are tested with fixture states and no executor.

## Command availability
The app greys out the commands the device would reject, so the requirements are also published as availability (contracts#6). It is the same check, so what the app shows and what the device accepts can't disagree.

*   **Evaluation:** the vehicle evaluates every command's requirements against its state about once a second (E-GMP: `commands::report_availability`, called from `state_update_task`) and reports it with `report_command_availability(kind, &[(command_id, state)])`. A command with several requests, like locking and unlocking, is available when any of them would be accepted. Commands are identified by their field number in the vehicle's `BasicCommand` / `AdvancedCommand` oneof (`commands::command_id`; a test pins them to the wire), so `core-interface` stays vehicle-agnostic.
*   **What `core-interface` publishes** (`availability.rs`): `SystemState.command_availability` in a `StateUpdate` of its own like the CAN access state: when it changes, every `COMMAND_AVAILABILITY_REPEAT_MS` (5 s) while it doesn't, and again when the app subscribes. It is never more optimistic than what the device would answer:
    *   nothing is published until the vehicle has reported;
    *   while the device is read-only every command is unavailable, whatever was reported (a flip is published within a second);
    *   a report older than `REPORT_MAX_AGE_MS` (5 s) counts as unavailable, so a vehicle that stopped reporting can't leave a command shown as available.
*   **BLE and MQTT:** BLE gets both command sets. MQTT clients get the basic commands only (advanced commands are BLE only), at the MQTT state's rate: every 5 s while a client is active, once per MQTT idle period otherwise (`publish_command_availability_mqtt`). MQTT commands are not authenticated yet (`MQTT_COMMANDS_AUTHENTICATED`) and the device rejects all of them, so over MQTT every command is reported unavailable until that changes.
*   **States:** `AVAILABLE`, `UNAVAILABLE` (not right now: read-only, state out of date, the car is driving, ...) and `UNSUPPORTED` (not on this car). Nothing sends `UNSUPPORTED` yet; how a car declares it is still to be decided.
*   **The app** must trust an availability only if it was received on the current connection and treat a command with no entry as unavailable. It never caches or persists it.
*   **Virtual car:** reports its commands from its simulated state, so the app's grey-out and rejection messages can be tried without a car (see below). Like any other platform they are unavailable until the device is read-write.

## Virtual car requirements
The virtual car is the platform for trying the app without a car, so it has one requirement of its own: the door lock command is refused (and reported unavailable) while the simulated car is driving (`is_driving`), with `command rejected: the car is driving`. The simulation only runs on the PC board in a debug build, started with `POST /simulate` on its debug HTTP server; the car then drives for about 75 s of every 100 s cycle. On the ESP board nothing starts it, so the virtual car never drives there and its door lock is available whenever the device is read-write. The advanced command has no requirement. Its state is always fresh, so only the car requirement matters; the device being read-write is enforced by the dispatcher.

## E-GMP requirements
No E-GMP command is implemented yet (firmware#37 to #42), so each needs only what every command needs: the device is not read-only and the car was heard from in the last 10 s (`commands::MAX_STATE_AGE`, provisional until checked against a log of the car falling asleep). While the device is read-only the answer is the read-only reason rather than "Not implemented".

Each command adds its own requirements in the PR that implements it, from real behaviour of the car. Candidates to settle there: not driving (charge port, trunk and frunk, windows), car awake (`power_state` not Off) for the commands that need the car on, and the current state differing from the requested one. A state that isn't known yet fails a requirement.

| Command | Additional requirements | Confirmed on a car |
|---|---|---|
| Door lock / unlock | none yet | no |
| Charge port open / close | none yet | no |
| Window close / vent | none yet | no |
| Flash lights | none yet | no |
| Climate control | none yet | no |
| Battery preconditioning | none yet | no |

## Convention for new commands
Each command PR lists its requirements in its description and updates the table above: the requirements, the maximum state age if it differs, and whether each was confirmed on a real car. A command doesn't build a frame before its requirements pass, and always answers through `respond_to_command`. Its requirements also decide its availability, so a new command needs an entry in `basic_variants()` / `advanced_availability()` and a field number in `command_id`.
