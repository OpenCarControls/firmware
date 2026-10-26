# Vehicle commands

How a vehicle crate answers a command from the app, and the rule every command follows before it builds a CAN frame. See [Architecture](architecture.md) for the channels a command travels through and [CAN bus](can.md#read-write-policy) for the read-write policy (car identity, owner approval, dev lock).

## The preconditions rule
Every command runs a check before its frames are built. The check is generic and lives in `core-interface` (`preconditions.rs`); each vehicle declares what its commands need.

*   **`Preconditions`** is what a command needs: how fresh the vehicle's state must be (`max_state_age`) plus a list of **`Requirement`**s. A requirement is a condition on the car that the vehicle has already evaluated from its own state: `Some(true)` holds, `Some(false)` doesn't, `None` the state isn't known yet. The two failing outcomes carry the message the app shows. An unknown condition rejects the command, so a missing signal never lets a command through.
*   **`Snapshot`** is the gate's input: why the device is read-only, if it is (`can_read_only_reason()`), and the time since the vehicle last heard from the car (`ActivityClock`). `Snapshot::capture` reads the live values; tests build one by hand.
*   **`Preconditions::check(&snapshot)`** returns the first reason the command can't run, in this order:
    1.  the device is read-only, with the reason from the read-write policy (car not identified, not approved, dev lock locked, logging active, ...);
    2.  no data from the car since boot, or the last data is older than `max_state_age` (the car is asleep or the bus is silent; see firmware#11);
    3.  each requirement, in the order declared.

    Freshness comes before the requirements on purpose: a "parked" fact read from stale state must not be trusted.
*   The vehicle feeds the `ActivityClock` by touching it for every CAN frame that matched its filters. It is a "time since the last matching frame" input, not a per-signal age: a live bus does not prove the specific signal a requirement uses is recent.

### Always answered
A command always gets a `CommandResponse`, including when it is rejected because the device is read-only. A rejection is `success: false` with the `COMMAND_STATUS_CODE_REJECTED_UNAVAILABLE` status code and a message that says why. The dispatcher's read-only refusal uses the same code. Rejections are queued with `try_send` (`try_send_response`): nothing was sent to the car, so a dropped rejection only costs the app a retry, and waiting on a full `CMD_RESP_CHANNEL` could stall the handler behind the router and the BLE/MQTT drivers (see the `try_send` rules in [Architecture](architecture.md#channel-flow)). The response of a command that did run still uses the blocking send so its result isn't lost.

The BLE and MQTT dispatcher already refuses vehicle commands while the device is read-only, before they reach the vehicle (see [CAN bus](can.md#read-write-policy)). The check repeats that against the live flag, because a command can wait in the queue while the policy changes, so the two use the same wording.

### In a vehicle crate
A handler decodes the command, calls the vehicle's admission function (for E-GMP `commands::admit_basic` / `admit_advanced`), and only if that passes goes on to build frames. The requirements are plain functions of the vehicle state, so they are tested with fixture states and no executor.

## Command availability
The app greys out the commands the device would reject, so the preconditions are also published as availability (contracts#6). It is the same check, so what the app shows and what the device accepts can't disagree.

*   **Evaluation:** the vehicle evaluates every command's preconditions against its state about once a second (E-GMP: `commands::report_availability`, called from `state_update_task`) and reports it with `report_command_availability(kind, &[(command_id, state)])`. A command with several requests, like locking and unlocking, is available when any of them would be accepted. Commands are identified by their field number in the vehicle's `BasicCommand` / `AdvancedCommand` oneof (`commands::command_id`; a test pins them to the wire), so `core-interface` stays vehicle-agnostic.
*   **What `core-interface` publishes** (`availability.rs`): `SystemState.command_availability`, BLE only, in a `StateUpdate` of its own like the CAN access state: when it changes, every `COMMAND_AVAILABILITY_REPEAT_MS` (5 s) while it doesn't, and again when the app subscribes. It is never more optimistic than what the device would answer:
    *   nothing is published until the vehicle has reported;
    *   while the device is read-only every command is unavailable, whatever was reported (a flip is published within a second);
    *   a report older than `REPORT_MAX_AGE_MS` (5 s) counts as unavailable, so a vehicle that stopped reporting can't leave a command shown as available.
*   **States:** `AVAILABLE`, `UNAVAILABLE` (not right now: read-only, state out of date, the car is driving, ...) and `UNSUPPORTED` (not on this car). Nothing sends `UNSUPPORTED` yet; how a car declares it is still to be decided.
*   **The app** must trust an availability only if it was received on the current connection and treat a command with no entry as unavailable. It never caches or persists it.
*   **Virtual car:** reports its commands available and leaves the rest to `core-interface`, so they are unavailable until the device is read-write, like any other platform.

## E-GMP preconditions
Every command needs the gate above (device not read-only, car state at most 10 s old, `commands::MAX_STATE_AGE`). Beyond that:

| Command | Additional requirements | Status |
|---|---|---|
| Door lock / unlock | The doors aren't already in the requested state | provisional |
| Charge port open / close | Car not driving (`is_driving` is false); the port isn't already in the requested state | provisional |
| Window close / vent | Car not driving | provisional |
| Flash lights | none | provisional |
| Climate control | none | provisional |
| Battery preconditioning | none | provisional |

*Provisional* means the requirement was chosen as a safe starting point and isn't confirmed on a real car. Candidates still to settle per command from real behaviour: not driving for trunk and frunk, car awake (`power_state` not Off) for the commands that need the car on, and the current state differing from the requested one for the rest. A state that isn't known yet (`None`, or an unspecified charge port) fails the requirement.

All commands are still "Not implemented" (firmware#37 to #42). The checks run first, so while the device is read-only the answer is the read-only reason rather than "Not implemented".

## Convention for new commands
Each command PR lists its preconditions in its description and updates the table above: the requirements, the maximum state age if it differs, and whether each was confirmed on a real car. A command doesn't build a frame before its preconditions pass, and always answers through `commands::respond`. Its preconditions also decide its availability, so a new command needs an entry in `basic_variants()` / `advanced_availability()` and a field number in `command_id`.
