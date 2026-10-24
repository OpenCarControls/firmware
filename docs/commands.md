# Vehicle commands

How a vehicle crate answers a command from the app, and the rule every command follows before it builds a CAN frame. See [Architecture](architecture.md) for the channels a command travels through and [CAN bus](can.md) for read-only mode and the car identity.

## The preconditions rule
Every command runs a check before its frames are built. The check is generic and lives in `core-interface` (`preconditions.rs`); each vehicle declares what its commands need.

*   **`Preconditions`** is what a command needs: how fresh the vehicle's state must be (`max_state_age`) plus a list of **`Requirement`**s. A requirement is a condition on the car that the vehicle has already evaluated from its own state: `Some(true)` holds, `Some(false)` doesn't, `None` the state isn't known yet. The two failing outcomes carry the message the app shows. An unknown condition rejects the command, so a missing signal never lets a command through.
*   **`Snapshot`** is the gate's input: whether the car identity is confirmed, `is_can_read_only()`, and the time since the vehicle last heard from the car (`ActivityClock`). `Snapshot::capture` reads the live values; tests build one by hand.
*   **`Preconditions::check(&snapshot)`** returns the first reason the command can't run, in this order:
    1.  the car identity isn't confirmed;
    2.  the CAN bus is read-only;
    3.  no data from the car since boot, or the last data is older than `max_state_age` (the car is asleep or the bus is silent; see firmware#11);
    4.  each requirement, in the order declared.

    Freshness comes before the requirements on purpose: a "parked" fact read from stale state must not be trusted.
*   The vehicle feeds the `ActivityClock` by touching it for every CAN frame that matched its filters. It is a "time since the last matching frame" input, not a per-signal age: a live bus does not prove the specific signal a requirement uses is recent.

### Always answered
A command always gets a `CommandResponse`, including when it is rejected because read-only is on or the identity isn't confirmed. A rejection is `success: false` with a message that says why. It uses the generic `Failed` status code until the dedicated one from contracts#13 exists. Rejections are queued with `try_send` (`try_send_response`): nothing was sent to the car, so a dropped rejection only costs the app a retry, and waiting on a full `CMD_RESP_CHANNEL` could stall the handler behind the router and the BLE/MQTT drivers (see the `try_send` rules in [Architecture](architecture.md#channel-flow)). The response of a command that did run still uses the blocking send so its result isn't lost.

### In a vehicle crate
A handler decodes the command, calls the vehicle's admission function (for E-GMP `commands::admit_basic` / `admit_advanced`), and only if that passes goes on to build frames. The requirements are plain functions of the vehicle state, so they are tested with fixture states and no executor.

## E-GMP preconditions
Every command needs the gate above (identity confirmed, bus not read-only, car state at most 10 s old, `commands::MAX_STATE_AGE`). Beyond that:

| Command | Additional requirements | Status |
|---|---|---|
| Door lock / unlock | The doors aren't already in the requested state | provisional |
| Charge port open / close | Car not driving (`is_driving` is false); the port isn't already in the requested state | provisional |
| Window close / vent | Car not driving | provisional |
| Flash lights | none | provisional |
| Climate control | none | provisional |
| Battery preconditioning | none | provisional |

*Provisional* means the requirement was chosen as a safe starting point and isn't confirmed on a real car. Candidates still to settle per command from real behaviour: not driving for trunk and frunk, car awake (`power_state` not Off) for the commands that need the car on, and the current state differing from the requested one for the rest. A state that isn't known yet (`None`, or an unspecified charge port) fails the requirement.

All commands are still "Not implemented" (firmware#37 to #42). The checks run first, so until the car identity is confirmed the answer is the identity message rather than "Not implemented".

## Convention for new commands
Each command PR lists its preconditions in its description and updates the table above: the requirements, the maximum state age if it differs, and whether each was confirmed on a real car. A command doesn't build a frame before its preconditions pass, and always answers through `commands::respond`.
