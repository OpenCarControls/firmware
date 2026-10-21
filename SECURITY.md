# Security policy

This project is experimental and not ready for use on real vehicles (see the warning in the [README](README.md)). Security reports are still very welcome, and they matter more here than in most projects.

## Reporting a vulnerability

Please **do not open a public issue** for security problems. Report them privately through GitHub: open the **Security** tab of this repository and choose **Report a vulnerability**.

Include what you found, how to reproduce it, and what an attacker could do with it. You will get an acknowledgement as soon as the maintainer can respond; this is a hobby project, so please allow some time. Please give us a chance to fix the problem before disclosing it publicly.

## What counts

Because this project controls vehicle hardware, we are especially interested in:

*   Anything that lets an unauthenticated or unpaired device send commands, or bypass the BLE pairing window or encryption.
*   Any path that writes to a CAN bus while the controller is in read-only mode.
*   Trusting data supplied by the app or the network (for example a spoofed `source_device_id`).
*   Flaws in OTA updates, such as accepting unsigned or tampered images.
*   Leaked credentials or secrets.

Only the latest `main` is supported; there are no maintained older releases yet.
