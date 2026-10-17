# Agent rules

Rules for AI agents working in this repo. Project knowledge (architecture, BLE, CAN, testing, releases) lives in `docs/` and `README.md`; read the relevant page before changing an area.

## Rules

### Build & test
*   Always build through `cargo xtask` (e.g. `cargo xtask build --board pc`), never plain `cargo build` on the final binary.
*   Run host tests with `cargo xtask test`. When running `cargo test` directly, always pass `-- --test-threads=1` (Embassy channels are process-global statics).
*   Don't hand-edit `.app_build/` or `.app_test_build/`; they are regenerated. Change `main.template.rs`, `tests.template.rs` or `xtask_builder.rs` instead.
*   ESP builds need the dev container toolchain (`+esp`, `-Zbuild-std`). If you can't run an ESP build, say so rather than claiming it passes.

### Architecture boundaries
*   `core-interface` knows nothing about boards or cars; boards know nothing about vehicles or protos; vehicles never touch board code. The only link between them is the static channels in `core-interface`.
*   Board crates only read/write the edge channels; vehicle crates only the inner channels.
*   Never block on a channel send where a downstream stall could deadlock (see the `try_send` rules in `docs/architecture.md`).
*   `#[embassy_executor::task]` can't take generic parameters; keep tasks concrete and put testable logic in plain helper functions.
*   Keep `no_std` crates host-testable: HAL code stays behind the `hardware` feature.

### Safety
*   CAN TX must stay gated by `is_can_read_only()`; never add a path that writes to a bus without going through the read-only check. Don't call `set_can_read_only(false)` before the vehicle has validated the car.
*   BLE: never trust `source_device_id` from the app; keep characteristics `encrypted` and the pairing-window checks intact.

### Git & files
*   `config.toml` is gitignored and holds per-developer secrets (WiFi credentials, pins). Never commit it or print its contents; edit `config.toml.example` for defaults.
*   Don't commit build artifacts or `.bin` images.
*   Commits use conventional-commit style (`feat(scope):`, `fix(scope):`, `chore:`), matching the existing history.

### Keeping docs current
*   When a change alters behaviour described in `docs/` or the README, update that page in the same change.
*   Put new project knowledge in the matching `docs/` page (or a new page there), not in this file. This file only gets rules about how to work.
*   Don't record volatile facts (test counts, "verified builds", line numbers) in docs; they go stale.
