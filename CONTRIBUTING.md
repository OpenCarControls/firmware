# Contributing to Open Car Firmware

Thanks for your interest! This project is under construction and drives real vehicle hardware, so contributions are reviewed carefully. Please read the warning in the [README](README.md) first.

## Before you start

*   **Open an issue first** for anything beyond a small fix, so we can agree on the approach before you spend time on it.
*   Supporting a new vehicle or board? Say which one, how you captured the CAN data, and whether you have access to the car.
*   Report security problems privately, as described in [SECURITY.md](SECURITY.md).

## Setup

Use the dev container (`.devcontainer/`); it has the toolchains for every board. See the [README](README.md) and the pages in [`docs/`](docs/) (architecture, CAN, BLE, testing, hardware setup). Read the page for the area you are changing.

## Before opening a pull request

```bash
cargo fmt --all -- --check
cargo xtask clippy
cargo xtask test
```

*   Build through `cargo xtask`, never plain `cargo build` on the final binary.
*   If you run `cargo test` directly, pass `-- --test-threads=1` (the Embassy channels are process-global statics).
*   Don't hand-edit `.app_build/` or `.app_test_build/`; they are regenerated.
*   ESP builds need the dev container toolchain. If you could not run one, say so in the PR.
*   Update the matching page in `docs/` (or the README) when you change documented behaviour.
*   Never commit `config.toml` (it holds credentials), build artifacts or `.bin` images.

## Safety-sensitive areas

These get extra scrutiny, and changes to them need a clear explanation and tests:

*   **CAN TX** must stay gated by `is_can_read_only()`. Never add a path that writes to a bus without it.
*   **BLE**: never trust `source_device_id` from the app; keep characteristics encrypted and the pairing-window checks intact.
*   **Architecture boundaries** (see [`docs/architecture.md`](docs/architecture.md)): `core-interface` knows nothing about boards or cars, boards know nothing about vehicles, and vehicles never touch board code.

## Changes to the wire format

Messages and transport constants live in the shared [`contracts`](https://github.com/OpenCarControls/contracts) repository (the `contracts/` submodule). Contract changes must follow its append-only rules. Open the contracts PR first, then bump the submodule pin here in its own commit.

## Pull requests

*   Keep them small and focused; one topic per PR.
*   Use conventional-commit style for commits and the PR title: `feat(scope):`, `fix(scope):`, `docs:`, `chore:`. CI validates the PR title.
*   Fill in the PR template, including how you tested and whether real hardware was involved.
*   AI-assisted contributions are fine, but you must understand and be able to explain every line you submit.
*   CI runs on your first PR only after a maintainer approves it.

By contributing you agree that your work is released under the project's [LICENSE](LICENSE).
