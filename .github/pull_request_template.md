## What and why

<!-- What does this change do, and why? Link the issue it addresses. -->

## How it was tested

<!-- Host tests, on-device tests, real hardware or vehicle? Say what you could NOT run (e.g. ESP build). -->

## Checklist

- [ ] PR title uses conventional-commit style (`feat(scope):`, `fix(scope):`, `docs:`, `chore:`)
- [ ] `cargo fmt --all -- --check`, `cargo xtask clippy` and `cargo xtask test` pass
- [ ] Docs (`docs/` or README) updated if behaviour changed
- [ ] No `config.toml`, credentials, build artifacts or `.bin` files committed
- [ ] If the wire format changed: the contracts PR is linked and the submodule bump is its own commit

## Safety

- [ ] This change does **not** touch CAN TX, BLE pairing/encryption, or OTA
- [ ] It does, and CAN TX is still gated by `is_can_read_only()` and BLE checks are intact (explain below)

<!-- Explain any safety-sensitive change here. -->
