# ── base ──────────────────────────────────────────────────────────────────────
# Tools needed in every container.
FROM mcr.microsoft.com/devcontainers/rust:2-1 AS base

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
    protobuf-compiler \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ── pc ────────────────────────────────────────────────────────────────────────
# Standard Rust development targeting the host (PC board) and Raspberry Pi.
FROM base AS pc

USER vscode

# Raspberry Pi cross-compile target (64-bit)
RUN rustup target add aarch64-unknown-linux-gnu

# ── esp-base ──────────────────────────────────────────────────────────────────
# Shared foundation for both ESP containers: USB packages, Espressif Cargo tools.
FROM base AS esp-base

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
    python3 python3-pip \
    libusb-1.0-0 libudev-dev pkg-config \
    usbip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout vscode

USER vscode

RUN curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash

RUN cargo binstall -y espup espflash

RUN cargo binstall -y probe-rs-tools

# ── esp ──────────────────────────────────────────────────────────────────────
# All ESP targets: RISC-V (C3/C6) + Xtensa (S2/S3/classic) in one container.
# rust-analyzer uses the stable toolchain with the RISC-V target for IDE support;
# xtask invokes `+esp -Zbuild-std` explicitly for Xtensa firmware builds.
FROM esp-base AS esp

RUN rustup target add riscv32imc-unknown-none-elf riscv32imac-unknown-none-elf

RUN espup install --export-file /home/vscode/export-esp.sh

RUN echo "source /home/vscode/export-esp.sh" >> ~/.bashrc
