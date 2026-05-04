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

# ── esp-riscv ─────────────────────────────────────────────────────────────────
# ESP32-C3 / ESP32-C6: RISC-V bare-metal targets, no Xtensa toolchain needed.
FROM esp-base AS esp-riscv

RUN rustup target add riscv32imc-unknown-none-elf riscv32imac-unknown-none-elf

# ── esp-xtensa ────────────────────────────────────────────────────────────────
# ESP32-S3 / ESP32-S2 / ESP32: full Xtensa toolchain via espup.
FROM esp-base AS esp-xtensa

RUN espup install --export-file /home/vscode/export-esp.sh

RUN echo "source /home/vscode/export-esp.sh" >> ~/.bashrc
