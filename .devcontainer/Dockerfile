# One image for every board: PC/Raspberry Pi and all ESP targets
# (RISC-V C3/C6 + Xtensa S3/classic). Which board you work on is chosen in
# config.toml, not by picking a container; see `cargo xtask ide`.
FROM mcr.microsoft.com/devcontainers/rust:2-1

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
    protobuf-compiler \
    python3 python3-pip \
    libusb-1.0-0 libudev-dev pkg-config \
    usbip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout vscode

USER vscode

# Raspberry Pi cross-compile target (64-bit) and the ESP RISC-V targets.
RUN rustup target add aarch64-unknown-linux-gnu riscv32imc-unknown-none-elf riscv32imac-unknown-none-elf

# Espressif Cargo tools
RUN curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash

RUN cargo binstall -y espup espflash

RUN cargo binstall -y probe-rs-tools

# Xtensa toolchain (`+esp`), used by xtask for ESP32/ESP32-S3 builds.
RUN espup install --export-file /home/vscode/export-esp.sh

RUN echo "source /home/vscode/export-esp.sh" >> ~/.bashrc
