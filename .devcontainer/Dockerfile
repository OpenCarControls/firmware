FROM mcr.microsoft.com/devcontainers/rust:2-1

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
    python3 python3-pip \
    libusb-1.0-0 libudev-dev pkg-config \
    protobuf-compiler \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER vscode

# Install Raspberry Pi standard Rust target (64-bit)
RUN rustup target add aarch64-unknown-linux-gnu
# Install RISC-V targets for ESP32-C3 / ESP32-C6
RUN rustup target add riscv32imc-unknown-none-elf riscv32imac-unknown-none-elf

# Install cargo-binstall for lightning-fast binary installations
RUN curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash

# Install Espressif tools using pre-compiled binaries instead of building from source
RUN cargo binstall -y espup espflash

# Install the ESP Xtensa bare-metal toolchain
RUN espup install --export-file /home/vscode/export-esp.sh

RUN echo "source /home/vscode/export-esp.sh" >> ~/.bashrc
