FROM mcr.microsoft.com/devcontainers/rust:2-1

RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
    gcc-aarch64-linux-gnu libc6-dev-arm64-cross \
    python3 python3-pip python3-venv \
    flex bison gperf ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 cmake \
    protobuf-compiler \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER vscode

# Install Raspberry Pi standard Rust target (64-bit)
RUN rustup target add aarch64-unknown-linux-gnu

# Install cargo-binstall for lightning-fast binary installations
RUN curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash

# Install Espressif tools using pre-compiled binaries instead of building from source
RUN cargo binstall -y espup ldproxy espflash

# Install the ESP32 FreeRTOS toolchains
RUN espup install --export-file /home/vscode/export-esp.sh

RUN echo "source /home/vscode/export-esp.sh" >> ~/.bashrc
