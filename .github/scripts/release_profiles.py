"""Lists the hardware profiles to release a vehicle platform for.

Usage: release_profiles.py <platform>   (e.g. "egmp" or "virtual-car"), from the repo root.

Prints one tab-separated line per profile to build: `<profile> <mcu> <target-triple>`.
A profile is skipped (with the reason on stderr) when its board isn't released, or when it
defines fewer CAN buses than the platform needs. Exits with an error when no profile is left,
so a platform is never released without firmware.
"""

import sys
import tomllib
from pathlib import Path

# Only ESP boards get release images; the PC board is for development.
ESP_TARGETS = {
    "esp32": "xtensa-esp32-none-elf",
    "esp32s3": "xtensa-esp32s3-none-elf",
    "esp32c3": "riscv32imc-unknown-none-elf",
    "esp32c6": "riscv32imac-unknown-none-elf",
}


def main() -> None:
    platform = sys.argv[1]
    # Same lookup as xtask's `load_platform_meta`: contracts use underscores.
    meta_path = Path("contracts/opencar/cars") / platform.replace("-", "_") / "v1/meta.toml"
    can_bus_count = tomllib.loads(meta_path.read_text())["can_bus_count"]

    selected = 0
    for path in sorted(Path("hardware-profiles").glob("*.toml")):
        profile = tomllib.loads(path.read_text())
        name = path.stem
        board = profile.get("target", {}).get("board")
        if board != "esp":
            print(f"Skipping {name}: board {board!r} isn't released", file=sys.stderr)
            continue
        esp = profile.get("hardware", {}).get("esp", {})
        mcu = esp.get("mcu")
        if mcu not in ESP_TARGETS:
            sys.exit(f"{name}: unsupported or missing [hardware.esp].mcu {mcu!r}")
        buses = len(esp.get("can_buses", []))
        if buses < can_bus_count:
            print(
                f"Skipping {name}: {buses} CAN bus(es), {platform} needs {can_bus_count}",
                file=sys.stderr,
            )
            continue
        print(f"{name}\t{mcu}\t{ESP_TARGETS[mcu]}")
        selected += 1

    if selected == 0:
        sys.exit(f"No hardware profile can run {platform}")


if __name__ == "__main__":
    main()
