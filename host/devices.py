from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Device:
    name: str
    flash_start: int
    flash_size: int
    ram_start: int
    ram_size: int
    eeprom_start: int
    eeprom_size: int
    option_start: int
    option_size: int
    block_size: int

    @property
    def flash_end(self) -> int:
        return self.flash_start + self.flash_size


DEVICES: dict[str, Device] = {
    "stm8s003f3": Device("stm8s003f3", 0x8000, 8 * 1024, 0x0000, 1024, 0x4000, 128, 0x4800, 64, 64),
    "stm8s103f3": Device("stm8s103f3", 0x8000, 8 * 1024, 0x0000, 1024, 0x4000, 640, 0x4800, 64, 64),
}


def get_device(name: str) -> Device:
    key = name.lower()
    try:
        return DEVICES[key]
    except KeyError as exc:
        known = ", ".join(sorted(DEVICES))
        raise ValueError(f"unknown device {name!r}; known devices: {known}") from exc
