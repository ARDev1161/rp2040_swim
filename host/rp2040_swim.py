#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from devices import DEVICES, get_device
from entry_waveform import (
    ENTRY_FAST_PERIOD_US,
    ENTRY_FAST_PULSES,
    ENTRY_INITIAL_LOW_US,
    ENTRY_PROTOCOL_US,
    ENTRY_SLOW_PERIOD_US,
    ENTRY_SLOW_PULSES,
    um0470_entry_segments,
)
from ihex import HexError, image_for_range, load_ihex
from protocol import Probe, ProtocolError, autodetect_port, list_serial_ports


VOLTAGE_WARNING = "RP2040 GPIO is not 5V tolerant. Use 3.3V target or level shifter."
STM8_FLASH_IAPSR = 0x505F
STM8_FLASH_PUKR = 0x5062
STM8_FLASH_DUKR = 0x5064
STM8_IAPSR_PUL = 0x02
STM8_IAPSR_DUL = 0x08

ENTER_STAGE_NAMES = {
    0: "IDLE",
    1: "RESET_ASSERTED",
    2: "ENTRY_SENT",
    3: "SYNC1_OK",
    4: "COMM_RESET_SENT",
    5: "SYNC2_OK",
    6: "SWIM_CSR_WRITE_START",
    7: "SWIM_CSR_WRITE_OK",
    8: "SWIM_CSR_READ_START",
    9: "SWIM_CSR_READ_OK",
    10: "DONE",
    11: "FAIL",
}

def parse_int(value: str) -> int:
    return int(value, 0)


def open_probe(args: argparse.Namespace) -> Probe:
    port = args.port or autodetect_port()
    probe = Probe(port)
    probe.set_pins(args.swim_pin, args.nrst_pin, args.pullup)
    probe.set_speed(args.high_speed)
    return probe

def write_key_pair_same_register(probe: Probe, key_reg: int, key1: int, key2: int) -> None:
    """Write STM8 FLASH key bytes as two byte writes to the same key register."""
    # PUKR/DUKR are key registers, not a two-byte memory window. A single
    # multi-byte MEMORY_WRITE to key_reg would store key2 at key_reg + 1 and
    # the STM8 MASS unlock sequence would be ignored.
    probe.write_memory(key_reg, bytes([key1]))
    probe.write_memory(key_reg, bytes([key2]))


def wait_iapsr_mask(probe: Probe, mask: int, attempts: int = 100) -> int:
    last_iapsr = 0
    for _ in range(attempts):
        last_iapsr = probe.read_memory(STM8_FLASH_IAPSR, 1)[0]
        if (last_iapsr & mask) == mask:
            return last_iapsr
        time.sleep(0.001)
    return last_iapsr

def cmd_list(_args: argparse.Namespace) -> int:
    for port in list_serial_ports():
        print(port)
    return 0


def cmd_probe(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    with open_probe(args) as probe:
        print(probe.version())
        probe.enter_swim()
        data = probe.read_memory(0x8000, 4)
        print(f"target responded; reset vector bytes: {data.hex()}")
    return 0


def cmd_enter_debug(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    enter_error: ProtocolError | None = None
    with open_probe(args) as probe:
        print(probe.version())
        try:
            probe.enter_swim()
        except ProtocolError as exc:
            enter_error = exc
        debug = probe.swim_debug()
    print(f"synced: {'yes' if debug.synced else 'no'}")
    print(f"speed: {debug.speed}")
    print(f"last_sync_low_us: {debug.last_sync_low_us}")
    print(f"last_sync_low_ns: {debug.last_sync_low_ns}")
    print(f"derived_tswim_ns: {debug.derived_tswim_ns}")
    print(f"sync_low_loop_count: {debug.sync_low_loop_count}")
    print(f"enter_stage: {ENTER_STAGE_NAMES.get(debug.enter_stage, debug.enter_stage)}")
    print(f"comm_reset_sent: {'yes' if debug.comm_reset_sent else 'no'}")
    print(f"comm_reset_low_us: {debug.comm_reset_low_us}")
    print(f"comm_reset_low_ns: {debug.comm_reset_low_ns}")
    print(f"second_sync_seen: {'yes' if debug.second_sync_seen else 'no'}")
    print(f"phy_backend: {debug.phy_backend}")
    print(f"entry_protocol_us: {debug.entry_protocol_us}")
    print(f"entry_slow_pulses: {debug.entry_slow_pulses}")
    print(f"entry_fast_pulses: {debug.entry_fast_pulses}")
    print(f"pio_init_ok: {'true' if debug.pio_init_ok else 'false'}")
    print(f"pio_error: {debug.pio_error}")
    if debug.swim_csr_valid:
        dm = "set" if (debug.swim_csr & 0x20) else "clear"
        mask_rst = "set" if (debug.swim_csr & 0x80) else "clear"
        print(f"swim_csr: 0x{debug.swim_csr:02x} SWIM_DM={dm} SAFE_MASK={mask_rst}")
    else:
        print("swim_csr: unavailable")
    if enter_error is not None:
        print(f"enter_swim: FAIL: {enter_error}", file=sys.stderr)
        return 1
    print("enter_swim: PASS")
    return 0


def cmd_entry_waveform(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    print(f"initial_low_us={ENTRY_INITIAL_LOW_US}")
    print(f"slow_pulses={ENTRY_SLOW_PULSES}")
    print(f"slow_period_us={ENTRY_SLOW_PERIOD_US}")
    print(f"fast_pulses={ENTRY_FAST_PULSES}")
    print(f"fast_period_us={ENTRY_FAST_PERIOD_US}")
    print(f"total_entry_protocol_us={ENTRY_PROTOCOL_US}")
    for index, segment in enumerate(um0470_entry_segments()):
        level = "low" if int(segment.level) == 1 else "release"
        print(f"segment[{index}] level={level} requested_duration_us={segment.duration_us}")
    with open_probe(args) as probe:
        print(probe.version())
        probe.entry_waveform(args.delay_ms)
        debug = probe.swim_debug()
    print(f"phy_backend={debug.phy_backend}")
    print(f"pio_init_ok={'true' if debug.pio_init_ok else 'false'}")
    print(f"pio_error={debug.pio_error}")
    print("nrst_behavior=released after waveform")
    return 0


def cmd_reset(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    with open_probe(args) as probe:
        probe.reset_target()
    return 0


def cmd_read(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    with open_probe(args) as probe:
        probe.enter_swim()
        data = probe.read_memory(args.addr, args.length)
    Path(args.out).write_bytes(data)
    print(f"read {len(data)} bytes from 0x{args.addr:06x} to {args.out}")
    return 0


def cmd_write_ram(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    data = Path(args.file).read_bytes()
    with open_probe(args) as probe:
        probe.enter_swim()
        probe.write_memory(args.addr, data)
    print(f"wrote {len(data)} bytes to RAM at 0x{args.addr:06x}")
    return 0


def cmd_unlock(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    get_device(args.device)
    with open_probe(args) as probe:
        probe.enter_swim()
        if args.flash:
            write_key_pair_same_register(probe, STM8_FLASH_PUKR, 0x56, 0xAE)
            mask = STM8_IAPSR_PUL
            area = "flash"
        else:
            write_key_pair_same_register(probe, STM8_FLASH_DUKR, 0xAE, 0x56)
            mask = STM8_IAPSR_DUL
            area = "EEPROM"
        iapsr = wait_iapsr_mask(probe, mask)
        if (iapsr & mask) != mask:
            raise ProtocolError(f"{area} unlock did not set IAPSR bit; IAPSR=0x{iapsr:02x}")
    print(f"unlocked {area} on {args.device}; IAPSR=0x{iapsr:02x}")
    return 0


def cmd_erase(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    device = get_device(args.device)
    with open_probe(args) as probe:
        probe.enter_swim()
        probe.flash_erase(device.flash_start, device.flash_size)
    print(f"erased {device.name} program flash")
    return 0


def cmd_flash(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    device = get_device(args.device)
    segments = load_ihex(args.file)
    image = image_for_range(segments, device.flash_start, device.flash_size)

    with open_probe(args) as probe:
        probe.enter_swim()
        probe.flash_erase(device.flash_start, device.flash_size)
        for offset in range(0, len(image), device.block_size):
            block = image[offset : offset + device.block_size]
            if block == bytes([0xFF]) * len(block):
                continue
            probe.flash_write_block(device.flash_start + offset, block)
        if args.verify:
            readback = probe.read_memory(device.flash_start, len(image))
            if readback != image:
                raise ProtocolError("verify failed")
        if args.reset:
            probe.reset_target()
    print(f"programmed {args.file} to {device.name}")
    return 0


def add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", help="serial port, autodetected when omitted")
    parser.add_argument("--swim-pin", type=parse_int, default=2, help="RP2040 GPIO for SWIM")
    parser.add_argument("--nrst-pin", type=parse_int, default=3, help="RP2040 GPIO for NRST")
    parser.add_argument("--pullup", action="store_true", help="enable RP2040 internal SWIM pull-up")
    parser.add_argument("--high-speed", action="store_true", help="request high-speed SWIM timing (currently unsupported)")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RP2040-Zero STM8 SWIM programmer host tool")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("list", help="list serial ports")
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("probe", help="enter SWIM and read reset vector")
    add_common(p)
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("enter-debug", help="enter SWIM and print initialization timing/debug state")
    add_common(p)
    p.set_defaults(func=cmd_enter_debug)

    p = sub.add_parser("entry-waveform", help="emit only the UM0470 entry waveform for analyzer capture")
    add_common(p)
    p.add_argument("--delay-ms", type=parse_int, default=1000, help="delay before touching pins")
    p.set_defaults(func=cmd_entry_waveform)

    p = sub.add_parser("reset", help="reset target")
    add_common(p)
    p.set_defaults(func=cmd_reset)

    p = sub.add_parser("read", help="read target memory")
    add_common(p)
    p.add_argument("--addr", type=parse_int, required=True)
    p.add_argument("--len", dest="length", type=parse_int, required=True)
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_read)

    p = sub.add_parser("write-ram", help="write bytes to RAM")
    add_common(p)
    p.add_argument("--addr", type=parse_int, required=True)
    p.add_argument("--file", required=True)
    p.set_defaults(func=cmd_write_ram)

    p = sub.add_parser("flash", help="flash Intel HEX/IHX program")
    add_common(p)
    p.add_argument("--device", choices=sorted(DEVICES), required=True)
    p.add_argument("--file", required=True)
    p.add_argument("--verify", action="store_true")
    p.add_argument("--reset", action="store_true")
    p.set_defaults(func=cmd_flash)

    p = sub.add_parser("erase", help="erase device flash")
    add_common(p)
    p.add_argument("--device", choices=sorted(DEVICES), required=True)
    p.set_defaults(func=cmd_erase)

    p = sub.add_parser("unlock", help="unlock flash or EEPROM")
    add_common(p)
    p.add_argument("--device", choices=sorted(DEVICES), required=True)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--flash", action="store_true")
    group.add_argument("--eeprom", action="store_true")
    p.set_defaults(func=cmd_unlock)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (OSError, ValueError, ProtocolError, HexError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
