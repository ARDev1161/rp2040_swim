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
STM8_FLASH_CR2 = 0x505B
STM8_FLASH_NCR2 = 0x505C
STM8_FLASH_IAPSR = 0x505F
STM8_FLASH_PUKR = 0x5062
STM8_FLASH_DUKR = 0x5064
STM8_FLASH_CR2_OPT = 0x80
STM8_FLASH_NCR2_NOPT = 0x80
STM8_IAPSR_PUL = 0x02
STM8_IAPSR_EOP = 0x04
STM8_IAPSR_DUL = 0x08
STM8_IAPSR_WR_PG_DIS = 0x01
STM8_FLASH_ERASED_VALUE = 0x00
STM8_ROP_ENABLED_VALUE = 0xAA
STM8_ROP_DISABLED_VALUE = 0x00

ROP_DESTRUCTIVE_WARNING = (
    "ROP unprotect is destructive: it will mass erase program flash, "
    "data EEPROM and option bytes."
)

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


def read_u8(probe: Probe, address: int) -> int:
    return probe.read_memory(address, 1)[0]


def write_u8(probe: Probe, address: int, value: int) -> None:
    probe.write_memory(address, bytes([value & 0xFF]))


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


def wait_flash_eop(probe: Probe, attempts: int = 250) -> int:
    """Wait until STM8 reports end-of-programming for an EEPROM/option write."""
    last_iapsr = 0
    for _ in range(attempts):
        last_iapsr = read_u8(probe, STM8_FLASH_IAPSR)
        if last_iapsr & STM8_IAPSR_WR_PG_DIS:
            raise ProtocolError(f"flash/option write rejected; IAPSR=0x{last_iapsr:02x}")
        if last_iapsr & STM8_IAPSR_EOP:
            return last_iapsr
        time.sleep(0.001)
    raise ProtocolError(f"timeout waiting for flash/option EOP; IAPSR=0x{last_iapsr:02x}")


def unlock_data_eeprom(probe: Probe) -> int:
    """Unlock STM8 data EEPROM / option-byte area."""
    iapsr = read_u8(probe, STM8_FLASH_IAPSR)
    if (iapsr & STM8_IAPSR_DUL) == STM8_IAPSR_DUL:
        return iapsr
    write_key_pair_same_register(probe, STM8_FLASH_DUKR, 0xAE, 0x56)
    iapsr = wait_iapsr_mask(probe, STM8_IAPSR_DUL)
    if (iapsr & STM8_IAPSR_DUL) != STM8_IAPSR_DUL:
        raise ProtocolError(f"EEPROM/option unlock did not set DUL; IAPSR=0x{iapsr:02x}")
    return iapsr


def set_option_byte_programming(probe: Probe, enabled: bool) -> None:
    """Enable/disable STM8 option-byte programming through FLASH_CR2/NCR2."""
    cr2 = read_u8(probe, STM8_FLASH_CR2)
    ncr2 = read_u8(probe, STM8_FLASH_NCR2)
    if enabled:
        cr2 |= STM8_FLASH_CR2_OPT
        ncr2 &= ~STM8_FLASH_NCR2_NOPT
    else:
        cr2 &= ~STM8_FLASH_CR2_OPT
        ncr2 |= STM8_FLASH_NCR2_NOPT
    write_u8(probe, STM8_FLASH_CR2, cr2)
    write_u8(probe, STM8_FLASH_NCR2, ncr2)


def read_rop_byte(probe: Probe, device_name: str) -> int:
    device = get_device(device_name)
    return read_u8(probe, device.option_start)


def rop_is_enabled(rop_byte: int) -> bool:
    return (rop_byte & 0xFF) == STM8_ROP_ENABLED_VALUE


def write_rop_byte(probe: Probe, device_name: str, value: int) -> int:
    """Write STM8 OPT0/ROP byte and wait for the option write to complete."""
    device = get_device(device_name)
    unlock_data_eeprom(probe)
    set_option_byte_programming(probe, True)
    try:
        write_u8(probe, device.option_start, value)
        return wait_flash_eop(probe)
    finally:
        try:
            set_option_byte_programming(probe, False)
        except ProtocolError:
            pass


def reconnect_after_rop_change(probe: Probe, attempts: int = 12) -> None:
    """Reset and re-enter SWIM after an ROP transition or destructive unprotect."""
    last_error: Exception | None = None
    for _ in range(attempts):
        try:
            probe.reset_target()
            time.sleep(0.1)
            probe.enter_swim()
            return
        except (OSError, ProtocolError) as exc:
            last_error = exc
            time.sleep(0.25)
    raise ProtocolError(f"target did not reconnect after ROP change: {last_error}")


def unlock_program_flash(probe: Probe) -> int:
    """Unlock STM8 program flash using the validated host-side key sequence."""
    iapsr = probe.read_memory(STM8_FLASH_IAPSR, 1)[0]
    if (iapsr & STM8_IAPSR_PUL) == STM8_IAPSR_PUL:
        return iapsr
    write_key_pair_same_register(probe, STM8_FLASH_PUKR, 0x56, 0xAE)
    return wait_iapsr_mask(probe, STM8_IAPSR_PUL)


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


def cmd_rop_status(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    device = get_device(args.device)
    with open_probe(args) as probe:
        probe.enter_swim()
        rop = read_rop_byte(probe, device.name)
    enabled = rop_is_enabled(rop)
    print(f"Device: {device.name}")
    print(f"OPT0/ROP @ 0x{device.option_start:06x}: 0x{rop:02x}")
    print(f"ROP: {'enabled' if enabled else 'disabled'}")
    if enabled:
        print("Program flash and data EEPROM readout are protected.")
        print(ROP_DESTRUCTIVE_WARNING)
    return 0


def cmd_set_rop(args: argparse.Namespace) -> int:
    print(
        "error: set-rop is temporarily disabled. "
        "The first hardware test produced an invalid option-byte state. "
        "ROP writes must be reworked as a firmware-side atomic option-byte transaction.",
        file=sys.stderr,
    )
    return 1


def cmd_unprotect_rop(args: argparse.Namespace) -> int:
    print(
        "error: unprotect-rop is temporarily disabled. "
        "ROP unprotect is destructive and option-byte programming is not validated yet.",
        file=sys.stderr,
    )
    return 1


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
        unlock_program_flash(probe)
        probe.flash_erase(device.flash_start, device.flash_size)
    print(f"erased {device.name} program flash")
    return 0


def cmd_flash(args: argparse.Namespace) -> int:
    print(VOLTAGE_WARNING, file=sys.stderr)
    device = get_device(args.device)
    segments = load_ihex(args.file)
    image = image_for_range(segments, device.flash_start, device.flash_size, fill=STM8_FLASH_ERASED_VALUE)

    with open_probe(args) as probe:
        probe.enter_swim()
        unlock_program_flash(probe)
        probe.flash_erase(device.flash_start, device.flash_size)
        for offset in range(0, len(image), device.block_size):
            block = image[offset : offset + device.block_size]
            if block == bytes([STM8_FLASH_ERASED_VALUE]) * len(block):
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

    p = sub.add_parser("rop-status", help="read STM8 ROP/readout-protection status")
    add_common(p)
    p.add_argument("--device", choices=sorted(DEVICES), required=True)
    p.set_defaults(func=cmd_rop_status)

    p = sub.add_parser("set-rop", help="enable STM8 ROP/readout protection")
    add_common(p)
    p.add_argument("--device", choices=sorted(DEVICES), required=True)
    p.add_argument("--yes-i-know", action="store_true", help="confirm that readout protection will be enabled")
    p.set_defaults(func=cmd_set_rop)

    p = sub.add_parser("unprotect-rop", help="disable STM8 ROP by destructive mass erase")
    add_common(p)
    p.add_argument("--device", choices=sorted(DEVICES), required=True)
    p.add_argument("--yes-erase-all", action="store_true", help="confirm destructive mass erase")
    p.set_defaults(func=cmd_unprotect_rop)

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
