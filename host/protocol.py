from __future__ import annotations

import binascii
import enum
import glob
import struct
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover - exercised by users without pyserial
    serial = None  # type: ignore[assignment]


MAGIC = 0x53575052
VERSION = 1
MAX_PAYLOAD = 1024


class Command(enum.IntEnum):
    GET_VERSION = 0x01
    SET_PINS = 0x02
    SET_SPEED = 0x03
    ENTER_SWIM = 0x04
    RESET_TARGET = 0x05
    SWIM_READ = 0x06
    SWIM_WRITE = 0x07
    MEMORY_READ = 0x08
    MEMORY_WRITE = 0x09
    FLASH_ERASE = 0x0A
    FLASH_WRITE_BLOCK = 0x0B
    FLASH_VERIFY = 0x0C
    GET_LAST_ERROR = 0x0D
    DEBUG_WAVEFORM = 0x0E
    GET_SWIM_DEBUG = 0x0F
    ENTRY_WAVEFORM = 0x10


class Status(enum.IntEnum):
    OK = 0
    BAD_FRAME = 1
    BAD_CRC = 2
    BAD_COMMAND = 3
    BAD_ARGUMENT = 4
    SWIM_TIMEOUT = 5
    SWIM_NACK = 6
    TARGET = 7
    UNSUPPORTED = 8
    FLASH_GUARD = 9
    INTERNAL = 10


class ProtocolError(RuntimeError):
    pass


@dataclass(frozen=True)
class Response:
    command: int
    sequence: int
    status: Status
    payload: bytes


@dataclass(frozen=True)
class SwimDebug:
    synced: bool
    speed: str
    swim_csr_valid: bool
    swim_csr: int
    last_sync_low_us: int
    last_sync_low_ns: int
    derived_tswim_ns: int
    sync_low_loop_count: int
    phy_backend: str = "unknown"
    entry_protocol_us: int = 0
    entry_slow_pulses: int = 0
    entry_fast_pulses: int = 0
    pio_init_ok: bool = False
    pio_error: str = ""

    enter_stage: int = 0
    comm_reset_sent: bool = False
    second_sync_seen: bool = False
    comm_reset_low_us: int = 0
    comm_reset_low_ns: int = 0


def crc32(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def encode_frame(command: Command, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    header = struct.pack("<IBBHH", MAGIC, VERSION, int(command), sequence & 0xFFFF, len(payload))
    body = header + payload
    return body + struct.pack("<I", crc32(body))


def decode_response(frame: bytes) -> Response:
    if len(frame) < 16:
        raise ProtocolError("short response")
    magic, version, command, sequence, length = struct.unpack("<IBBHH", frame[:10])
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if length < 2 or length > MAX_PAYLOAD + 2:
        raise ProtocolError("bad payload length")
    expected_len = 10 + length + 4
    if len(frame) != expected_len:
        raise ProtocolError("truncated response")
    expected_crc = struct.unpack("<I", frame[-4:])[0]
    if crc32(frame[:-4]) != expected_crc:
        raise ProtocolError("bad crc")
    status = Status(struct.unpack("<H", frame[10:12])[0])
    return Response(command=command, sequence=sequence, status=status, payload=frame[12:-4])


def list_serial_ports() -> list[str]:
    if serial is not None:
        return [p.device for p in serial.tools.list_ports.comports()]
    ports: list[str] = []
    for pattern in ("/dev/ttyACM*", "/dev/tty.usbmodem*", "/dev/cu.usbmodem*", "COM*"):
        ports.extend(glob.glob(pattern))
    return sorted(ports)


class Probe:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 15.0) -> None:
        if serial is None:
            raise ProtocolError("pyserial is required: python3 -m pip install pyserial")
        self.port = port
        self._sequence = 1
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)
        time.sleep(0.25)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()

    def __enter__(self) -> "Probe":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def command(self, command: Command, payload: bytes = b"") -> bytes:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        self._ser.write(encode_frame(command, sequence, payload))
        self._ser.flush()
        response = self._read_response()
        if response.sequence != sequence:
            raise ProtocolError(f"sequence mismatch: got {response.sequence}, expected {sequence}")
        if response.command != (int(command) | 0x80):
            raise ProtocolError(f"command mismatch: got 0x{response.command:02x}")
        if response.status != Status.OK:
            detail = ""
            if command != Command.GET_LAST_ERROR:
                try:
                    detail = self.command(Command.GET_LAST_ERROR).decode("ascii", "replace")
                except Exception:
                    detail = ""
            suffix = f": {detail}" if detail else ""
            raise ProtocolError(f"{command.name} failed: {response.status.name.lower()}{suffix}")
        return response.payload

    def _read_response(self) -> Response:
        header = self._ser.read(10)
        if len(header) != 10:
            raise ProtocolError("timeout waiting for response header")
        magic, version, command, sequence, length = struct.unpack("<IBBHH", header)
        if magic != MAGIC or version != VERSION:
            raise ProtocolError("bad response header")
        payload_crc = self._ser.read(length + 4)
        if len(payload_crc) != length + 4:
            raise ProtocolError("timeout waiting for response payload")
        return decode_response(header + payload_crc)

    def version(self) -> str:
        payload = self.command(Command.GET_VERSION)
        if len(payload) < 5:
            raise ProtocolError("malformed version response")
        major, minor, patch, proto = payload[:4]
        name = payload[4:].split(b"\x00", 1)[0].decode("ascii", "replace")
        return f"{name} {major}.{minor}.{patch} protocol {proto}"

    def set_pins(self, swim_pin: int, nrst_pin: int, pullup: bool) -> None:
        self.command(Command.SET_PINS, bytes([swim_pin, nrst_pin, 1 if pullup else 0]))

    def set_speed(self, high_speed: bool) -> None:
        self.command(Command.SET_SPEED, bytes([1 if high_speed else 0]))

    def enter_swim(self) -> None:
        self.command(Command.ENTER_SWIM)

    def entry_waveform(self, delay_ms: int) -> None:
        self.command(Command.ENTRY_WAVEFORM, struct.pack("<I", delay_ms))

    def swim_debug(self) -> SwimDebug:
        payload = self.command(Command.GET_SWIM_DEBUG)
        if len(payload) < 20:
            raise ProtocolError("malformed SWIM debug response")

        synced, speed_raw, csr_valid, csr = payload[:4]
        last_sync_low_us, last_sync_low_ns, tswim_ns, loop_count = struct.unpack("<IIII", payload[4:20])
        speed = "low" if speed_raw == 0 else "high"

        phy_backend = "unknown"
        pio_init_ok = False
        entry_slow_pulses = 0
        entry_fast_pulses = 0
        entry_protocol_us = 0
        pio_error = ""

        enter_stage = 0
        comm_reset_sent = False
        second_sync_seen = False
        comm_reset_low_us = 0
        comm_reset_low_ns = 0

        if len(payload) >= 29:
            backend_raw = payload[20]
            phy_backend = "pio" if backend_raw == 0 else "bitbang_fallback"
            pio_init_ok = bool(payload[21])
            entry_slow_pulses = payload[22]
            entry_fast_pulses = payload[23]
            entry_protocol_us = struct.unpack("<I", payload[24:28])[0]

            # New extended layout:
            # 28 enter_stage
            # 29 comm_reset_sent
            # 30 second_sync_seen
            # 31 reserved
            # 32..35 comm_reset_low_us
            # 36..39 comm_reset_low_ns
            # 40 pio_error_len
            # 41.. pio_error
            if len(payload) >= 41:
                enter_stage = payload[28]
                comm_reset_sent = bool(payload[29])
                second_sync_seen = bool(payload[30])
                comm_reset_low_us = struct.unpack("<I", payload[32:36])[0]
                comm_reset_low_ns = struct.unpack("<I", payload[36:40])[0]

                error_len = payload[40]
                if 41 + error_len <= len(payload):
                    pio_error = payload[41 : 41 + error_len].decode("ascii", "replace")
            else:
                # Old layout:
                # 28 pio_error_len
                # 29.. pio_error
                error_len = payload[28]
                if 29 + error_len <= len(payload):
                    pio_error = payload[29 : 29 + error_len].decode("ascii", "replace")

        return SwimDebug(
            synced=bool(synced),
            speed=speed,
            swim_csr_valid=bool(csr_valid),
            swim_csr=csr,
            last_sync_low_us=last_sync_low_us,
            last_sync_low_ns=last_sync_low_ns,
            derived_tswim_ns=tswim_ns,
            sync_low_loop_count=loop_count,
            phy_backend=phy_backend,
            entry_protocol_us=entry_protocol_us,
            entry_slow_pulses=entry_slow_pulses,
            entry_fast_pulses=entry_fast_pulses,
            pio_init_ok=pio_init_ok,
            pio_error=pio_error,
            enter_stage=enter_stage,
            comm_reset_sent=comm_reset_sent,
            second_sync_seen=second_sync_seen,
            comm_reset_low_us=comm_reset_low_us,
            comm_reset_low_ns=comm_reset_low_ns,
        )

    def reset_target(self) -> None:
        self.command(Command.RESET_TARGET)

    def read_memory(self, address: int, length: int) -> bytes:
        chunks: list[bytes] = []
        done = 0
        while done < length:
            chunk = min(MAX_PAYLOAD, length - done)
            payload = struct.pack("<IH", address + done, chunk)
            chunks.append(self.command(Command.MEMORY_READ, payload))
            done += chunk
        return b"".join(chunks)

    def write_memory(self, address: int, data: bytes) -> None:
        done = 0
        while done < len(data):
            chunk = data[done : done + MAX_PAYLOAD - 6]
            payload = struct.pack("<IH", address + done, len(chunk)) + chunk
            self.command(Command.MEMORY_WRITE, payload)
            done += len(chunk)

    def flash_erase(self, address: int, length: int) -> None:
        self.command(Command.FLASH_ERASE, struct.pack("<II", address, length))

    def flash_write_block(self, address: int, data: bytes) -> None:
        self.command(Command.FLASH_WRITE_BLOCK, struct.pack("<IH", address, len(data)) + data)


def autodetect_port() -> str:
    ports = list_serial_ports()
    if not ports:
        raise ProtocolError("no serial ports found")
    if len(ports) > 1:
        raise ProtocolError("multiple serial ports found; pass --port")
    return ports[0]


def iter_blocks(data: bytes, block_size: int) -> Iterable[bytes]:
    for offset in range(0, len(data), block_size):
        yield data[offset : offset + block_size]
