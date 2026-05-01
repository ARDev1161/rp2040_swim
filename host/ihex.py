from __future__ import annotations

from dataclasses import dataclass


class HexError(ValueError):
    pass


@dataclass(frozen=True)
class Segment:
    address: int
    data: bytes


def _parse_byte(text: str) -> int:
    try:
        return int(text, 16)
    except ValueError as exc:
        raise HexError(f"invalid hex byte {text!r}") from exc


def parse_ihex(text: str) -> dict[int, int]:
    memory: dict[int, int] = {}
    upper_linear = 0
    upper_segment = 0
    eof = False

    for lineno, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        if eof:
            raise HexError(f"data after EOF at line {lineno}")
        if not line.startswith(":"):
            raise HexError(f"line {lineno}: missing ':'")
        if len(line) < 11 or len(line[1:]) % 2 != 0:
            raise HexError(f"line {lineno}: malformed record length")

        payload = bytes(_parse_byte(line[i : i + 2]) for i in range(1, len(line), 2))
        count = payload[0]
        if len(payload) != count + 5:
            raise HexError(f"line {lineno}: byte count mismatch")
        if (sum(payload) & 0xFF) != 0:
            raise HexError(f"line {lineno}: checksum mismatch")

        offset = (payload[1] << 8) | payload[2]
        rectype = payload[3]
        data = payload[4 : 4 + count]

        if rectype == 0x00:
            base = upper_linear + upper_segment
            for i, b in enumerate(data):
                address = base + offset + i
                if address in memory and memory[address] != b:
                    raise HexError(f"line {lineno}: conflicting byte at 0x{address:08x}")
                memory[address] = b
        elif rectype == 0x01:
            if count != 0:
                raise HexError(f"line {lineno}: EOF record must be empty")
            eof = True
        elif rectype == 0x02:
            if count != 2:
                raise HexError(f"line {lineno}: bad extended segment record")
            upper_segment = (((data[0] << 8) | data[1]) << 4)
            upper_linear = 0
        elif rectype == 0x04:
            if count != 2:
                raise HexError(f"line {lineno}: bad extended linear record")
            upper_linear = (((data[0] << 8) | data[1]) << 16)
            upper_segment = 0
        elif rectype in (0x03, 0x05):
            continue
        else:
            raise HexError(f"line {lineno}: unsupported record type 0x{rectype:02x}")

    if not eof:
        raise HexError("missing EOF record")
    return memory


def load_ihex(path: str) -> list[Segment]:
    with open(path, "r", encoding="ascii") as f:
        memory = parse_ihex(f.read())
    if not memory:
        return []

    segments: list[Segment] = []
    addresses = sorted(memory)
    start = prev = addresses[0]
    buf = bytearray([memory[start]])
    for address in addresses[1:]:
        if address == prev + 1:
            buf.append(memory[address])
        else:
            segments.append(Segment(start, bytes(buf)))
            start = address
            buf = bytearray([memory[address]])
        prev = address
    segments.append(Segment(start, bytes(buf)))
    return segments


def image_for_range(segments: list[Segment], start: int, size: int, fill: int = 0xFF) -> bytes:
    image = bytearray([fill] * size)
    end = start + size
    for segment in segments:
        seg_end = segment.address + len(segment.data)
        if segment.address < start or seg_end > end:
            raise HexError(
                f"segment 0x{segment.address:08x}-0x{seg_end - 1:08x} outside "
                f"range 0x{start:08x}-0x{end - 1:08x}"
            )
        offset = segment.address - start
        image[offset : offset + len(segment.data)] = segment.data
    return bytes(image)
