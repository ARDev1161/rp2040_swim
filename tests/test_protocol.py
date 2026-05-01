import struct

import pytest

from host.protocol import Command, ProtocolError, Status, crc32, decode_response, encode_frame


def test_encode_frame_crc() -> None:
    frame = encode_frame(Command.GET_VERSION, 7, b"abc")
    assert frame[:4] == struct.pack("<I", 0x53575052)
    assert struct.unpack("<I", frame[-4:])[0] == crc32(frame[:-4])


def test_decode_response() -> None:
    payload = struct.pack("<H", Status.OK) + b"ok"
    header = struct.pack("<IBBHH", 0x53575052, 1, 0x81, 9, len(payload))
    frame = header + payload + struct.pack("<I", crc32(header + payload))
    response = decode_response(frame)
    assert response.command == 0x81
    assert response.sequence == 9
    assert response.status == Status.OK
    assert response.payload == b"ok"


def test_decode_bad_crc() -> None:
    frame = bytearray(encode_frame(Command.GET_VERSION, 1))
    frame[5] = 0x81
    frame[8] = 2
    frame[10:10] = b"\x00\x00"
    with pytest.raises(ProtocolError):
        decode_response(bytes(frame))
