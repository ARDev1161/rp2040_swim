import pytest

from host.ihex import HexError, image_for_range, parse_ihex


def test_parse_basic_record() -> None:
    data = parse_ihex(":0400000001020304F2\n:00000001FF\n")
    assert data == {0: 1, 1: 2, 2: 3, 3: 4}


def test_parse_extended_linear_address() -> None:
    text = ":020000040001F9\n:0400100001020304E2\n:00000001FF\n"
    data = parse_ihex(text)
    assert data[0x10010] == 1
    assert data[0x10013] == 4


def test_checksum_error() -> None:
    with pytest.raises(HexError):
        parse_ihex(":0400000001020304F3\n:00000001FF\n")


def test_image_range_guard() -> None:
    with pytest.raises(HexError):
        image_for_range([type("SegmentLike", (), {"address": 0x7FFF, "data": b"x"})()], 0x8000, 4)
