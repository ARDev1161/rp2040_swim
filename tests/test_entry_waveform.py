from pathlib import Path
import sys

from host.entry_waveform import (
    ENTRY_FAST_PULSES,
    ENTRY_INITIAL_LOW_US,
    ENTRY_PROTOCOL_US,
    ENTRY_SLOW_PULSES,
    SegmentLevel,
    protocol_duration_us,
    um0470_entry_segments,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "host"))

from rp2040_swim import build_parser  # noqa: E402


def test_um0470_entry_segment_list_exact() -> None:
    segments = um0470_entry_segments()
    assert segments[0].level == SegmentLevel.RELEASE
    assert segments[0].duration_us == 10
    assert segments[1].level == SegmentLevel.LOW
    assert segments[1].duration_us == ENTRY_INITIAL_LOW_US

    slow = segments[2:10]
    assert len(slow) == ENTRY_SLOW_PULSES * 2
    for i in range(0, len(slow), 2):
        assert slow[i].level == SegmentLevel.LOW
        assert slow[i].duration_us == 500
        assert slow[i + 1].level == SegmentLevel.RELEASE
        assert slow[i + 1].duration_us == 500

    fast = segments[10:18]
    assert len(fast) == ENTRY_FAST_PULSES * 2
    for i in range(0, len(fast), 2):
        assert fast[i].level == SegmentLevel.LOW
        assert fast[i].duration_us == 250
        assert fast[i + 1].level == SegmentLevel.RELEASE
        assert fast[i + 1].duration_us == 250

    assert segments[-1].level == SegmentLevel.RELEASE
    assert segments[-1].duration_us == 10


def test_um0470_entry_protocol_duration_excludes_settle_segments() -> None:
    assert protocol_duration_us(um0470_entry_segments()) == ENTRY_PROTOCOL_US


def test_entry_waveform_command_exists() -> None:
    parser = build_parser()
    args = parser.parse_args(["entry-waveform", "--port", "/dev/null", "--delay-ms", "1"])
    assert args.command == "entry-waveform"
    assert args.delay_ms == 1


def test_flash_guards_remain_enabled() -> None:
    source = (REPO_ROOT / "firmware/src/stm8_flash.c").read_text()
    assert "RPSW_ERR_FLASH_GUARD" in source
    assert source.count("RPSW_ERR_FLASH_GUARD") >= 2


def test_high_speed_remains_disabled() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    assert "speed != SWIM_SPEED_LOW" in source
    assert "return false" in source
