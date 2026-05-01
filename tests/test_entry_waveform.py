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
        assert slow[i].level == SegmentLevel.RELEASE
        assert slow[i].duration_us == 500
        assert slow[i + 1].level == SegmentLevel.LOW
        assert slow[i + 1].duration_us == 500

    fast = segments[10:18]
    assert len(fast) == ENTRY_FAST_PULSES * 2
    for i in range(0, len(fast), 2):
        assert fast[i].level == SegmentLevel.RELEASE
        assert fast[i].duration_us == 250
        assert fast[i + 1].level == SegmentLevel.LOW
        assert fast[i + 1].duration_us == 250

    assert segments[-1].level == SegmentLevel.RELEASE
    assert segments[-1].duration_us == 10


def test_only_one_initial_low_before_slow_pulse_group() -> None:
    segments = um0470_entry_segments()
    assert [segment.level for segment in segments[:2]].count(SegmentLevel.LOW) == 1
    assert segments[1].level == SegmentLevel.LOW
    assert segments[1].duration_us == 16
    assert segments[2:10] == [
        item
        for _ in range(ENTRY_SLOW_PULSES)
        for item in (
            type(segments[0])(SegmentLevel.RELEASE, 500),
            type(segments[0])(SegmentLevel.LOW, 500),
        )
    ]


def test_firmware_entry_segment_list_has_single_16us_low() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    assert source.count("{SWIM_SEG_LOW, 16}") == 1
    assert source.index("{SWIM_SEG_LOW, 16}") < source.index("{SWIM_SEG_LOW, 500}")


def test_um0470_entry_protocol_duration_excludes_settle_segments() -> None:
    assert protocol_duration_us(um0470_entry_segments()) == ENTRY_PROTOCOL_US


def test_real_entry_path_skips_debug_end_settle_for_fast_sync() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    assert "return emit_entry_segments(entry_segment_count() - 1u);" in source
    assert "void swim_phy_entry_waveform(void)" in source
    assert "(void)emit_entry_segments(entry_segment_count());" in source


def test_enter_flow_restores_comm_reset_and_second_sync_before_csr() -> None:
    source = (REPO_ROOT / "firmware/src/swim_link.c").read_text()
    sync1 = source.index("SWIM_ENTER_STAGE_SYNC1_OK")
    comm_reset = source.index("swim_phy_comm_reset_wait_sync(SWIM_SYNC_TIMEOUT_US)")
    sync2 = source.index("SWIM_ENTER_STAGE_SYNC2_OK")
    csr_write = source.index("SWIM_ENTER_STAGE_SWIM_CSR_WRITE_START")
    assert sync1 < comm_reset < sync2 < csr_write
    assert "swim_phy_mark_second_sync_seen();" in source


def test_sync_detection_filters_short_lows_and_accepts_plausible_window() -> None:
    phy_source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    pio_source = (REPO_ROOT / "firmware/src/swim_pio_waveform.c").read_text()
    for source in (phy_source, pio_source):
        assert "#define SWIM_SYNC_IGNORE_SHORT_US 10u" in source
        assert "#define SWIM_SYNC_MIN_US 10u" in source
        assert "#define SWIM_SYNC_MAX_US 300u" in source
        assert "sync_low_width_is_plausible" in source


def test_comm_reset_wait_restores_irq_before_waiting_for_sync() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    function = source[source.index("bool swim_phy_comm_reset_wait_sync"):]
    function = function[:function.index("bool swim_phy_write_bit")]
    assert function.index("swim_phy_sio_release_fast();") < function.index("restore_interrupts(irq_state);")
    assert function.rindex("restore_interrupts(irq_state);") < function.rindex("return wait_sync_fast(timeout_us);")


def test_entry_waveform_command_exists() -> None:
    parser = build_parser()
    args = parser.parse_args(["entry-waveform", "--port", "/dev/null", "--delay-ms", "1"])
    assert args.command == "entry-waveform"
    assert args.delay_ms == 1


def test_entry_waveform_command_prints_segment_list(capsys) -> None:
    import rp2040_swim  # noqa: PLC0415

    class FakeDebug:
        phy_backend = "pio"
        pio_init_ok = True
        pio_error = "ok"

    class FakeProbe:
        def __init__(self, args) -> None:
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_exc: object) -> None:
            return None

        def version(self) -> str:
            return "fake 0.0.0 protocol 1"

        def entry_waveform(self, delay_ms: int) -> None:
            assert delay_ms == 1

        def swim_debug(self) -> FakeDebug:
            return FakeDebug()

    parser = build_parser()
    args = parser.parse_args(["entry-waveform", "--port", "/dev/null", "--delay-ms", "1"])
    original_open_probe = rp2040_swim.open_probe
    rp2040_swim.open_probe = FakeProbe
    try:
        assert args.func(args) == 0
    finally:
        rp2040_swim.open_probe = original_open_probe
    out = capsys.readouterr().out
    assert "segment[0] level=release requested_duration_us=10" in out
    assert "segment[1] level=low requested_duration_us=16" in out
    assert out.count("requested_duration_us=16") == 1


def test_pio_waveform_is_open_drain_and_deterministic() -> None:
    pio_source = (REPO_ROOT / "firmware/pio/swim_waveform.pio").read_text()
    c_source = (REPO_ROOT / "firmware/src/swim_pio_waveform.c").read_text()
    assert "set pindirs, 1" in pio_source
    assert "set pindirs, 0" in pio_source
    assert "set pins, 1" not in pio_source
    assert "pio_sm_set_pins_with_mask" in c_source
    assert "pio_sm_set_pindirs_with_mask" in c_source
    assert "pio_gpio_init(g_pio.pio, g_pio.swim_pin)" in c_source
    assert "#define SWIM_PIO_TICKS_PER_US 10u" in c_source
    assert "#define SWIM_PIO_SEGMENT_OVERHEAD_TICKS 8u" in c_source
    assert "pio_interrupt_get" in c_source


def test_flash_guards_remain_enabled() -> None:
    source = (REPO_ROOT / "firmware/src/stm8_flash.c").read_text()
    assert "RPSW_ERR_FLASH_GUARD" in source
    assert source.count("RPSW_ERR_FLASH_GUARD") >= 2


def test_high_speed_remains_disabled() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    assert "speed != SWIM_SPEED_LOW" in source
    assert "return false" in source
