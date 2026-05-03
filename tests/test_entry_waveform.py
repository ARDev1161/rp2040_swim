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
    limits_source = (REPO_ROOT / "firmware/src/swim_sync_limits.h").read_text()
    assert "#define SWIM_SYNC_IGNORE_SHORT_US 8u" in limits_source
    assert "#define SWIM_SYNC_MIN_US 12u" in limits_source
    assert "#define SWIM_SYNC_MAX_US 300u" in limits_source
    assert "swim_sync_low_width_is_plausible_us" in limits_source
    assert "swim_sync_low_width_is_plausible_us(elapsed_us)" in phy_source


def test_comm_reset_wait_uses_pio_tick_tx_and_rx_capture() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    function = source[source.index("bool swim_phy_comm_reset_wait_sync"):]
    function = function[:function.index("bool swim_phy_write_bit")]
    assert "swim_pio_emit_tick_segments_capture_response" in function
    assert ".duration_ticks = SWIM_SYNC_CLOCKS" in function
    assert ".level = SWIM_SEG_RELEASE" not in function
    assert "record_sync_measurement_ns(width.low_ns, width.loops_used);" in function


def test_pio_state_machines_restart_at_program_start() -> None:
    tx_source = (REPO_ROOT / "firmware/src/swim_pio_waveform.c").read_text()
    rx_source = (REPO_ROOT / "firmware/src/swim_pio_rx.c").read_text()
    assert "restart_tx_sm_at_program_start" in tx_source
    assert "pio_encode_jmp(g_pio.offset)" in tx_source
    assert "restart_rx_sm_at_program_start" in rx_source
    assert "pio_encode_jmp(g_rx_offset)" in rx_source


def test_ack_after_pio_tx_uses_pio_rx_width_capture() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    function = source[source.index("bool swim_phy_write_frame_bits_read_ack"):]
    function = function[:function.index("bool swim_phy_read_bit")]
    assert "swim_pio_emit_tick_segments_capture_response" in function
    assert "swim_pio_emit_tick_segments_wait_response" not in function
    assert "response.low_ns" in function


def test_read_bit_uses_pio_rx_width_capture() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    function = source[source.index("bool swim_phy_read_bit"):]
    function = function[:function.index("bool swim_phy_wait_sync")]
    assert "swim_pio_rx_arm_now" in function
    assert "swim_pio_rx_get_width" in function
    assert "width.low_ns" in function


def test_recv_byte_after_ack_uses_immediate_ack_target_frame_capture() -> None:
    link_source = (REPO_ROOT / "firmware/src/swim_link.c").read_text()
    recv = link_source[link_source.index("static rpsw_status_t recv_byte_after_ack"):]
    recv = recv[:recv.index("static rpsw_status_t send_byte_read_ack_frame_labeled")]
    assert "swim_phy_write_bit_read_target_frame(true, SWIM_READ_TIMEOUT_US, &frame)" in recv
    assert "decode_data_frame_no_ack(frame, byte)" in recv
    assert "send_target_frame_ack(false)" in recv


def test_rotf_captures_first_data_byte_with_final_address_ack() -> None:
    source = (REPO_ROOT / "firmware/src/swim_link.c").read_text()
    read_fn = source[source.index("rpsw_status_t swim_link_read"):]
    read_fn = read_fn[:read_fn.index("rpsw_status_t swim_link_write")]
    assert "send_byte_read_ack_frame_labeled((uint8_t)(address & 0xffu), &data[0], \"ROTF AL\")" in read_fn
    assert "for (size_t i = 1; i < len; i++)" in read_fn
    assert "recv_byte_after_ack(&data[i])" in read_fn
    assert "single immediate SWIM bit" in read_fn
    assert "send_target_frame_ack(true)" in read_fn


def test_pio_rx_switches_between_width_and_decode_programs() -> None:
    source = (REPO_ROOT / "firmware/src/swim_pio_rx.c").read_text()
    assert '#include "swim_rx_width.pio.h"' in source
    assert '#include "swim_rx_decode.pio.h"' in source
    assert "SWIM_RX_PROGRAM_WIDTH" in source
    assert "SWIM_RX_PROGRAM_DECODE" in source
    assert "switch_rx_program" in source
    assert "configure_width_sms" in source
    assert "configure_decode_sm" in source


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
    assert "SWIM_PIO_SEGMENT_OVERHEAD_TICKS" in c_source
    assert "pio_interrupt_get" in c_source


def test_flash_erase_and_write_mvp_is_enabled() -> None:
    source = (REPO_ROOT / "firmware/src/stm8_flash.c").read_text()
    header = (REPO_ROOT / "firmware/src/stm8_flash.h").read_text()
    assert "RPSW_ERR_FLASH_GUARD" not in source
    assert "const char *stm8_flash_last_error(void)" in header
    assert "g_flash_last_error" in source
    assert "#define STM8_FLASH_BLOCK_SIZE   64u" in source
    assert "#define STM8_FLASH_ERASED_VALUE 0x00u" in source
    assert "#define STM8_CR2_ERASE   0x20u" in source
    assert "stm8_dm_memory_write(block, erase_block, STM8_FLASH_BLOCK_SIZE)" in source
    assert "st == RPSW_ERR_SWIM_TIMEOUT || st == RPSW_ERR_SWIM_NACK" in source
    assert "flash_busy_delay_and_resync(30000u)" in source
    assert "Use conservative byte programming for the MVP" in source


def test_flash_commands_report_detailed_firmware_errors() -> None:
    main_source = (REPO_ROOT / "firmware/src/main.c").read_text()
    protocol_source = (REPO_ROOT / "host/protocol.py").read_text()
    assert "stm8_flash_last_error()" in main_source
    assert "snprintf(g_last_error, sizeof(g_last_error), \"%s: %s\"" in main_source
    assert "self.command(Command.GET_LAST_ERROR).decode" in protocol_source
    assert "suffix = f\": {detail}\" if detail else \"\"" in protocol_source


def test_flash_write_block_payload_length_is_validated() -> None:
    source = (REPO_ROOT / "firmware/src/main.c").read_text()
    write_case = source[source.index("case CMD_FLASH_WRITE_BLOCK") : source.index("case CMD_FLASH_VERIFY")]
    assert "uint16_t len = rpsw_get_u16le(&request->payload[4]);" in write_case
    assert "request->length != (uint16_t)(6u + len)" in write_case
    assert "return RPSW_ERR_BAD_ARGUMENT;" in write_case


def test_host_unlock_writes_keys_as_two_single_byte_transactions() -> None:
    source = (REPO_ROOT / "host/rp2040_swim.py").read_text()
    helper = source[source.index("def write_key_pair_same_register") : source.index("def wait_iapsr_mask")]
    assert "probe.write_memory(key_reg, bytes([key1]))" in helper
    assert "probe.write_memory(key_reg, bytes([key2]))" in helper
    assert "bytes([key1, key2])" not in helper
    assert "unlock_program_flash(probe)" in source
    assert "fill=STM8_FLASH_ERASED_VALUE" in source


def test_host_protocol_uses_long_timeout_for_full_flash_erase() -> None:
    source = (REPO_ROOT / "host/protocol.py").read_text()
    assert "timeout: float = 15.0" in source


def test_high_speed_remains_disabled() -> None:
    source = (REPO_ROOT / "firmware/src/swim_phy.c").read_text()
    assert "speed != SWIM_SPEED_LOW" in source
    assert "return false" in source
