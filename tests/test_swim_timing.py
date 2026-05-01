import pytest

from host.swim_timing import (
    classify_low_pulse_clocks,
    classify_low_pulse_counts,
    low_speed_bit_durations_ns,
    sync_low_to_tswim_ns,
)


def test_sync_low_to_tswim_ns_nominal_16mhz_hsi_div2() -> None:
    assert sync_low_to_tswim_ns(16_000) == 125


def test_sync_low_to_tswim_ns_rounds_to_nearest_clock() -> None:
    assert sync_low_to_tswim_ns(16_063) == 125
    assert sync_low_to_tswim_ns(16_064) == 126


def test_low_speed_bit_durations_use_um0470_ratios() -> None:
    assert low_speed_bit_durations_ns(125, 1) == (250, 2500)
    assert low_speed_bit_durations_ns(125, 0) == (2500, 250)


@pytest.mark.parametrize(("low_clocks", "bit"), [(0, 1), (2, 1), (8, 1), (9, 0), (20, 0)])
def test_classify_low_speed_thresholds(low_clocks: int, bit: int) -> None:
    assert classify_low_pulse_clocks(low_clocks) == bit


def test_classify_low_speed_counts_against_sync_count() -> None:
    assert classify_low_pulse_counts(20, 1280) == 1
    assert classify_low_pulse_counts(200, 1280) == 0


def test_invalid_timing_inputs() -> None:
    with pytest.raises(ValueError):
        sync_low_to_tswim_ns(0)
    with pytest.raises(ValueError):
        low_speed_bit_durations_ns(0, 1)
    with pytest.raises(ValueError):
        classify_low_pulse_counts(1, 0)
