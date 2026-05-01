from __future__ import annotations


SYNC_CLOCKS = 128
LOW_SPEED_ONE_LOW_CLOCKS = 2
LOW_SPEED_ONE_HIGH_CLOCKS = 20
LOW_SPEED_ZERO_LOW_CLOCKS = 20
LOW_SPEED_ZERO_HIGH_CLOCKS = 2
LOW_SPEED_ONE_MAX_LOW_CLOCKS = 8
LOW_SPEED_ZERO_MIN_LOW_CLOCKS = 9


def sync_low_to_tswim_ns(sync_low_ns: int) -> int:
    if sync_low_ns <= 0:
        raise ValueError("sync_low_ns must be positive")
    return (sync_low_ns + SYNC_CLOCKS // 2) // SYNC_CLOCKS


def low_speed_bit_durations_ns(tswim_ns: int, bit: int) -> tuple[int, int]:
    if tswim_ns <= 0:
        raise ValueError("tswim_ns must be positive")
    if bit not in (0, 1):
        raise ValueError("bit must be 0 or 1")
    if bit:
        return (
            tswim_ns * LOW_SPEED_ONE_LOW_CLOCKS,
            tswim_ns * LOW_SPEED_ONE_HIGH_CLOCKS,
        )
    return (
        tswim_ns * LOW_SPEED_ZERO_LOW_CLOCKS,
        tswim_ns * LOW_SPEED_ZERO_HIGH_CLOCKS,
    )


def classify_low_pulse_clocks(low_clocks: int) -> int:
    if low_clocks < 0:
        raise ValueError("low_clocks must be non-negative")
    if low_clocks <= LOW_SPEED_ONE_MAX_LOW_CLOCKS:
        return 1
    if low_clocks >= LOW_SPEED_ZERO_MIN_LOW_CLOCKS:
        return 0
    raise ValueError("unreachable SWIM low-speed threshold gap")


def classify_low_pulse_counts(low_count: int, sync_low_count: int) -> int:
    if low_count < 0:
        raise ValueError("low_count must be non-negative")
    if sync_low_count <= 0:
        raise ValueError("sync_low_count must be positive")
    low_clocks = (low_count * SYNC_CLOCKS + sync_low_count // 2) // sync_low_count
    return classify_low_pulse_clocks(low_clocks)
