from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class SegmentLevel(IntEnum):
    RELEASE = 0
    LOW = 1


@dataclass(frozen=True)
class EntrySegment:
    level: SegmentLevel
    duration_us: int


ENTRY_SLOW_PULSES = 4
ENTRY_FAST_PULSES = 4
ENTRY_SLOW_PERIOD_US = 1000
ENTRY_FAST_PERIOD_US = 500
ENTRY_INITIAL_LOW_US = 16
ENTRY_PROTOCOL_US = 6016


def um0470_entry_segments() -> list[EntrySegment]:
    segments = [
        EntrySegment(SegmentLevel.RELEASE, 10),
        EntrySegment(SegmentLevel.LOW, ENTRY_INITIAL_LOW_US),
    ]
    for _ in range(ENTRY_SLOW_PULSES):
        segments.extend(
            [
                EntrySegment(SegmentLevel.RELEASE, 500),
                EntrySegment(SegmentLevel.LOW, 500),
            ]
        )
    for _ in range(ENTRY_FAST_PULSES):
        segments.extend(
            [
                EntrySegment(SegmentLevel.RELEASE, 250),
                EntrySegment(SegmentLevel.LOW, 250),
            ]
        )
    segments.append(EntrySegment(SegmentLevel.RELEASE, 10))
    return segments


def protocol_duration_us(segments: list[EntrySegment]) -> int:
    return sum(segment.duration_us for segment in segments[1:-1])
