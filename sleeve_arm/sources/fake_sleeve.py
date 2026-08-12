from __future__ import annotations

import math
import time

from sleeve_arm.domain.sensor import SleeveFrame
from sleeve_arm.sources.base import SleeveSource, SourceStats


class FakeSleeveSource(SleeveSource):
    def __init__(self, channel_count: int = 11, frequency_hz: float = 50.0) -> None:
        if channel_count <= 0 or frequency_hz <= 0:
            raise ValueError("channel_count and frequency_hz must be positive")
        self.channel_count = channel_count
        self.frequency_hz = frequency_hz
        self.running = False
        self._started_at = 0.0
        self._last_index = -1
        self._latest: SleeveFrame | None = None

    def start(self) -> None:
        self.running = True
        self._started_at = time.monotonic()
        self._last_index = -1
        self._latest = None

    def latest(self) -> SleeveFrame | None:
        if not self.running:
            raise RuntimeError("fake sleeve is not started")
        now = time.monotonic()
        index = int((now - self._started_at) * self.frequency_hz)
        if index != self._last_index:
            value = math.sin(index * 0.1)
            self._latest = SleeveFrame(now, tuple(value + channel for channel in range(self.channel_count)), index + 1)
            self._last_index = index
        return self._latest

    @property
    def stats(self) -> SourceStats:
        count = max(0, self._last_index + 1)
        timestamp = None if self._latest is None else self._latest.timestamp
        return SourceStats(count, 0, timestamp, self.frequency_hz if count > 1 else 0.0)

    def close(self) -> None:
        self.running = False
