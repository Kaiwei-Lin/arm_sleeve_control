from __future__ import annotations

import math
import time
from collections import deque

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
        self._pending: deque[SleeveFrame] = deque()

    def start(self) -> None:
        self.running = True
        self._started_at = time.monotonic()
        self._last_index = -1
        self._latest = None
        self._pending.clear()

    def latest(self) -> SleeveFrame | None:
        if not self.running:
            raise RuntimeError("fake sleeve is not started")
        self._generate()
        frame = self._latest
        self._pending.clear()
        return frame

    def drain(self) -> tuple[SleeveFrame, ...]:
        if self.running:
            self._generate()
        frames = tuple(self._pending)
        self._pending.clear()
        return frames

    @property
    def stats(self) -> SourceStats:
        count = max(0, self._last_index + 1)
        timestamp = None if self._latest is None else self._latest.timestamp
        return SourceStats(count, 0, timestamp, self.frequency_hz if count > 1 else 0.0)

    def close(self) -> None:
        if self.running:
            self._generate()
        self.running = False

    def _generate(self) -> None:
        now = time.monotonic()
        index = int((now - self._started_at) * self.frequency_hz)
        for sample_index in range(self._last_index + 1, index + 1):
            timestamp = self._started_at + sample_index / self.frequency_hz
            host_timestamp_ns = round(timestamp * 1_000_000_000)
            value = math.sin(sample_index * 0.1)
            frame = SleeveFrame(
                timestamp,
                tuple(value + channel for channel in range(self.channel_count)),
                sample_index + 1,
                host_timestamp_ns,
            )
            self._latest = frame
            self._pending.append(frame)
        self._last_index = max(self._last_index, index)
