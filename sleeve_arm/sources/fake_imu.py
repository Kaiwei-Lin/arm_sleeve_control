from __future__ import annotations

import time

from sleeve_arm.domain.sensor import ImuFrame
from sleeve_arm.sources.base import ImuSource, SourceStats


class FakeImuSource(ImuSource):
    def __init__(self, frequency_hz: float = 100.0, timestamp_offset_s: float = 0.0) -> None:
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        self.frequency_hz = frequency_hz
        self.timestamp_offset_s = timestamp_offset_s
        self.running = False
        self._started_at = 0.0
        self._last_index = -1
        self._latest: ImuFrame | None = None

    def start(self) -> None:
        self.running = True
        self._started_at = time.monotonic()
        self._last_index = -1
        self._latest = None

    def latest(self) -> ImuFrame | None:
        if not self.running:
            raise RuntimeError("fake IMU is not started")
        now = time.monotonic()
        index = int((now - self._started_at) * self.frequency_hz)
        if index != self._last_index:
            timestamp = now + self.timestamp_offset_s
            self._latest = ImuFrame(timestamp, 0.0, 0.0, 9.81, 0.0, 0.0, float(index), 1.0, 0.0, 0.0, 0.0, index + 1)
            self._last_index = index
        return self._latest

    @property
    def stats(self) -> SourceStats:
        count = max(0, self._last_index + 1)
        timestamp = None if self._latest is None else self._latest.timestamp
        return SourceStats(count, 0, timestamp, self.frequency_hz if count > 1 else 0.0)

    def close(self) -> None:
        self.running = False
