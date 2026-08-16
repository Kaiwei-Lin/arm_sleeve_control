from __future__ import annotations

import time
from collections import deque
from collections.abc import Callable, Sequence

from sleeve_arm.domain.sensor import ImuFrame
from sleeve_arm.sources.base import ImuSource, SourceStats


class FakeImuSource(ImuSource):
    def __init__(
        self,
        frequency_hz: float = 100.0,
        timestamp_offset_s: float = 0.0,
        quaternion_fn: Callable[[float], Sequence[float]] | None = None,
    ) -> None:
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be positive")
        self.frequency_hz = frequency_hz
        self.timestamp_offset_s = timestamp_offset_s
        self.quaternion_fn = quaternion_fn
        self.running = False
        self._started_at = 0.0
        self._last_index = -1
        self._latest: ImuFrame | None = None
        self._pending: deque[ImuFrame] = deque()

    def start(self) -> None:
        self.running = True
        self._started_at = time.monotonic()
        self._last_index = -1
        self._latest = None
        self._pending.clear()

    def latest(self) -> ImuFrame | None:
        if not self.running:
            raise RuntimeError("fake IMU is not started")
        self._generate()
        frame = self._latest
        self._pending.clear()
        return frame

    def drain(self) -> tuple[ImuFrame, ...]:
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
            elapsed_s = sample_index / self.frequency_hz
            timestamp = self._started_at + elapsed_s + self.timestamp_offset_s
            host_timestamp_ns = round(timestamp * 1_000_000_000)
            quaternion = (
                (1.0, 0.0, 0.0, 0.0)
                if self.quaternion_fn is None
                else tuple(float(value) for value in self.quaternion_fn(elapsed_s))
            )
            if len(quaternion) != 4:
                raise ValueError("quaternion_fn must return four WXYZ values")
            frame = ImuFrame(
                timestamp,
                0.0,
                0.0,
                9.81,
                0.0,
                0.0,
                float(sample_index),
                *quaternion,
                sample_index + 1,
                round(elapsed_s * 1_000_000),
                host_timestamp_ns,
            )
            self._latest = frame
            self._pending.append(frame)
        self._last_index = max(self._last_index, index)
