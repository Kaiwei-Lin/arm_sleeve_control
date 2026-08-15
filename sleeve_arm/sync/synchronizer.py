from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from sleeve_arm.domain.sensor import ImuFrame, SensorSample, SleeveFrame


@dataclass(frozen=True, slots=True)
class SynchronizerStats:
    samples_created: int = 0
    imu1_matched: int = 0
    imu1_missed: int = 0
    imu2_matched: int = 0
    imu2_missed: int = 0


class SensorSynchronizer:
    """Match optional IMUs to Sleeve frames using host monotonic timestamps."""

    def __init__(self, max_time_delta_ms: float = 20.0, buffer_duration_ms: float = 500.0, imu1_enabled: bool = False, imu2_enabled: bool = False) -> None:
        if max_time_delta_ms < 0 or buffer_duration_ms <= 0:
            raise ValueError("sync time values must be non-negative/positive")
        if max_time_delta_ms > buffer_duration_ms:
            raise ValueError("max_time_delta_ms cannot exceed buffer_duration_ms")
        self.max_delta_s = max_time_delta_ms / 1000.0
        self.buffer_duration_s = buffer_duration_ms / 1000.0
        self.imu1_enabled = imu1_enabled
        self.imu2_enabled = imu2_enabled
        self._imu1: deque[ImuFrame] = deque()
        self._imu2: deque[ImuFrame] = deque()
        self.stats = SynchronizerStats()

    def add_imu1(self, frame: ImuFrame) -> None:
        self._append(self._imu1, frame)

    def add_imu2(self, frame: ImuFrame) -> None:
        self._append(self._imu2, frame)

    def synchronize(self, sleeve: SleeveFrame) -> SensorSample:
        self._prune(self._imu1, sleeve.timestamp)
        self._prune(self._imu2, sleeve.timestamp)
        imu1 = self._nearest(self._imu1, sleeve.timestamp) if self.imu1_enabled else None
        imu2 = self._nearest(self._imu2, sleeve.timestamp) if self.imu2_enabled else None
        old = self.stats
        self.stats = SynchronizerStats(
            samples_created=old.samples_created + 1,
            imu1_matched=old.imu1_matched + int(self.imu1_enabled and imu1 is not None),
            imu1_missed=old.imu1_missed + int(self.imu1_enabled and imu1 is None),
            imu2_matched=old.imu2_matched + int(self.imu2_enabled and imu2 is not None),
            imu2_missed=old.imu2_missed + int(self.imu2_enabled and imu2 is None),
        )
        return SensorSample(sleeve.timestamp, sleeve, imu1, imu2)

    def buffer_sizes(self) -> tuple[int, int]:
        return len(self._imu1), len(self._imu2)

    def latest_imu_pair(
        self,
        max_delta_ms: float | None = None,
    ) -> tuple[ImuFrame, ImuFrame] | None:
        """Return the newest mutually synchronized pair, independent of Sleeve timing."""
        if not self.imu1_enabled or not self.imu2_enabled or not self._imu1 or not self._imu2:
            return None
        if max_delta_ms is None:
            return self._imu1[-1], self._imu2[-1]
        max_delta_s = max_delta_ms / 1000.0
        if max_delta_s < 0:
            raise ValueError("max_delta_ms must be non-negative")
        left = len(self._imu1) - 1
        right = len(self._imu2) - 1
        while left >= 0 and right >= 0:
            imu1, imu2 = self._imu1[left], self._imu2[right]
            gap = imu1.timestamp - imu2.timestamp
            if abs(gap) <= max_delta_s:
                return imu1, imu2
            if gap > 0:
                left -= 1
            else:
                right -= 1
        return None

    def _append(self, buffer: deque[ImuFrame], frame: ImuFrame) -> None:
        if buffer and frame.timestamp < buffer[-1].timestamp:
            raise ValueError("IMU frames must be appended in timestamp order")
        if buffer and frame.timestamp == buffer[-1].timestamp:
            return
        buffer.append(frame)
        self._prune(buffer, frame.timestamp)

    def _prune(self, buffer: deque[ImuFrame], reference: float) -> None:
        cutoff = reference - self.buffer_duration_s
        while buffer and buffer[0].timestamp < cutoff:
            buffer.popleft()

    def _nearest(self, buffer: deque[ImuFrame], timestamp: float) -> ImuFrame | None:
        # ponytail: the deque is bounded to milliseconds; use a linear scan until profiling says otherwise.
        frame = min(buffer, key=lambda item: abs(item.timestamp - timestamp), default=None)
        if frame is None or abs(frame.timestamp - timestamp) > self.max_delta_s:
            return None
        return frame
