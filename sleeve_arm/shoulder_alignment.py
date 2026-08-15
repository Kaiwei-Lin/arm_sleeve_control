from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from typing import Generic, TypeVar

import numpy as np

from sleeve_arm.domain import ImuFrame, SleeveFrame
from sleeve_arm.estimation import align_quaternion_sign, normalize_quaternion, quaternion_slerp


FrameT = TypeVar("FrameT", SleeveFrame, ImuFrame)


class TimestampBuffer(Generic[FrameT]):
    def __init__(self, duration_ms: float) -> None:
        self.duration_ns = round(float(duration_ms) * 1_000_000)
        if self.duration_ns <= 0:
            raise ValueError("buffer duration must be positive")
        self.frames: list[FrameT] = []
        self.timestamps: list[int] = []

    def add(self, frame: FrameT) -> None:
        timestamp_ns = int(frame.host_timestamp_ns)  # populated by the domain frame
        if not self.timestamps or timestamp_ns >= self.timestamps[-1]:
            self.timestamps.append(timestamp_ns)
            self.frames.append(frame)
        else:
            index = bisect_left(self.timestamps, timestamp_ns)
            self.timestamps.insert(index, timestamp_ns)
            self.frames.insert(index, frame)
        newest = self.timestamps[-1]
        self.prune(newest - self.duration_ns)

    def extend(self, frames: tuple[FrameT, ...] | list[FrameT]) -> None:
        for frame in frames:
            self.add(frame)

    def prune(self, before_ns: int) -> None:
        index = bisect_left(self.timestamps, int(before_ns))
        if index > 0:
            del self.timestamps[:index]
            del self.frames[:index]

    def neighbors(self, target_ns: int) -> tuple[FrameT | None, FrameT | None]:
        index = bisect_left(self.timestamps, int(target_ns))
        before = self.frames[index - 1] if index > 0 else None
        after = self.frames[index] if index < len(self.frames) else None
        if after is not None and int(after.host_timestamp_ns) == target_ns:
            return after, after
        return before, after

    def nearest(self, target_ns: int) -> FrameT | None:
        before, after = self.neighbors(target_ns)
        choices = [frame for frame in (before, after) if frame is not None]
        return min(
            choices,
            key=lambda frame: abs(int(frame.host_timestamp_ns) - target_ns),
            default=None,
        )


@dataclass(frozen=True, slots=True)
class AlignedValue:
    value: tuple[float, ...] | None
    skew_ms: float | None
    valid: bool


@dataclass(frozen=True, slots=True)
class AlignedInputs:
    timestamp_ns: int
    flex: AlignedValue
    torso: AlignedValue
    arm: AlignedValue

    @property
    def frame_valid(self) -> bool:
        return self.flex.valid and self.torso.valid and self.arm.valid


def _skew_ms(target_ns: int, *frames: SleeveFrame | ImuFrame) -> float:
    return min(abs(int(frame.host_timestamp_ns) - target_ns) for frame in frames) / 1_000_000.0


def _bracket_valid(
    before: SleeveFrame | ImuFrame,
    after: SleeveFrame | ImuFrame,
    target_ns: int,
    max_skew_ms: float,
) -> tuple[float, bool]:
    skew = _skew_ms(target_ns, before, after)
    span_ms = (int(after.host_timestamp_ns) - int(before.host_timestamp_ns)) / 1_000_000.0
    return skew, skew <= max_skew_ms and span_ms <= 2.0 * max_skew_ms


def interpolate_flex(
    buffer: TimestampBuffer[SleeveFrame],
    target_ns: int,
    method: str,
    max_skew_ms: float,
) -> AlignedValue:
    before, after = buffer.neighbors(target_ns)
    if method == "nearest":
        nearest = buffer.nearest(target_ns)
        if nearest is None:
            return AlignedValue(None, None, False)
        skew = _skew_ms(target_ns, nearest)
        return AlignedValue(nearest.channels, skew, skew <= max_skew_ms)
    if method != "linear":
        raise ValueError("flex interpolation must be linear or nearest")
    if before is None or after is None:
        nearest = buffer.nearest(target_ns)
        if nearest is None:
            return AlignedValue(None, None, False)
        return AlignedValue(nearest.channels, _skew_ms(target_ns, nearest), False)
    skew, bracket_valid = _bracket_valid(before, after, target_ns, max_skew_ms)
    if before is after:
        return AlignedValue(before.channels, skew, skew <= max_skew_ms)
    if len(before.channels) != len(after.channels):
        return AlignedValue(None, skew, False)
    span = int(after.host_timestamp_ns) - int(before.host_timestamp_ns)
    if span <= 0:
        return AlignedValue(before.channels, skew, False)
    fraction = (target_ns - int(before.host_timestamp_ns)) / span
    values = tuple(
        float(left + fraction * (right - left))
        for left, right in zip(before.channels, after.channels)
    )
    return AlignedValue(values, skew, bracket_valid)


def _frame_quaternion(frame: ImuFrame) -> tuple[float, float, float, float] | None:
    values = (frame.quat_w, frame.quat_x, frame.quat_y, frame.quat_z)
    if any(value is None for value in values):
        return None
    try:
        normalized = normalize_quaternion(values)  # type: ignore[arg-type]
    except ValueError:
        return None
    return tuple(float(value) for value in normalized)


def interpolate_imu(
    buffer: TimestampBuffer[ImuFrame],
    target_ns: int,
    max_skew_ms: float,
) -> AlignedValue:
    before, after = buffer.neighbors(target_ns)
    if before is None or after is None:
        nearest = buffer.nearest(target_ns)
        if nearest is None:
            return AlignedValue(None, None, False)
        return AlignedValue(
            _frame_quaternion(nearest),
            _skew_ms(target_ns, nearest),
            False,
        )
    skew, bracket_valid = _bracket_valid(before, after, target_ns, max_skew_ms)
    q0, q1 = _frame_quaternion(before), _frame_quaternion(after)
    if q0 is None or q1 is None:
        return AlignedValue(None, skew, False)
    if before is after:
        return AlignedValue(q0, skew, skew <= max_skew_ms)
    span = int(after.host_timestamp_ns) - int(before.host_timestamp_ns)
    if span <= 0:
        return AlignedValue(q0, skew, False)
    fraction = (target_ns - int(before.host_timestamp_ns)) / span
    interpolated = quaternion_slerp(q0, q1, fraction)
    return AlignedValue(
        tuple(float(value) for value in interpolated),
        skew,
        bracket_valid,
    )


class ShoulderAligner:
    def __init__(
        self,
        buffer_duration_ms: float,
        max_skew_ms: float,
        flex_method: str,
    ) -> None:
        self.max_skew_ms = float(max_skew_ms)
        self.flex_method = flex_method
        self.flex_buffer: TimestampBuffer[SleeveFrame] = TimestampBuffer(buffer_duration_ms)
        self.torso_buffer: TimestampBuffer[ImuFrame] = TimestampBuffer(buffer_duration_ms)
        self.arm_buffer: TimestampBuffer[ImuFrame] = TimestampBuffer(buffer_duration_ms)
        self._previous_torso: np.ndarray | None = None
        self._previous_arm: np.ndarray | None = None

    def add_flex(self, frames: tuple[SleeveFrame, ...] | list[SleeveFrame]) -> None:
        self.flex_buffer.extend(frames)

    def add_torso(self, frames: tuple[ImuFrame, ...] | list[ImuFrame]) -> None:
        self.torso_buffer.extend(frames)

    def add_arm(self, frames: tuple[ImuFrame, ...] | list[ImuFrame]) -> None:
        self.arm_buffer.extend(frames)

    def align(self, timestamp_ns: int) -> AlignedInputs:
        flex = interpolate_flex(
            self.flex_buffer,
            timestamp_ns,
            self.flex_method,
            self.max_skew_ms,
        )
        torso = interpolate_imu(self.torso_buffer, timestamp_ns, self.max_skew_ms)
        arm = interpolate_imu(self.arm_buffer, timestamp_ns, self.max_skew_ms)
        torso = self._continuous(torso, "torso")
        arm = self._continuous(arm, "arm")
        prune_before = timestamp_ns - self.flex_buffer.duration_ns
        self.flex_buffer.prune(prune_before)
        self.torso_buffer.prune(prune_before)
        self.arm_buffer.prune(prune_before)
        return AlignedInputs(timestamp_ns, flex, torso, arm)

    def _continuous(self, value: AlignedValue, source: str) -> AlignedValue:
        if value.value is None:
            return value
        previous = self._previous_torso if source == "torso" else self._previous_arm
        quaternion = normalize_quaternion(value.value)
        if previous is not None:
            quaternion = align_quaternion_sign(quaternion, previous)
        if source == "torso":
            self._previous_torso = quaternion
        else:
            self._previous_arm = quaternion
        return AlignedValue(tuple(float(item) for item in quaternion), value.skew_ms, value.valid)


class UniformTimeline:
    def __init__(self, output_hz: float, start_timestamp_ns: int) -> None:
        if output_hz <= 0.0:
            raise ValueError("output_hz must be positive")
        self.period_ns = round(1_000_000_000 / float(output_hz))
        self.next_timestamp_ns = int(start_timestamp_ns)
        self.frame_index = 0

    def due(self, available_through_ns: int) -> list[tuple[int, int]]:
        frames: list[tuple[int, int]] = []
        while self.next_timestamp_ns <= available_through_ns:
            frames.append((self.frame_index, self.next_timestamp_ns))
            self.frame_index += 1
            self.next_timestamp_ns += self.period_ns
        return frames
