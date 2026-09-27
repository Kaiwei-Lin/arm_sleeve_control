from __future__ import annotations

import math

from .types import VALID_ACTIONS


class ActionDebouncer:
    def __init__(
        self,
        *,
        required_frames: int = 3,
        confidence_threshold: float = 0.55,
        low_confidence_grace_frames: int = 2,
    ) -> None:
        if required_frames < 1:
            raise ValueError("required_frames must be at least 1")
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if low_confidence_grace_frames < 0:
            raise ValueError("low_confidence_grace_frames must be non-negative")
        self.required_frames = int(required_frames)
        self.confidence_threshold = float(confidence_threshold)
        self.low_confidence_grace_frames = int(low_confidence_grace_frames)
        self.reset()

    @property
    def stable_action(self) -> str:
        return self._stable

    def update(self, candidate: str, confidence: float) -> str:
        if candidate not in VALID_ACTIONS:
            raise ValueError(f"unsupported action: {candidate}")
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be finite and in [0, 1]")
        if candidate == "Rest":
            self._stable = "Rest"
            self._candidate = None
            self._candidate_count = 0
            self._low_confidence_count = 0
            return self._stable
        if confidence < self.confidence_threshold or candidate == "Unknown":
            self._candidate = None
            self._candidate_count = 0
            self._low_confidence_count += 1
            if self._low_confidence_count > self.low_confidence_grace_frames:
                self._stable = "Unknown"
            return self._stable
        self._low_confidence_count = 0
        if candidate == self._stable:
            self._candidate = None
            self._candidate_count = 0
            return self._stable
        if candidate == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = candidate
            self._candidate_count = 1
        if self._candidate_count >= self.required_frames:
            self._stable = candidate
            self._candidate = None
            self._candidate_count = 0
        return self._stable

    def reset(self) -> None:
        self._stable = "Unknown"
        self._candidate: str | None = None
        self._candidate_count = 0
        self._low_confidence_count = 0


class AngleFilter:
    def __init__(
        self,
        *,
        alpha: float = 0.25,
        max_velocity_deg_s: float = 180.0,
        max_gap_s: float = 1.0,
    ) -> None:
        if not math.isfinite(alpha) or not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be finite and in (0, 1]")
        if not math.isfinite(max_velocity_deg_s) or max_velocity_deg_s <= 0.0:
            raise ValueError("max_velocity_deg_s must be finite and positive")
        if not math.isfinite(max_gap_s) or max_gap_s <= 0.0:
            raise ValueError("max_gap_s must be finite and positive")
        self.alpha = float(alpha)
        self.max_velocity_deg_s = float(max_velocity_deg_s)
        self.max_gap_s = float(max_gap_s)
        self.reset()

    @property
    def value(self) -> float | None:
        return self._value

    def update(self, angle_deg: float, timestamp_ns: int) -> float:
        if not math.isfinite(angle_deg):
            raise ValueError("angle_deg must be finite")
        if not isinstance(timestamp_ns, int) or timestamp_ns <= 0:
            raise ValueError("timestamp_ns must be a positive integer")
        if self._timestamp_ns is None or self._value is None:
            self._value = float(angle_deg)
            self._timestamp_ns = timestamp_ns
            return self._value
        if timestamp_ns <= self._timestamp_ns:
            raise ValueError("timestamps must be strictly increasing")
        dt = (timestamp_ns - self._timestamp_ns) / 1e9
        if dt > self.max_gap_s:
            self._value = float(angle_deg)
            self._timestamp_ns = timestamp_ns
            return self._value
        max_delta = self.max_velocity_deg_s * dt
        limited = min(max(float(angle_deg), self._value - max_delta), self._value + max_delta)
        self._value = self.alpha * limited + (1.0 - self.alpha) * self._value
        self._timestamp_ns = timestamp_ns
        return self._value

    def rest(self) -> float:
        self._value = 0.0
        self._timestamp_ns = None
        return 0.0

    def reset(self) -> None:
        self._value: float | None = None
        self._timestamp_ns: int | None = None
