"""Own local Bend5 acquisition/calibration and map named fingers to Aurora radians."""
from __future__ import annotations

import math
import time
from dataclasses import replace

from sleeve_arm.config import GloveConfig
from sleeve_arm.sources.glove import Bend5GloveSource


# GR3 Aurora hand group order and SDK bounds.
# Aurora's order differs from the direct FDH6 Ethernet SDK mapper.
HAND_JOINTS = ("thumb_bend", "index_flexion", "middle_flexion",
               "ring_flexion", "pinky_flexion", "thumb_swing")
HAND_FINGERS = ("thumb", "index", "middle", "ring", "pinky", "thumb")
HAND_LIMITS = ((0.12, 1.28), (0.17, 1.78), (0.17, 1.78),
               (0.17, 1.78), (0.17, 1.78), (0.0, 1.68))


class GloveRuntime:
    def __init__(self, config: GloveConfig, *, mode="real", clock=None):
        if mode not in ("real", "fake"):
            raise ValueError("glove mode must be real or fake")
        if mode == "real" and not config.port:
            raise ValueError("set sensors.glove.port before using --glove real")
        for pose in (config.open_pose_rad, config.closed_pose_rad):
            if len(pose) != 6 or any(not math.isfinite(v) for v in pose):
                raise ValueError("glove open/closed poses require six finite values")
        self.config = replace(config, **{
            name: tuple(min(max(v, low), high) for v, (low, high) in zip(getattr(config, name), HAND_LIMITS))
            for name in ("open_pose_rad", "closed_pose_rad")
        })
        self.mode = mode
        self.clock = clock or time
        self.source = None

    def start(self):
        options = dict(self.config.source_options)
        calibration = self.config.calibration_path
        if self.mode == "fake":
            calibration = None
            options.update(zero_on_start=False, filter_alpha=1.0, adaptive_baseline=False,
                           per_finger_max_delta=1.0, deadzone_value=0.0)
        elif calibration is not None and not calibration.is_file():
            raise ValueError(f"glove calibration file not found: {calibration}")
        self.source = Bend5GloveSource(
            port=self.config.port or "FAKE", baudrate=self.config.baudrate,
            timeout=self.config.timeout_s,
            calibration_path=calibration, clock=self.clock, **options,
        )
        if calibration is not None and not self.source.calibration_loaded:
            raise ValueError(f"invalid glove calibration file: {calibration}")
        self.started_at = self.clock.monotonic()
        if self.mode == "real":
            self.source.start()

    @property
    def reading(self):
        return None if self.source is None else self.source.latest()

    def latest(self):
        if self.source is None:
            return None
        if self.mode == "fake":
            import numpy as np
            amount = (1.0 - math.cos(self.clock.monotonic() - self.started_at)) / 2.0
            self.source._store_packet(np.asarray(
                [amount] * 5 + [0.0] * (self.source.expected_fields - 5), dtype=np.float32))
        return self.reading

    def wait_ready(self):
        deadline = self.clock.monotonic() + self.config.startup_timeout_s
        while self.clock.monotonic() < deadline:
            reading = self.latest()
            if reading is not None and reading.status == "ok":
                self.targets()
                return
            self.clock.sleep(0.01)
        raise RuntimeError("glove readiness timeout: no calibrated Bend5 packet")

    def targets(self):
        reading = self.latest()
        if reading is None or reading.status != "ok":
            raise RuntimeError("glove is not ready/calibrated")
        age = self.clock.monotonic() - reading.timestamp
        if not math.isfinite(age) or age < 0 or age > self.config.stale_timeout_s:
            raise RuntimeError(f"glove watchdog: stale/invalid Bend5 packet ({age:.3f}s)")
        values = [reading.curls[name] for name in HAND_FINGERS]
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in values):
            raise ValueError("Bend5 curls must be finite values in [0, 1]")
        return dict(zip(HAND_JOINTS, (
            opened + curl * (closed - opened)
            for curl, opened, closed in zip(values, self.config.open_pose_rad, self.config.closed_pose_rad)
        )))

    def report(self, print_fn=print):
        reading = self.reading
        if reading is not None:
            print_fn(f"[GLOVE {self.mode}] raw={reading.raw[:5]} "
                     f"curls={ {k: round(v, 3) for k, v in reading.curls.items()} } "
                     f"status={reading.status}")

    def close(self):
        if self.source is not None:
            self.source.close()
            self.source = None


def create_glove_runtime(args, configs):
    if configs.offline_preview or configs.glove_mode == "none":
        return None
    return GloveRuntime(configs.sensors.glove, mode=configs.glove_mode)
