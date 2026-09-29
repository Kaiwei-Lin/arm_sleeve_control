"""Reuse Bend5 acquisition/calibration; map named fingers to Aurora radians."""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from sleeve_arm.config import GloveConfig


# electronic_skin_project_v9_11/docs/reference/aurora_reference_demo.py.
# Aurora's order differs from the direct FDH6 Ethernet SDK mapper.
HAND_JOINTS = ("thumb_bend", "index_flexion", "middle_flexion",
               "ring_flexion", "pinky_flexion", "thumb_swing")
HAND_FINGERS = ("thumb", "index", "middle", "ring", "pinky", "thumb")
HAND_LIMITS = ((0.12, 1.28), (0.17, 1.78), (0.17, 1.78),
               (0.17, 1.78), (0.17, 1.78), (0.0, 1.68))


@dataclass(frozen=True)
class GloveReading:
    timestamp: float  # monotonic packet receipt, never polling time
    raw: tuple[float, ...]
    curls: dict[str, float]
    status: str


class GloveRuntime:
    def __init__(self, config: GloveConfig, *, mode="real", clock=None):
        if mode not in ("real", "fake"):
            raise ValueError("glove mode must be real or fake")
        if mode == "real" and not config.port:
            raise ValueError("set sensors.glove.port before using --glove real")
        for pose in (config.open_pose_rad, config.closed_pose_rad):
            if len(pose) != 6 or any(not math.isfinite(v) or not low <= v <= high
                                     for v, (low, high) in zip(pose, HAND_LIMITS)):
                raise ValueError("glove open/closed poses exceed Aurora hand limits")
        self.config, self.mode = config, mode
        self.clock = clock or time
        self.source = None
        self.reading = None

    def start(self):
        # The dependency is optional: arm-only runs do not import electronic_skin.
        root = self.config.project_path.resolve()
        if not (root / "electronic_skin/sources/gloves/bend5.py").is_file():
            raise ValueError(f"Bend5 implementation not found at {root}; set sensors.glove.project_path")
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        import electronic_skin
        if root not in Path(electronic_skin.__file__).resolve().parents:
            raise ValueError("another electronic_skin package is already imported; check glove.project_path")
        import numpy as np
        from electronic_skin.sources.gloves.bend5 import Bend5SerialHandInterface

        runtime = self

        class TimedBend5(Bend5SerialHandInterface):
            def _store_packet(self, packet):
                received_at = runtime.clock.monotonic()
                if not np.isfinite(packet).all():
                    raise ValueError("Bend5 packet contains non-finite values")
                super()._store_packet(packet)
                frame = self.read_frame()
                # Publish one complete reading atomically after calibration.
                runtime.reading = GloveReading(
                    received_at, tuple(float(v) for v in packet),
                    {name: value.value for name, value in frame.upper_limb.hand.joints.items()},
                    frame.meta["status"],
                )

        options = dict(self.config.source_options)
        calibration = self.config.calibration_path
        if self.mode == "fake":
            calibration = None
            options.update(zero_on_start=False, filter_alpha=1.0, adaptive_baseline=False,
                           per_finger_max_delta=1.0, deadzone_value=0.0)
        elif calibration is not None and not calibration.is_file():
            raise ValueError(f"glove calibration file not found: {calibration}")
        self.source = TimedBend5(
            port=self.config.port or "FAKE", baudrate=self.config.baudrate,
            timeout=self.config.timeout_s,
            calibration_path=None if calibration is None else str(calibration), **options,
        )
        if calibration is not None and not self.source.calibration_loaded:
            raise ValueError(f"invalid glove calibration file: {calibration}")
        self.started_at = self.clock.monotonic()
        self.reading = None
        if self.mode == "real":
            self.source.open()
            self.source.start()

    def latest(self):
        if self.source is None:
            return None
        if self.mode == "fake":
            import numpy as np
            amount = (1.0 - math.cos(self.clock.monotonic() - self.started_at)) / 2.0
            self.source._store_packet(np.asarray([amount] * 5 + [0.0] * 6, dtype=np.float32))
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
