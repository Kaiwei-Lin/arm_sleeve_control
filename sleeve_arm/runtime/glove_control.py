"""Own local glove acquisition/calibration and map named inputs to Aurora radians."""
from __future__ import annotations

import math
import time
from dataclasses import replace

from sleeve_arm.config import GloveConfig, BEND6_DEFAULT_MAPPING, validate_glove_config, validate_bend6_mapping
from sleeve_arm.sources.bend6 import BEND6_CHANNEL_NAMES, Bend6GloveSource, Bend6Reading
from sleeve_arm.sources.glove import Bend5GloveSource


# Semantic pose order and per-joint bounds, NOT the SDK vector order.
# AuroraRobotArm uses the selected profile's indices for commands and feedback.
HAND_JOINTS = ("thumb_bend", "index_flexion", "middle_flexion",
               "ring_flexion", "pinky_flexion", "thumb_swing")
HAND_FINGERS = ("thumb", "index", "middle", "ring", "pinky", "thumb")
HAND_LIMITS = ((0.12, 1.28), (0.17, 1.78), (0.17, 1.78),
               (0.17, 1.78), (0.17, 1.78), (0.0, 1.68))


class GloveRuntime:
    def __init__(self, config: GloveConfig, *, mode="real", clock=None, serial_factory=None):
        validate_glove_config(config)
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
        self.serial_factory = serial_factory
        self.source = None
        # Preflight before RobotRuntime.connect(); constructors never open serial.
        self._prepared_source = self._make_source()

    def _make_source(self):
        options = dict(self.config.source_options)
        calibration = self.config.calibration_path
        source_type = Bend5GloveSource
        if self.config.backend == "bend6":
            source_type = Bend6GloveSource
            if self.mode == "fake":
                calibration = None
                # Explicit simulation only: this identity calibration is never
                # written to disk or used to interpret a real serial stream.
                options.update(synthetic=True, calibration={
                    "mode": "normalized", "channels": {
                        name: {"raw_min": 0., "raw_max": 1., "invert": False}
                        for name in BEND6_CHANNEL_NAMES
                    },
                })
        elif self.mode == "fake":
            calibration = None
            options.update(zero_on_start=False, filter_alpha=1.0, adaptive_baseline=False,
                           per_finger_max_delta=1.0, deadzone_value=0.0)
        elif calibration is not None and not calibration.is_file():
            raise ValueError(f"glove calibration file not found: {calibration}")
        source = source_type(
            port=self.config.port or "FAKE", baudrate=self.config.baudrate,
            timeout=self.config.timeout_s,
            calibration_path=calibration, clock=self.clock, serial_factory=self.serial_factory, **options,
        )
        if self.config.backend == "bend5" and calibration is not None and not source.calibration_loaded:
            raise ValueError(f"invalid glove calibration file: {calibration}")
        return source

    def validate_for_control(self):
        if self.config.backend == "bend6":
            validate_bend6_mapping(self.config.bend6_mapping)
            source = self.source or self._prepared_source
            if source.calibration.mode != "normalized":
                raise ValueError("Bend6 raw data is read-only; control requires complete normalized calibration")
            uncalibrated = set(self.config.bend6_mapping.values()) & set(source.calibration.raw_only_channels)
            if uncalibrated:
                raise ValueError(f"Bend6 mapping uses uncalibrated raw-only channels: {sorted(uncalibrated)}")

    def start(self):
        if self.source is not None:
            raise RuntimeError("glove runtime is already started")
        self.source = self._prepared_source or self._make_source()
        self._prepared_source = None
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
            amount = (1.0 - math.cos(self.clock.monotonic() - self.started_at)) / 2.0
            if self.config.backend == "bend6":
                self.source._store_packet([amount] * 7 + [1. - amount, amount, 0., 0.])
            else:
                import numpy as np
                self.source._store_packet(np.asarray(
                    [amount] * 5 + [0.0] * (self.source.expected_fields - 5), dtype=np.float32))
        return self.reading

    def wait_ready(self, *, require_control=True):
        if require_control:
            self.validate_for_control()
        deadline = self.clock.monotonic() + self.config.startup_timeout_s
        while self.clock.monotonic() < deadline:
            reading = self.latest()
            if reading is not None and reading.status == "ok":
                # Holding is a running-state policy. Startup still needs a fresh
                # frame, including after an earlier sensor/model setup delay.
                if self._check_age(reading) <= self.config.stale_timeout_s:
                    self.targets() if require_control else self.check_readable()
                    return
            self.clock.sleep(0.01)
        raise RuntimeError(f"glove readiness timeout: no ready {self.config.backend} packet{self._serial_diagnostics()}")

    def _serial_diagnostics(self):
        if self.config.backend == "bend6" and self.source is not None:
            return f"; serial={self.source.diagnostics()}"
        return ""

    @property
    def hold_last_on_stale(self):
        return self.config.backend == "bend6" and self.config.stale_policy == "hold_last"

    def _check_age(self, reading):
        age = self.clock.monotonic() - reading.timestamp
        if (not math.isfinite(age) or age < 0
                or (age > self.config.stale_timeout_s and not self.hold_last_on_stale)):
            raise RuntimeError(f"glove watchdog: stale/invalid {self.config.backend} packet ({age:.3f}s)"
                               f"{self._serial_diagnostics()}")
        return age

    def check_readable(self):
        # Source retains the last complete frame on empty reads. Always poll it
        # so reader failures propagate, even while retaining an old target.
        reading = self.latest()
        if reading is None or reading.status != "ok":
            raise RuntimeError("glove is not ready/calibrated")
        self._check_age(reading)
        return reading

    def _bend6_values(self, reading):
        # Future posture/crosstalk compensation belongs at this named mapping
        # boundary; it must not discard or rewrite the source's nine channels.
        self.validate_for_control()
        if not isinstance(reading, Bend6Reading) or reading.calibration_mode not in ("normalized", "partial"):
            raise RuntimeError("Bend6 raw data is read-only; targets require normalized calibration")
        if set(reading.channels) != set(BEND6_CHANNEL_NAMES):
            raise ValueError("Bend6 reading requires all nine channels")
        if set(self.config.bend6_mapping.values()) & set(reading.raw_only_channels):
            raise ValueError("Bend6 mapping uses uncalibrated raw-only channels")
        if any(not math.isfinite(v) or (name not in reading.raw_only_channels and not 0 <= v <= 1)
               for name, v in reading.channels.items()):
            raise ValueError("Bend6 channels must be finite normalized values in [0, 1]")
        return [reading.channels[self.config.bend6_mapping[name]] for name in HAND_JOINTS]

    def preview_targets(self):
        reading = self.check_readable()
        if isinstance(reading, Bend6Reading) and reading.calibration_mode == "raw":
            return None
        return self._targets_from_reading(reading)

    def targets(self):
        return self._targets_from_reading(self.check_readable())

    def _targets_from_reading(self, reading):
        self._check_age(reading)
        if reading.status != "ok":
            raise RuntimeError("glove is not ready/calibrated")
        values = (self._bend6_values(reading) if self.config.backend == "bend6"
                  else [reading.curls[name] for name in HAND_FINGERS])
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in values):
            raise ValueError("glove inputs must be finite values in [0, 1]")
        targets = dict(zip(HAND_JOINTS, (
            opened + curl * (closed - opened)
            for curl, opened, closed in zip(values, self.config.open_pose_rad, self.config.closed_pose_rad)
        )))
        if any(not math.isfinite(v) for v in targets.values()):
            raise ValueError("glove targets must be finite")
        return targets

    def describe_startup(self, print_fn=print):
        if self.config.backend == "bend5":
            print_fn("Bend5: keep fingers straight during startup calibration.")
        else:
            print_fn("Bend6: discard the first terminated record for synchronization; "
                     "raw data is read-only, control requires normalized calibration for every mapped channel.")
            if self.hold_last_on_stale:
                print_fn("Bend6 stale_policy=hold_last: after startup, retain the last valid frame "
                         "until new data arrives; timestamps remain unchanged.")

    def report(self, print_fn=print):
        reading = self.reading
        if isinstance(reading, Bend6Reading):
            age = self.clock.monotonic() - reading.timestamp
            input_state = ("holding_last" if self.hold_last_on_stale else "stale") if (
                age > self.config.stale_timeout_s) else "fresh"
            print_fn(f"[GLOVE {self.mode} Bend6] raw={reading.raw} raw_channels={reading.raw_channels} "
                     f"channels={dict(reading.channels)} calibration={reading.calibration_mode} "
                     f"raw_only_channels={reading.raw_only_channels} "
                     f"filter={'ema' if reading.filter_enabled else 'off'} synthetic={reading.synthetic} "
                     f"side={reading.side} sequence={reading.sequence} "
                     f"age={age:.3f}s status={reading.status} input_state={input_state} "
                     f"stale_policy={self.config.stale_policy}")
            if reading.calibration_mode == "raw":
                print_fn("target_rad=unavailable (Bend6 raw data is read-only; normalized calibration required)")
            return
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
    glove = GloveRuntime(configs.sensors.glove, mode=configs.glove_mode)
    if not args.print_only:
        glove.validate_for_control()
    return glove
