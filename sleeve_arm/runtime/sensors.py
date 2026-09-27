"""Sensor ownership, synchronized samples and freshness; no robot dependencies."""
from __future__ import annotations

import time
import math
from dataclasses import replace

from sleeve_arm.config import SensorConfig
from sleeve_arm.domain import ImuFrame, SensorSample
from sleeve_arm.sources import (
    FakeImuSource, FakeSleeveSource, ImuSource, create_imu_source, create_sleeve_source,
)
from sleeve_arm.sync import SensorSynchronizer


def _add_latest_pair(
    sync: SensorSynchronizer,
    sources: dict[str, ImuSource],
    names: tuple[str, str],
) -> None:
    for name, add in zip(names, (sync.add_imu1, sync.add_imu2)):
        source = sources[name]
        frame = source.latest()
        if frame is None:
            continue
        add(frame)


def _attach_latest_pair(
    sample: SensorSample,
    sync: SensorSynchronizer,
    max_sync_ms: float,
) -> SensorSample:
    pair = sync.latest_imu_pair(max_sync_ms)
    if pair is None:
        return replace(sample, imu1=None, imu2=None)
    return replace(sample, imu1=pair[0], imu2=pair[1])


def _latest_imu_pair(
    imu_sources: dict[str, ImuSource],
    sync: SensorSynchronizer,
    names: tuple[str, str],
    max_sync_ms: float,
) -> tuple[ImuFrame, ImuFrame] | None:
    _add_latest_pair(sync, imu_sources, names)
    return sync.latest_imu_pair(max_sync_ms)


def _require_fresh_pair(
    pair: tuple[ImuFrame, ImuFrame] | None,
    max_age_s: float,
    label: str,
) -> tuple[ImuFrame, ImuFrame]:
    if pair is None:
        raise ValueError(f"{label} requires a synchronized IMU pair")
    if any(abs(time.monotonic() - frame.timestamp) > max_age_s for frame in pair):
        raise ValueError(f"{label} IMU pair is stale")
    return pair


def _pair_synchronizer(sensor_config: SensorConfig) -> SensorSynchronizer:
    return SensorSynchronizer(
        sensor_config.synchronization.max_time_delta_ms,
        sensor_config.synchronization.buffer_duration_ms,
        imu1_enabled=True,
        imu2_enabled=True,
    )


class SensorRuntime:
    """Own sources lazily so construction cannot open a serial port.

    IMU read failures accompany the next sample via ``imu_error``. The intent
    pipeline rejects that prediction, allowing the app's existing error policy
    to decide whether to hold or fault.
    """

    def __init__(self, config, *, sleeve="real", imus="real", shoulder_backend="dual_imu"):
        self.config = config
        self.sleeve_mode = sleeve
        self.imu_mode = imus
        self.shoulder_backend = shoulder_backend
        self.source = None
        self.imu_sources = {}
        self.imu_error = None
        self.rotation_pair = None
        self.shoulder_names = (config.shoulder_imu.arm_imu, config.shoulder_imu.chest_imu)
        self.rotation_names = (config.upper_arm_rotation.upper_imu, config.upper_arm_rotation.reference_imu)
        self.required_imus = tuple(dict.fromkeys(
            (self.shoulder_names if shoulder_backend == "dual_imu" else ())
            + (self.rotation_names if config.upper_arm_rotation.enabled else ())
        ))
        for name in self.required_imus:
            if not getattr(config, name).enabled:
                raise ValueError(f"model control requires sensors.{name}.enabled=true")
        self.reset_synchronization()

    def max_sync_ms(self, pair_config):
        return (self.config.synchronization.max_time_delta_ms
                if pair_config.max_sync_ms is None else pair_config.max_sync_ms)

    def reset_synchronization(self):
        config = self.config
        self.shoulder_sync = (
            _pair_synchronizer(config) if self.shoulder_backend == "dual_imu"
            else SensorSynchronizer(config.synchronization.max_time_delta_ms,
                                    config.synchronization.buffer_duration_ms)
        )
        self.rotation_sync = _pair_synchronizer(config) if config.upper_arm_rotation.enabled else None
        self.rotation_pair = None
        self.imu_error = None

    def start(self):
        self.source = FakeSleeveSource() if self.sleeve_mode == "fake" else create_sleeve_source(self.config)
        delayed = set()
        if self.shoulder_backend == "dual_imu":
            delayed.add(self.shoulder_names[1])
        if self.rotation_sync is not None:
            delayed.add(self.rotation_names[1])
        for name in self.required_imus:
            source = (FakeImuSource(timestamp_offset_s=0.005 if name in delayed else 0.0)
                      if self.imu_mode == "fake" else create_imu_source(name, getattr(self.config, name)))
            if source is None:
                raise RuntimeError(f"model control could not create configured {name} source")
            self.imu_sources[name] = source
        self.source.start()
        for source in self.imu_sources.values():
            source.start()

    def latest(self) -> SensorSample | None:
        self.imu_error = None
        try:
            if self.shoulder_backend == "dual_imu":
                _add_latest_pair(self.shoulder_sync, self.imu_sources, self.shoulder_names)
            if self.rotation_sync is not None:
                _add_latest_pair(self.rotation_sync, self.imu_sources, self.rotation_names)
        except Exception as exc:
            self.imu_error = exc
        if self.rotation_sync is not None:
            self.rotation_pair = self.rotation_sync.latest_imu_pair(self.max_sync_ms(self.config.upper_arm_rotation))
        frame = self.source.latest()
        if frame is None:
            return None
        sample = self.shoulder_sync.synchronize(frame)
        if self.shoulder_backend == "dual_imu":
            sample = _attach_latest_pair(sample, self.shoulder_sync, self.max_sync_ms(self.config.shoulder_imu))
        return sample

    def set_fake_calibration_pose(self, stage):
        """Supply a real 45° forward fixture to the unchanged dual-IMU algorithm.

        Identity-only fake data cannot calibrate a forward direction. This hook
        only changes synthetic frames; real sources and estimators are untouched.
        """
        if self.imu_mode != "fake":
            return
        half = math.radians(45 if stage == "forward" else 0) / 2
        quaternion = (math.cos(half), 0., math.sin(half), 0.)
        self.imu_sources[self.shoulder_names[0]].quaternion_fn = lambda _: quaternion

    @property
    def stats(self):
        return self.source.stats

    def close(self):
        errors = []
        sources = ([self.source] if self.source is not None else []) + list(reversed(tuple(self.imu_sources.values())))
        for source in sources:
            try:
                source.close()
            except BaseException as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError("; ".join(errors))


def create_sensor_runtime(args, configs):
    if configs.offline_preview:
        return None
    return SensorRuntime(configs.sensors, sleeve=args.sleeve, imus=args.imus,
                         shoulder_backend=configs.shoulder_backend)
