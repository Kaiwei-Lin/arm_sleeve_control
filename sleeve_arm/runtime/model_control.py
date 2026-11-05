"""Application lifecycle and watchdog policy, independent of sensor/robot hardware."""
from __future__ import annotations

import sys
import time
from enum import Enum, auto

from sleeve_arm.control import SensorWatchdog
from sleeve_arm.runtime.intent_pipeline import RotationPredictionError
from sleeve_arm.runtime.telemetry import Telemetry


class RuntimeState(Enum):
    INIT = auto()
    ROBOT_READY = auto()
    SENSOR_READY = auto()
    ARMED = auto()
    RUNNING = auto()
    STALE = auto()
    FAULT = auto()
    STOPPING = auto()


class ModelControlApp:
    def __init__(self, *, sensors, intents, robot, phase3, phase4, duration=None,
                 telemetry=None, clock=None, print_fn=print, glove=None):
        self.sensors = sensors
        self.intents = intents
        self.robot = robot
        self.glove = glove
        self.glove_control_required = not getattr(robot, "read_only", False)
        self.phase3 = phase3
        self.phase4 = phase4
        self.duration = duration
        self.telemetry = telemetry or Telemetry()
        self.clock = clock or time
        self.print = print_fn
        self.sample_label = getattr(sensors, "sample_label", "Sleeve")
        self.state = RuntimeState.INIT
        self.fault = None
        self.invalid = self.consecutive_errors = self.stale = 0
        self.rotation_invalid = self.rotation_consecutive_errors = 0
        self.cycles = self.predictions = 0

    def run(self):
        try:
            if self.robot.offline_preview:
                self.robot.describe_preview()
                return 0
            self.watchdog = SensorWatchdog(self.phase3.sensor_timeout_ms, self.phase3.hard_timeout_ms)
            if self.glove is not None and self.glove_control_required:
                self.glove.validate_for_control()
            # Proven DyMotor ordering: its connect performs Servo On, before
            # opening serial sources or importing/initializing external models.
            self.robot.connect()
            self.state = RuntimeState.ROBOT_READY
            self.sensors.start()
            if self.glove is not None:
                self.glove.describe_startup(self.print)
                self.glove.start()
            self.intents.prepare(self.sensors)
            sample, intent = self._wait_ready()
            if self.glove is not None:
                self.glove.wait_ready(require_control=self.glove_control_required)
            self.state = RuntimeState.SENSOR_READY
            if not self.robot.prepare():
                return 0
            self.state = RuntimeState.ARMED
            self._control_loop(sample, intent)
            return 0
        except KeyboardInterrupt:
            self.print("\nStopping on Ctrl+C...")
            return 0
        except Exception as exc:
            self.state = RuntimeState.FAULT
            self.fault = str(exc)
            self.print(f"ERROR [FAULT] {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
        finally:
            self.state = RuntimeState.STOPPING
            for label, close in (("shutdown", self.robot.shutdown),
                                 ("glove close", None if self.glove is None else self.glove.close),
                                 ("source close", None if self.sensors is None else self.sensors.close)):
                if close is not None:
                    try:
                        close()
                    except BaseException as exc:
                        self.print(f"ERROR [STOPPING] {label}: {exc}", file=sys.stderr)

    def _wait_ready(self):
        deadline = self.clock.monotonic() + self.phase3.hard_timeout_ms / 1000.0
        last_error = None
        while self.clock.monotonic() < deadline:
            sample = self.sensors.latest()
            if sample is not None:
                try:
                    return sample, self.intents.predict(sample)
                except Exception as exc:
                    self.invalid += 1
                    last_error = exc
            self.clock.sleep(0.001)
        reason = (f"no {self.sample_label} sample received; prediction was not attempted"
                  if last_error is None else str(last_error))
        describe = getattr(self.sensors, "readiness_diagnostics", None)
        details = "" if describe is None else f"; {describe()}"
        raise RuntimeError(f"no valid model prediction before readiness timeout: {reason}{details}")

    def _predict(self, sample):
        try:
            intent = self.intents.predict(sample)
        except RotationPredictionError as exc:
            self.rotation_invalid += 1
            self.rotation_consecutive_errors += 1
            if self.robot.fail_fast or self.rotation_consecutive_errors >= self.phase4.max_consecutive_prediction_errors:
                raise RuntimeError(f"too many consecutive upper-arm rotation errors: {exc}") from exc
            self.print(f"WARNING: upper-arm rotation rejected; holding last safe target: {exc}", file=sys.stderr)
            intent = exc.intent
        except Exception as exc:
            self.invalid += 1
            self.consecutive_errors += 1
            if self.robot.fail_fast or self.consecutive_errors >= self.phase4.max_consecutive_prediction_errors:
                raise RuntimeError(f"too many consecutive prediction errors: {exc}") from exc
            self.print(f"WARNING: prediction rejected; holding last safe target: {exc}", file=sys.stderr)
            return None
        else:
            self.rotation_consecutive_errors = 0
        # No completed inference may submit a sample that crossed hard timeout.
        now = self.clock.monotonic()
        if self.watchdog.is_hard_timeout(sample.timestamp, now):
            raise RuntimeError(f"prediction completed after {self.sample_label} hard timeout")
        self.consecutive_errors = 0
        if (self.sample_label == "Sleeve" and self.phase3.sleeve_stale_policy == "pause"
                and self.watchdog.is_stale(sample.timestamp, now)):
            self._pause_sleeve(self.watchdog.age(sample.timestamp, now))
            return None
        return intent

    def _control_loop(self, sample, intent):
        period = 1.0 / self.phase3.control_hz
        started = next_tick = last_print = self.clock.monotonic()
        deadline = None if self.duration is None else started + self.duration
        last_timestamp = sample.timestamp
        targets = self.robot.startup
        pause_sleeve = self.sample_label == "Sleeve" and self.phase3.sleeve_stale_policy == "pause"
        self.state = RuntimeState.RUNNING
        while deadline is None or self.clock.monotonic() < deadline:
            incoming = self.sensors.latest()
            self.cycles += 1
            self.robot.check_health()
            if self.glove is not None:
                self._glove_targets()  # Apply the glove's stale policy; other input faults still propagate.
            now = self.clock.monotonic()
            if incoming is not None:
                sample = incoming
            # Check the last known timestamp even when latest() returns None.
            age = self.watchdog.age(sample.timestamp, now)
            if self.watchdog.is_hard_timeout(sample.timestamp, now):
                raise RuntimeError(f"{self.sample_label} hard timeout: {age * 1000:.1f} ms "
                                   f"(limit={self.phase3.hard_timeout_ms:g} ms){self._sensor_diagnostics()}")
            if self.watchdog.is_stale(sample.timestamp, now):
                if self.robot.fail_fast and not pause_sleeve:
                    raise RuntimeError(
                        f"sensor watchdog: stale/missing {self.sample_label.lower()} feedback; "
                        f"age={age * 1000:.1f} ms limit={self.phase3.sensor_timeout_ms:g} ms; "
                        f"restart requires confirmation{self._sensor_diagnostics()}"
                    )
                if pause_sleeve:
                    self._pause_sleeve(age)
                else:
                    if self.state is not RuntimeState.STALE:
                        self.print(f"WARNING: {self.sample_label} stale ({age * 1000:.1f} ms); holding last safe target")
                    self.state = RuntimeState.STALE
                    self.stale += 1
            elif incoming is not None and sample.timestamp != last_timestamp:
                predicted = self._predict(sample)
                last_timestamp = sample.timestamp
                if predicted is not None:
                    # Sensor-driven streaming: apply a limited step, never a
                    # blocking MoveCommand trajectory in this per-sample loop.
                    targets = self._apply(predicted, period)
                    intent = predicted
                    self.predictions += 1
                    if self.state is RuntimeState.STALE and pause_sleeve:
                        self.print(f"Sleeve feedback recovered ({age * 1000:.1f} ms); resuming new targets")
                    self.state = RuntimeState.RUNNING
            elif self.glove is not None or (getattr(self.robot, "stream_between_samples", False) and self.predictions):
                # Keep advancing toward the latest fresh intent at the control
                # rate, even if sensor samples arrive more slowly. The watchdog
                # above stops reuse as soon as the sample becomes stale.
                targets = self._apply(intent, period)
            if now - last_print >= 1.0:
                self.telemetry.report(
                    sample=sample, intent=intent, targets=targets, feedback=self.robot.feedback(),
                    sensor_stats=self.sensors.stats, diagnostics=self.intents.diagnostics(),
                    stats=dict(state=self.state.name, elapsed=now-started, age=age,
                               cycles=self.cycles, predictions=self.predictions, invalid=self.invalid,
                               stale=self.stale, rotation_invalid=self.rotation_invalid),
                )
                if self.glove is not None:
                    self.glove.report(self.print)
                last_print = now
            next_tick += period
            now = self.clock.monotonic()
            if next_tick < now:
                # Skip missed ticks after slow work; never burst old steps to catch up.
                next_tick = now + period
            self.clock.sleep(max(0.0, next_tick - now))

    def _pause_sleeve(self, age):
        if getattr(self.sensors, "imu_error", None) is not None:
            raise RuntimeError(f"IMU acquisition failed during Sleeve pause: {self.sensors.imu_error}") from self.sensors.imu_error
        if self.state is not RuntimeState.STALE:
            self.print(f"WARNING: Sleeve stale ({age * 1000:.1f} ms); pausing new targets; "
                       f"hard timeout={self.phase3.hard_timeout_ms:g} ms")
        self.state = RuntimeState.STALE
        self.stale += 1

    def _sensor_diagnostics(self):
        describe = getattr(self.sensors, "readiness_diagnostics", None)
        return "" if describe is None else f"; {describe()}"

    def _apply(self, intent, period):
        # Recheck source health and the selected stale policy after inference.
        kwargs = {} if self.glove is None else {"hand_targets": self._glove_targets()}
        return self.robot.apply(intent, period, **kwargs)

    def _glove_targets(self):
        return self.glove.targets() if self.glove_control_required else self.glove.preview_targets()
