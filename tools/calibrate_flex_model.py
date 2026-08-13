#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_PHASE4_CONFIG_PATH, DEFAULT_SENSOR_CONFIG_PATH, PROJECT_ROOT, load_sensor_config
from sleeve_arm.sources import create_sleeve_source


class LatestFlexReader:
    """Adapt non-blocking SleeveSource.latest() to fresh [CH2, CH3, CH4] reads."""

    def __init__(self, source, channels: tuple[int, int, int], timeout_s: float = 2.0) -> None:
        self.source = source
        self.indexes = tuple(channel - 1 for channel in channels)
        self.timeout_s = timeout_s
        self.last_timestamp: float | None = None

    def __call__(self) -> list[float]:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            frame = self.source.latest()
            if frame is not None and frame.timestamp != self.last_timestamp:
                if max(self.indexes) >= len(frame.channels):
                    raise ValueError(f"CH2/CH3/CH4 require at least 4 fields; got {len(frame.channels)}")
                self.last_timestamp = frame.timestamp
                return [frame.channels[index] for index in self.indexes]
            time.sleep(0.001)
        raise TimeoutError("no fresh Sleeve frame received during Flex calibration")


def calibration_settings(path: Path) -> tuple[str, tuple[int, int, int], Path]:
    with path.expanduser().resolve().open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    predictor = raw.get("predictor") if isinstance(raw, dict) else None
    if not isinstance(predictor, dict):
        raise ValueError("phase4 config requires predictor mapping")
    module = str(predictor.get("model_module", ""))
    channels_raw = predictor.get("sleeve_channels")
    if not module:
        raise ValueError("predictor.model_module is required")
    if not isinstance(channels_raw, list) or tuple(channels_raw) != (2, 3, 4):
        raise ValueError("predictor.sleeve_channels must be exactly [2, 3, 4]")
    output_value = predictor.get("calibration_file")
    if not output_value:
        raise ValueError("predictor.calibration_file is required")
    output = Path(str(output_value)).expanduser()
    if not output.is_absolute():
        output = PROJECT_ROOT / output
    return module, (2, 3, 4), output.resolve()


def main() -> int:
    parser = argparse.ArgumentParser(description="Quick-calibrate Flex CH2/CH3/CH4 without connecting any robot.")
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase4-config", type=Path, default=DEFAULT_PHASE4_CONFIG_PATH)
    parser.add_argument("--output", type=Path, help="override predictor.calibration_file")
    args = parser.parse_args()

    source = None
    try:
        module_name, channels, configured_output = calibration_settings(args.phase4_config)
        output = configured_output if args.output is None else args.output.expanduser().resolve()
        try:
            module = importlib.import_module(module_name)
            collect_flex_samples = getattr(module, "collect_flex_samples")
            quick_calibrate_flex = getattr(module, "quick_calibrate_flex")
        except (ImportError, ModuleNotFoundError, AttributeError) as exc:
            raise RuntimeError(
                f"Flex calibration API could not be loaded from module: {module_name}"
            ) from exc

        source = create_sleeve_source(load_sensor_config(args.sensor_config))
        source.start()
        read_flex = LatestFlexReader(source, channels)

        input("自然下垂并保持静止，按回车开始采集3秒基线：")
        rest_samples = collect_flex_samples(read_flex, 3.0)
        input("准备连续完成向前、向侧、向后三个完整动作，按回车开始采集12秒：")
        motion_samples = collect_flex_samples(read_flex, 12.0)
        input("再次自然下垂并保持静止，按回车开始采集2秒Trial基线：")
        trial_rest_samples = collect_flex_samples(read_flex, 2.0)

        calibration = quick_calibrate_flex(
            rest_samples=rest_samples,
            motion_samples=motion_samples,
            trial_rest_samples=trial_rest_samples,
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        calibration.save_json(str(output))
        print(f"calibration file = {output}")
        print("calibration_baseline =", list(calibration.calibration_baseline))
        print("calibration_scale =", list(calibration.calibration_scale))
        print("trial_rest =", list(calibration.trial_rest))
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\nCalibration cancelled.")
        return 130
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
