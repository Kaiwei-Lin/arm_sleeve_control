#!/usr/bin/env python3
"""Compare two Bend6 poses to locate channel routing errors; never connect a robot."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import PROJECT_ROOT, validate_bend6_mapping
from sleeve_arm.sources.bend6 import (
    BEND6_RAW_CHANNEL_NAMES, Bend6Calibration, Bend6GloveSource, load_bend6_calibration,
)
from tools.calibrate_bend6_glove import (
    capture_config, collect_pose, positive_float, settling_seconds, write_new,
)


ACTIONS = {
    "thumb_bend": "只弯曲拇指指尖，尽量保持拇指横摆和其余四指不动",
    "index_flexion": "只弯曲食指，尽量保持其余手指不动",
    "middle_flexion": "只弯曲中指，尽量保持其余手指不动",
    "ring_flexion": "只弯曲无名指，尽量保持其余手指不动",
    "pinky_flexion": "只弯曲小指，尽量保持其余手指不动",
    "thumb_swing": "向掌心横摆拇指，尽量保持拇指指尖和其余四指不动",
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-config", type=Path,
                        default=PROJECT_ROOT / "configs/sensors_bend6.local.yaml")
    parser.add_argument("--joint", choices=tuple(ACTIONS), required=True, help="本次人工检查的手指动作")
    parser.add_argument("--duration", type=positive_float, default=2., help="每姿态采集秒数，默认 2")
    parser.add_argument("--settle-seconds", type=settling_seconds, default=1.)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--output", type=Path, help="可选：保存新的 JSON 原始帧和对照报告，不覆盖已有文件")
    args = parser.parse_args(argv)
    if args.min_samples < 5:
        parser.error("--min-samples 必须至少为 5")
    if args.output is not None and args.output.suffix.lower() != ".json":
        parser.error("--output 必须是 .json")
    return args


def compare_poses(baseline, action, mapping, calibration):
    """Return observations, not an automatically inferred anatomical mapping."""
    validate_bend6_mapping(mapping)
    medians, noises = [], []
    for samples in (baseline, action):
        values = np.asarray([row["raw"] for row in samples], dtype=float)
        if (values.ndim != 2 or values.shape[0] < 5 or values.shape[1] != 11
                or not np.isfinite(values).all()):
            raise ValueError("每姿态至少需要五个有限的 11 字段原始帧")
        medians.append(np.median(values, axis=0))
        noises.append(np.percentile(values, 90, axis=0) - np.percentile(values, 10, axis=0))
    delta = medians[1] - medians[0]
    noise = np.maximum(*noises)
    if not np.isfinite(delta).all() or not np.isfinite(noise).all():
        raise ValueError("原始数据差值或噪声非有限")
    normalized = ([calibration.apply(values[:9]) for values in medians]
                  if calibration.mode == "normalized" else None)
    result = []
    for i, name in enumerate(BEND6_RAW_CHANNEL_NAMES):
        result.append(dict(
            index=i, channel=name, baseline=float(medians[0][i]), action=float(medians[1][i]),
            delta=float(delta[i]), noise_p90_p10=float(noise[i]),
            exceeds_noise=bool(abs(delta[i]) > 3 * noise[i]),
            normalized_delta=(normalized[1][i] - normalized[0][i]
                              if normalized is not None and i < 9 and name not in calibration.raw_only_channels else None),
            mapped_joints=[joint for joint, source in mapping.items() if source == name],
        ))
    return result


def main(argv=None, *, source_factory=Bend6GloveSource, clock=None, input_fn=None, print_fn=print):
    args = parse_args(argv)
    clock, input_fn = clock or time, input_fn or input
    source = None
    try:
        output = None if args.output is None else args.output.expanduser().resolve()
        if output is not None and output.exists():
            raise ValueError(f"文件已存在，未覆盖：{output}")
        path = args.sensor_config.expanduser().resolve()
        data, config = capture_config(path)
        calibration_path = data["sensors"]["glove"].get("calibration_path")
        calibration = Bend6Calibration(None if calibration_path is None else load_bend6_calibration(
            path.parent / Path(calibration_path).expanduser()))
        source = source_factory(config.port, config.baudrate, config.timeout_s,
                                side=config.source_options.get("side", "right"),
                                packet_delimiter=config.source_options.get("packet_delimiter", ";"),
                                filter={"enabled": False}, clock=clock)
        expected = config.bend6_mapping[args.joint]
        index = BEND6_RAW_CHANNEL_NAMES.index(expected)
        print_fn(f"Bend6 通道检查：只读取 {config.port}；不连接机器人，不修改标定或映射。")
        print_fn(f"本次动作 {args.joint}，当前配置使用 raw[{index}] {expected}。索引从 0 开始。")
        print_fn(f"采集原始值、关闭滤波；已有标定模式={calibration.mode}，仅用于显示归一化差值。")
        source.start()
        poses = {}
        for pose, instruction in (("baseline", "自然伸直手指"), ("action", ACTIONS[args.joint])):
            input_fn(f"请{instruction}，准备好后按 Enter：")
            print_fn(f"等待 {args.settle_seconds:g}s 后采集 {args.duration:g}s，请保持姿势。")
            clock.sleep(args.settle_seconds)
            poses[pose] = collect_pose(source, config, args.duration, args.min_samples,
                                       clock=clock, print_fn=print_fn)
        rows = compare_poses(poses["baseline"], poses["action"], config.bend6_mapping, calibration)
        print_fn("[Bend6 mapping] raw索引 通道：伸直 -> 动作；差值；归一化差值；当前控制关节")
        for row in rows:
            change = row["normalized_delta"]
            normalized_text = "不可用" if change is None else f"{change:+.3f}"
            targets = ",".join(row["mapped_joints"]) or ("reserved，不参与控制" if row["index"] >= 9 else "未映射")
            print_fn(f"raw[{row['index']}] {row['channel']}: {row['baseline']:.2f} -> {row['action']:.2f}; "
                     f"delta={row['delta']:+.2f}; normalized_delta={normalized_text}; -> {targets}; "
                     f"noise={row['noise_p90_p10']:.2f}")
        print_fn("此表只说明输入变化及当前软件路由，不能证明 GR3 物理槽位正确。"
                 "各通道量程不同且可能串扰，不能仅按最大差值自动交换通道。")
        if output is not None:
            report = dict(joint=args.joint, sensor_config=str(path), port=config.port,
                          side=config.source_options.get("side", "right"),
                          mapping=config.bend6_mapping, calibration_mode=calibration.mode,
                          poses=poses, channels=rows, robot_motion_verified=False)
            write_new(output, json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
            print_fn(f"检查报告：{output}")
        return 0
    except (KeyboardInterrupt, EOFError):
        print_fn("\n检查已取消。")
        return 130
    except Exception as exc:
        print_fn(f"ERROR [Bend6 mapping] {type(exc).__name__}: {exc}")
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
