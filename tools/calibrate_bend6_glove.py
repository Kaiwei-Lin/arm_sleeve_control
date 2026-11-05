#!/usr/bin/env python3
"""Capture open/fist Bend6 endpoints without importing or connecting a robot.
python tools/calibrate_bend6_glove.py \
  --sensor configs/sensors_bend6.local.yaml \
  --output configs/bend6_calibration.measured.yaml
  """
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shlex
import sys
import tempfile
import time

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import GloveConfig, PROJECT_ROOT, validate_glove_config
from sleeve_arm.sources.bend6 import (
    BEND6_CHANNEL_NAMES, BEND6_RAW_CHANNEL_NAMES, BEND6_BETWEEN_CHANNELS,
    Bend6Calibration, Bend6GloveSource, load_bend6_calibration,
)


def positive_float(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("必须为有限正数")
    return value


def settling_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError("必须为有限非负数")
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-config", "--sensor", type=Path,
                        default=PROJECT_ROOT / "configs/sensors_bend6.local.yaml")
    parser.add_argument("--output", type=Path,
                        help="新的标定 YAML/JSON；默认写入 calibrations/bend6_<时间>.yaml，不覆盖已有文件")
    parser.add_argument("--overwrite", action="store_true",
                        help="标定成功后覆盖同名输出；保留已有传感器副本的设置，仅更新标定路径")
    parser.add_argument("--duration", type=positive_float, default=3., help="每个动作采集秒数，默认 3")
    parser.add_argument("--settle-seconds", type=settling_seconds, default=1., help="回车后稳定等待秒数，默认 1")
    parser.add_argument("--min-samples", type=int, default=20, help="每动作至少需要的新帧数，默认 20")
    parser.add_argument("--min-span", type=positive_float, default=1., help="端点最小差值，原始数据单位，默认 1")
    args = parser.parse_args(argv)
    if args.min_samples < 5:
        parser.error("--min-samples 必须至少为 5")
    if args.output is None:
        args.output = PROJECT_ROOT / "calibrations" / f"bend6_{datetime.now():%Y%m%d_%H%M%S_%f}.yaml"
    if args.output.suffix.lower() not in (".yaml", ".yml", ".json"):
        parser.error("--output 必须是 .yaml、.yml 或 .json")
    return args


def capture_config(path):
    """Read only glove settings; an old/missing calibration must not block recalibration."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if (not isinstance(data, dict) or not isinstance(data.get("sensors"), dict)
            or not isinstance(data["sensors"].get("glove"), dict)):
        raise ValueError("配置缺少 sensors.glove")
    glove = dict(data["sensors"]["glove"])
    glove["calibration_path"] = None
    config = GloveConfig(**glove)
    validate_glove_config(config)
    if config.backend != "bend6" or not config.port:
        raise ValueError("标定需要 backend: bend6 和已填写的 glove.port")
    return data, config


def collect_pose(source, config, duration, min_samples, *, clock=time, print_fn=print):
    # Exclude cached data received while the operator was moving or settling.
    started = clock.monotonic()
    previous = source.latest()
    last_sequence = 0 if previous is None else previous.sequence
    first_at = last_at = None
    samples = []
    last_print = started
    while True:
        reading = source.latest()  # Thread/serial failures propagate immediately.
        now = clock.monotonic()
        if last_at is not None and now - last_at > config.stale_timeout_s:
            raise RuntimeError(f"标定断流：超过 {config.stale_timeout_s:g}s 没有新有效帧；"
                               "请检查距离/连接后重新采集，不生成标定文件")
        if reading is not None:
            if reading.sequence < last_sequence:
                raise RuntimeError("采集序号重置，请重新开始标定")
            if reading.sequence > last_sequence:
                last_sequence = reading.sequence
                if (not math.isfinite(reading.timestamp) or reading.timestamp > now
                        or (last_at is not None and reading.timestamp < last_at)):
                    raise RuntimeError("无效的手套单调时间戳")
                if reading.timestamp > started:
                    if now - reading.timestamp > config.stale_timeout_s:
                        raise RuntimeError("收到过期手套帧，请重新开始标定")
                    if (reading.status != "ok" or reading.synthetic or reading.calibration_mode != "raw"
                            or len(reading.raw) != 11 or not all(math.isfinite(v) for v in reading.raw)):
                        raise ValueError("标定必须使用真实、未归一化的 11 字段有效原始帧")
                    if first_at is None:
                        first_at = now
                        last_print = now
                    last_at = reading.timestamp
                    samples.append(dict(timestamp=reading.timestamp, sequence=reading.sequence,
                                        raw=list(reading.raw)))
        if first_at is None:
            if now - started >= config.startup_timeout_s:
                raise RuntimeError("标定等待新帧超时；请检查端口、距离和包结束符")
        elif now - first_at >= duration:
            break
        if first_at is not None and now - last_print >= 1.:
            print_fn(f"  已采集 {len(samples)} 帧，请继续保持姿势。")
            last_print = now
        clock.sleep(.005)
    if len(samples) < min_samples:
        raise RuntimeError(f"有效新帧不足：{len(samples)} < {min_samples}；请检查连接或增加 --duration")
    return samples


def derive_calibration(open_samples, fist_samples, *, min_span=1., previous=None):
    """Fit the two observed poses, not an unmeasured anatomical range."""
    summaries = {}
    for name, samples in (("open", open_samples), ("fist", fist_samples)):
        raw = np.asarray([sample["raw"] for sample in samples], dtype=float)
        if raw.ndim != 2 or raw.shape[0] < 5 or raw.shape[1] != 11 or not np.isfinite(raw).all():
            raise ValueError("每个动作至少需要五个有限的 11 字段原始帧")
        summaries[name] = dict(count=len(samples), median=np.median(raw, axis=0).tolist(),
                               p10=np.percentile(raw, 10, axis=0).tolist(),
                               p90=np.percentile(raw, 90, axis=0).tolist())
    channels, quality, issues = {}, {}, {}
    if previous is not None:
        Bend6Calibration(previous)
    for i, name in enumerate(BEND6_CHANNEL_NAMES):
        opened, closed = summaries["open"]["median"][i], summaries["fist"]["median"][i]
        low, high = sorted((opened, closed))
        span = high - low
        noise = max(summaries[pose]["p90"][i] - summaries[pose]["p10"][i] for pose in summaries)
        channels[name] = dict(raw_min=low, raw_max=high, invert=closed < opened)
        quality[name] = dict(open_median=opened, fist_median=closed, span=span, noise_p90_p10=noise)
        if name in BEND6_BETWEEN_CHANNELS:
            old = previous.get("channels", {}).get(name) if previous and previous["mode"] == "normalized" else None
            channels[name] = deepcopy(old) if old is not None else {"mode": "raw"}
            quality[name]["policy"] = "preserved" if old and old != {"mode": "raw"} else "raw_only"
            continue  # Open/fist poses do not measure finger spread; never fit their noise.
        if not math.isfinite(span) or span <= 0 or span < min_span:
            issues[name] = f"两姿态差值 {span:g} 小于要求 {min_span:g}，或端点相同"
        elif span <= 3 * noise:
            issues[name] = f"两姿态差值 {span:g} 未超过静态波动 {noise:g} 的 3 倍"
    calibration = dict(mode="normalized", channels=channels)
    if not issues:
        Bend6Calibration(calibration)  # Validate with the same code used by control.
    return calibration, summaries, quality, issues


def write_new(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(text)


def write_outputs(files, *, overwrite):
    """Stage complete files before replacement; restore earlier files on write failure."""
    if not overwrite:
        for path, text in files.items():
            write_new(path, text)
        return
    staged, backups, replaced = {}, {}, []

    def stage(path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            try:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            except BaseException:
                temporary.unlink(missing_ok=True)
                raise
        return temporary

    try:
        for path, text in files.items():
            backups[path] = stage(path, path.read_bytes()) if path.exists() else None
            staged[path] = stage(path, text.encode("utf-8"))
        for path, temporary in staged.items():
            os.replace(temporary, path)
            replaced.append(path)
    except BaseException:
        for path in reversed(replaced):
            if backups[path] is None:
                path.unlink(missing_ok=True)
            else:
                os.replace(backups[path], path)
        raise
    finally:
        for temporary in (*staged.values(), *backups.values()):
            if temporary is not None:
                temporary.unlink(missing_ok=True)


def main(argv=None, *, source_factory=Bend6GloveSource, clock=None, input_fn=None, print_fn=print):
    args = parse_args(argv)
    clock, input_fn = clock or time, input_fn or input
    source = None
    try:
        output = args.output.expanduser().resolve()
        report_path = output.with_suffix(".report.json")
        sensor_path = output.with_suffix(".sensors.yaml")
        for path in (output, report_path, sensor_path):
            if path.exists() and not args.overwrite:
                raise ValueError(f"文件已存在，未覆盖：{path}；请加 --overwrite 或换一个 --output 文件名")
        config_path = args.sensor_config.expanduser().resolve()
        if config_path in (output, report_path):
            raise ValueError("输入传感器配置不能同时作为标定或报告输出")
        sensor_data, config = capture_config(config_path)
        execution_config = config
        if args.overwrite and sensor_path.exists():
            # A raw-preview input must not erase the deployed hand poses/mapping.
            sensor_data, execution_config = capture_config(sensor_path)
            print_fn(f"覆盖模式：保留 {sensor_path} 中已有设置，仅更新 calibration_path；"
                     f"本次采集端口仍取自 {config_path}。")
        # Persist the resolved poses, even when input YAML relies on defaults.
        # Future overwrites must not silently switch to a different default pose.
        settings = sensor_data["sensors"]["glove"]
        settings["open_pose_rad"] = list(execution_config.open_pose_rad)
        settings["closed_pose_rad"] = list(execution_config.closed_pose_rad)
        settings["bend6_mapping"] = dict(execution_config.bend6_mapping)
        previous = None
        old_path = sensor_data["sensors"]["glove"].get("calibration_path")
        base_path = sensor_path.parent if args.overwrite and sensor_path.exists() else config_path.parent
        old_path = base_path / old_path if old_path else None
        for candidate in (output if args.overwrite else None, old_path):
            if candidate is not None and candidate.is_file():
                try:
                    previous = load_bend6_calibration(candidate)
                    Bend6Calibration(previous)
                    break
                except (ValueError, OSError, yaml.YAMLError) as exc:
                    previous = None
                    print_fn(f"旧标定不能复用 between 通道：{candidate}: {exc}")
        source = source_factory(config.port, config.baudrate, config.timeout_s,
                                side=config.source_options.get("side", "right"),
                                packet_delimiter=config.source_options.get("packet_delimiter", ";"),
                                filter={"enabled": False}, clock=clock)
        print_fn(f"Bend6 两动作标定：只读取 {config.port}，不连接机器人；使用 raw 数据，关闭滤波。")
        print_fn("每个动作取多帧中位数：小值写 raw_min，大值写 raw_max；自动设置 invert。")
        print_fn("两个 between 通道不检查动作幅度/噪声：保留旧标定，无旧标定则保留 raw 并禁止映射控制。")
        source.start()
        poses = {}
        for pose, instruction in (("open", "伸直五指，保持自然张开"), ("fist", "握拳，保持想要作为闭合端点的姿势")):
            input_fn(f"请{instruction}，准备好后按 Enter：")
            print_fn(f"等待 {args.settle_seconds:g}s 稳定后采集 {args.duration:g}s，请保持不动。")
            clock.sleep(args.settle_seconds)
            poses[pose] = collect_pose(source, config, args.duration, args.min_samples,
                                       clock=clock, print_fn=print_fn)
            print_fn(f"{pose} 完成：{len(poses[pose])} 个新帧。")
        calibration, summaries, quality, issues = derive_calibration(
            poses["open"], poses["fist"], min_span=args.min_span, previous=previous)
        for name, item in calibration["channels"].items():
            if name in BEND6_BETWEEN_CHANNELS:
                print_fn(f"{name}: 跳过两动作标定检查；" +
                         ("保留旧标定" if quality[name]["policy"] == "preserved" else "raw，只读，不参与控制"))
                continue
            print_fn(f"{name}: min={item['raw_min']:g}, max={item['raw_max']:g}, "
                     f"invert={item['invert']}" + (f"  [需重采] {issues[name]}" if name in issues else ""))
        report = dict(method="two_pose_medians", channel_names=BEND6_RAW_CHANNEL_NAMES,
                      sensor_config=str(config_path), port=config.port,
                      side=config.source_options.get("side", "right"), poses=poses, summaries=summaries,
                      quality=quality, candidate_channels=calibration["channels"], issues=issues,
                      usable=not issues, robot_motion_verified=False,
                      note="仅拟合伸直/握拳两个姿态；未验证拇指横摆、指间动作量程和机械串扰。")
        if issues:
            if args.overwrite:
                report_path = output.with_suffix(f".failed-{datetime.now():%Y%m%d_%H%M%S_%f}.report.json")
            write_new(report_path, json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
            print_fn(f"原始数据和质量报告：{report_path}")
            print_fn("部分通道动作幅度不足或不稳定：仅保存采集报告，未生成 normalized 标定和执行配置。"
                     "已有标定保持不变。请调整两个姿态重采；若通道不随握拳变化，需要针对该通道单独标定。")
            return 1
        data = {"glove": {"bend6": {"calibration": calibration}}}
        text = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
                if output.suffix.lower() == ".json" else yaml.safe_dump(data, sort_keys=False))
        sensor_data["sensors"]["glove"]["calibration_path"] = os.path.relpath(output, sensor_path.parent)
        write_outputs({
            report_path: json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            sensor_path: yaml.safe_dump(sensor_data, sort_keys=False, allow_unicode=True),
            output: text,
        }, overwrite=args.overwrite)
        print_fn(f"原始数据和质量报告：{report_path}")
        print_fn(f"标定已写入：{output}\n传感器配置：{sensor_path}")
        print_fn("机器人目标角度（rad，归一化 0 → 1；顺序为拇指弯曲、食指、中指、无名指、小指、拇指朝掌心）：")
        print_fn(f"  open_pose_rad={settings['open_pose_rad']}\n  closed_pose_rad={settings['closed_pose_rad']}")
        print_fn(f"  bend6_mapping={settings['bend6_mapping']}")
        print_fn("先逐指检查角度预览；两动作不代表完整手姿解耦，尤其需另行确认拇指横摆：")
        print_fn(f"python tools/debug_glove.py --sensor-config {shlex.quote(str(sensor_path))} "
                 "--glove real --duration 30")
        return 0
    except (KeyboardInterrupt, EOFError):
        print_fn("\n标定已取消。")
        return 130
    except Exception as exc:
        print_fn(f"ERROR [Bend6 calibration] {type(exc).__name__}: {exc}")
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
