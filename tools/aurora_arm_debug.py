#!/usr/bin/env python3
"""数字菜单 Aurora 手臂调试：默认离线预览，--simulate 模拟，--execute 真机。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import unicodedata

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sleeve_arm.control.aurora_motion import AuroraMotionService, preview_joint
from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import fake_session, gr3_fake_profile
from sleeve_arm.robot.aurora_profile import load_aurora_profile


MENU_ACTIONS = {
    "1": ("向前抬", "shoulder_flexion", 1),
    "2": ("向后抬", "shoulder_flexion", -1),
    "3": ("侧摆（向外抬）", "shoulder_abduction", 1),
    "4": ("向内收", "shoulder_abduction", -1),
    "5": ("屈肘", "elbow_flexion", 1),
    "6": ("伸肘", "elbow_flexion", -1),
}


@dataclass(frozen=True)
class ArmCommand:
    side: str
    joint: str
    angle_deg: float


def show_menu(side):
    print(f"\n{'右' if side == 'right' else '左'}臂动作菜单：")
    for number, (label, _, _) in MENU_ACTIONS.items():
        print(f"  {number}. {label}")
    print("  8. 查看当前角度\n  0. 退出")


def parse_angle(value):
    try:
        magnitude = float(unicodedata.normalize("NFKC", str(value)).strip())
    except ValueError as exc:
        raise ValueError("请输入角度数字，例如 30 或 12.5，不需要输入“度”。") from exc
    if not math.isfinite(magnitude) or not 0 <= magnitude <= 180:
        raise ValueError("角度必须在 0～180 度内；实际可用范围还受机器人限位约束。")
    return magnitude


def make_command(side, action, angle):
    if side not in ("right", "left") or action not in MENU_ACTIONS:
        raise ValueError("请选择菜单中的动作编号。")
    _, joint, sign = MENU_ACTIONS[action]
    return ArmCommand(side, joint, sign * parse_angle(angle))


def read_angle(input_fn):
    while True:
        text = input_fn("请输入角度（度，0～180，直接回车取消）：").strip()
        if not text:
            print("已取消，返回菜单。")
            return None
        try:
            return parse_angle(text)
        except ValueError as exc:
            print(f"输入错误：{exc}")


def snapshot(robot, group):
    # Disabled while awaiting user input: re-establish freshness after any pause.
    state = robot.session.fresh_snapshot({group.name})
    return list(state.groups[group.name].position), state.fsm


def show_status(profile, group, robot):
    if robot is None:
        print("离线预览：没有真实反馈；用 aurora_sdk_doctor.py --connect 做只读连接检查。")
        return
    vector, fsm = snapshot(robot, group)
    print(json.dumps({
        "simulation": profile.simulated, "fsm": fsm, "group": group.name,
        "smooth_limits": profile.smooth_limits,
        "sdk_position_rad": vector,
        "semantic_deg": {j.name: math.degrees(j.from_sdk(vector[j.index])) for j in group.joints},
        "velocity_limits_rad_s": {
            name: {"command": limits.max_velocity, "feedback": limits.max_feedback_velocity}
            for name, limits in robot.config.joints.items()
        },
    }, ensure_ascii=False, indent=2))


def run_command(args, profile, group, command, controller):
    robot = None if controller is None else controller.robot
    before, fsm = (None, None) if robot is None else snapshot(robot, group)
    preview = preview_joint(profile, side=command.side, joint=command.joint,
                            angle_deg=command.angle_deg, reference=args.reference,
                            current_sdk=before, fsm=fsm)
    print(json.dumps({"simulation": profile.simulated, **preview}, ensure_ascii=False, indent=2))
    if not (args.execute or args.simulate):
        return True
    if preview["blocking_reasons"]:
        print("拒绝执行：" + "; ".join(preview["blocking_reasons"]))
        return False

    target = (preview["clamped_target"] if profile.smooth_limits else preview["requested_semantic_position"])
    print(f"执行目标：{command.side} / {command.joint} → {math.degrees(target):.3f}°，按 profile 允许的最大速度运行")

    # --execute authorizes motion; still check PdStand/stance without changing FSM.
    robot.prepare_control_mode(operator_confirmed=True)
    key = robot.joint_key(command.side, command.joint)
    controller.enable()
    try:
        # Keep the preview's absolute target, including for relative commands.
        # The controller advances at its velocity/step limits until submitted.
        result = AuroraMotionService(controller).move_joints(
            {key: target}, reference="neutral",
        )
        measured_state = controller.read_joint_states()[key]
        measured = measured_state.position
        print(json.dumps({
            "simulation": profile.simulated, "submitted": result.submitted,
            "arrived": result.arrived, "delivery_confirmed": result.delivery_confirmed,
            "target_deg": math.degrees(target), "feedback_deg": math.degrees(measured),
            "error_deg": math.degrees(measured - target), "elapsed_s": result.elapsed_s,
            "feedback_velocity_rad_s": measured_state.velocity,
        }, ensure_ascii=False, indent=2))
        if result.arrived:
            print("模拟反馈到位。" if args.simulate else "已观察到提交后的新鲜关节反馈到位；SDK 不提供送达回执。")
        else:
            print("目标尚未到位：跟踪限制正在等待反馈跟上，可继续输入下一条动作。")
        return True
    finally:
        # Disable publishing during idle input without recreating the SDK singleton.
        controller.disable()


def main(argv=None, *, input_fn=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("aurora", "fake"), default="aurora")
    parser.add_argument("--profile", type=Path, default=ROOT / "configs/robot_aurora.yaml")
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--reference", choices=("neutral", "current"), default="neutral",
                        help="neutral: 目标角度相对标定零位；current: 相对当前姿态的增量")
    parser.add_argument("--action", choices=(*MENU_ACTIONS, "8", "0"),
                        help="只运行一个菜单项；1～6 需同时提供 --angle-deg")
    parser.add_argument("--angle-deg", type=float, help="单次动作的角度；交互模式会提示输入")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="连接真机并直接执行动作，无需输入 YES")
    mode.add_argument("--simulate", action="store_true", help="仅用于 --backend fake")
    args = parser.parse_args(argv)
    if args.execute and args.backend != "aurora":
        parser.error("fake 使用 --simulate，不接受 --execute")
    if args.simulate and args.backend != "fake":
        parser.error("--simulate 要求 --backend fake")
    if args.action in MENU_ACTIONS and args.angle_deg is None:
        parser.error("--action 1～6 必须同时提供 --angle-deg")
    if args.angle_deg is not None:
        if args.action not in MENU_ACTIONS:
            parser.error("--angle-deg 必须与 --action 1～6 一起使用")
        try:
            parse_angle(args.angle_deg)
        except ValueError as exc:
            parser.error(str(exc))
    read = input_fn or input
    controller = None
    code = 0
    try:
        profile = gr3_fake_profile() if args.backend == "fake" else load_aurora_profile(args.profile)
        group = profile.selected_groups((args.side,))[0]
        if args.execute or args.simulate:
            # Reject an unverified site profile before SDK import or DDS connection.
            profile.validate(execute=True, simulation=args.simulate, groups=(group,))
        if args.backend == "fake" or args.execute:
            session = fake_session(profile=profile, execute=args.simulate) if args.backend == "fake" else None
            robot = factory.create_robot(
                "aurora-fake" if args.backend == "fake" else "aurora", profile=profile,
                side=args.side, session=session, execute=args.execute, operator_confirmed=False,
            )
            controller = SafeArmController(robot, robot.config, clock=robot.session.clock)
            print("连接模拟客户端并检查反馈…" if args.backend == "fake" else "连接 Aurora SDK 并检查反馈…")
            controller.connect()
            show_status(profile, group, robot)
        print("模式：" + ("真机执行" if args.execute else "模拟执行" if args.simulate else "预览，不下发动作"))
        print("角度含义：" + ("相对标定零位的目标角度" if args.reference == "neutral" else "相对当前反馈的增量"))
        print("先选择方向，再输入角度；每次只改变一个关节，其余关节保持。")
        while True:
            if args.action is None:
                show_menu(args.side)
                action = unicodedata.normalize("NFKC", read("请选择动作编号：")).strip()
            else:
                action = args.action
            if action == "0":
                break
            if action == "8":
                show_status(profile, group, None if controller is None else controller.robot)
            elif action in MENU_ACTIONS:
                print(f"已选择：{'右' if args.side == 'right' else '左'}臂{MENU_ACTIONS[action][0]}")
                angle = args.angle_deg if args.action is not None else read_angle(read)
                if angle is None:
                    continue
                command = make_command(args.side, action, angle)
                success = run_command(args, profile, group, command, controller)
                if args.action is not None and not success:
                    code = 1
            else:
                print("输入错误：请选择 1～6、8 或 0。")
            if args.action is not None:
                break
    except EOFError:
        print("输入结束，关闭会话。")
    except KeyboardInterrupt:
        print("已中断，停止本程序下发；停止发布不等于硬件急停。")
        code = 130
    except Exception as exc:
        print(f"调试失败：{type(exc).__name__}: {exc}", file=sys.stderr)
        code = 1
    finally:
        if controller is not None:
            try:
                controller.shutdown()
            except BaseException as exc:
                print(f"清理失败：{exc}", file=sys.stderr)
                code = 1
    return code


if __name__ == "__main__":
    raise SystemExit(main())
