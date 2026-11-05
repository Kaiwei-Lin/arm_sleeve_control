#!/usr/bin/env python3
"""Aurora 悬挂抬臂测试（FSM 11）；--connect-only 只检查连接。"""
import argparse
from importlib import metadata
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sleeve_arm.robot.aurora_sdk import configure_dds_environment, initialize_client


ARM_GROUPS = ("left_manipulator", "right_manipulator")
UPPER_BODY_USER_CMD = 11
SHOULDER_PITCH_LIMIT = 2.9671  # GR3 官方肩部 pitch 限位，单位 rad。


def read_arm_positions(client):
    positions = {}
    for group in ARM_GROUPS:
        values = client.get_group_state(group, key="position")
        if values is None or len(values) != 7 or not all(math.isfinite(v) for v in values):
            raise RuntimeError(f"{group} 未收到有效的 7 关节位置反馈")
        positions[group] = list(values)
    return positions


def connect_client():
    # 私有初始化兼容只适用于已核对的 0.1.8，先检查实际安装版本。
    version = metadata.version("fourier_aurora_client")
    if version != "0.1.8":
        raise RuntimeError(f"此测试需要 fourier_aurora_client 0.1.8，当前为 {version}")
    configure_dds_environment()
    from fourier_aurora_client import AuroraClient

    # 只使用关节位置命令。完整核对 publisher 列表后，允许唯一的
    # velocity_cmd 未匹配；其他 publisher/subscriber/service 仍须匹配。
    profile = SimpleNamespace(
        sdk_version=version,
        allow_missing_velocity_cmd=True,
        connection={"domain_id": 123, "robot_name": "gr3", "is_ros_compatible": False},
    )
    client = initialize_client(AuroraClient, profile)
    if client is None:
        raise RuntimeError("Aurora 客户端初始化失败，请检查 SDK 日志中未匹配的端点和机器人连接")
    return client


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect-only", action="store_true",
                        help="初始化完整 SDK 并读取状态，不切换 FSM、不发送运动指令")
    args = parser.parse_args(argv)
    client = connect_client()
    try:
        time.sleep(1.0)
        if args.connect_only:
            positions = read_arm_positions(client)
            print(f"连接成功，当前 FSM：{client.get_fsm_state()}，站立反馈：{client.get_stand_pose()}")
            for group, position in positions.items():
                print(f"{group} 关节反馈（rad）：{position}")
            print("连接检查完成，未发送状态切换或运动指令。")
            return 0
        input("机器人需保持悬挂支撑；按 Enter 切换到 UpperBodyUserCmd（FSM 11，下身零扭矩）...")
        client.set_fsm_state(UPPER_BODY_USER_CMD)
        deadline = time.monotonic() + 5.0
        # FSM 11 用于悬挂上身控制，不使用 PdStand 的 stable_level 判据。
        while True:
            fsm = client.get_fsm_state()
            if fsm == UPPER_BODY_USER_CMD:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f"等待 UpperBodyUserCmd 超时：需要 FSM 11，当前 FSM={fsm}")
            time.sleep(0.1)
        read_arm_positions(client)

        input("按 Enter 将双臂从当前姿态缓慢前抬 30°（约 3 秒）...")
        initial = read_arm_positions(client)
        delta = -math.radians(30)  # 当前 GR3 标定：肩部 index 0，负方向为前抬。
        for group, position in initial.items():
            if not (-SHOULDER_PITCH_LIMIT <= position[0] <= SHOULDER_PITCH_LIMIT
                    and -SHOULDER_PITCH_LIMIT <= position[0] + delta <= SHOULDER_PITCH_LIMIT):
                raise RuntimeError(f"{group} 前抬 30° 会超出肩部限位")

        # 从当前反馈平滑起停；完整发送两臂各 7 个关节，其余关节保持原位置。
        for step in range(301):
            fsm = client.get_fsm_state()
            if fsm != UPPER_BODY_USER_CMD:
                raise RuntimeError(f"机器人已离开 UpperBodyUserCmd（当前 FSM={fsm}），停止发送抬臂指令")
            read_arm_positions(client)
            alpha = step / 300
            smooth = alpha * alpha * (3.0 - 2.0 * alpha)
            position_cmd = {
                group: [position[0] + delta * smooth, *position[1:]]
                for group, position in initial.items()
            }
            client.set_group_cmd(position_cmd=position_cmd)
            time.sleep(0.01)

        time.sleep(1.0)
        for group, position in read_arm_positions(client).items():
            print(f"{group} 肩部反馈：{math.degrees(position[0]):.1f}°（SDK 坐标）")
        print("抬臂指令发送完成；请结合实际动作确认，SDK 调用返回不代表关节已到位。")
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
