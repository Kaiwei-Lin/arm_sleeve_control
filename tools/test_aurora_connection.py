#!/usr/bin/env python3
"""Aurora 连接测试：按回车进入 PdStand，再缓慢前抬双臂 30°。"""
import math
import os
import time


ARM_GROUPS = ("left_manipulator", "right_manipulator")
SHOULDER_PITCH_LIMIT = 2.9671  # GR3 官方肩部 pitch 限位，单位 rad。


def is_standing(client):
    if client.get_fsm_state() != 2:
        return False
    pose = client.get_stand_pose()
    return (pose is not None and len(pose) == 4
            and math.isfinite(pose[3]) and pose[3] > 100)


def read_arm_positions(client):
    positions = {}
    for group in ARM_GROUPS:
        values = client.get_group_state(group, key="position")
        if values is None or len(values) != 7 or not all(math.isfinite(v) for v in values):
            raise RuntimeError(f"{group} 未收到有效的 7 关节位置反馈")
        positions[group] = list(values)
    return positions


def main():
    # 在导入 SDK 和创建 DDS 客户端之前设置传输方式。
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    from fourier_aurora_client import AuroraClient

    client = AuroraClient.get_instance(domain_id=123, robot_name="gr3")
    if client is None:
        raise RuntimeError("Aurora 客户端初始化失败，请检查 SDK 日志和机器人连接")
    try:
        time.sleep(1.0)
        input("按 Enter 切换到 PdStand（站立状态）...")
        client.set_fsm_state(2)
        deadline = time.monotonic() + 5.0
        while not is_standing(client):
            if time.monotonic() >= deadline:
                raise RuntimeError("等待 PdStand 超时：需要 FSM 2 且 stable_level > 100")
            time.sleep(0.1)

        input("按 Enter 将双臂从当前姿态缓慢前抬 30°（约 3 秒）...")
        initial = read_arm_positions(client)
        delta = -math.radians(30)  # 当前 GR3 标定：肩部 index 0，负方向为前抬。
        for group, position in initial.items():
            if not (-SHOULDER_PITCH_LIMIT <= position[0] <= SHOULDER_PITCH_LIMIT
                    and -SHOULDER_PITCH_LIMIT <= position[0] + delta <= SHOULDER_PITCH_LIMIT):
                raise RuntimeError(f"{group} 前抬 30° 会超出肩部限位")

        # 从当前反馈平滑起停；完整发送两臂各 7 个关节，其余关节保持原位置。
        for step in range(301):
            if not is_standing(client):
                raise RuntimeError("机器人已离开稳定站立状态，停止发送抬臂指令")
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
