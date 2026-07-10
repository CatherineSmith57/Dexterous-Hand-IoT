#!/usr/bin/env python3
"""抓握-释放循环演示（使用同步舵机写入）。

使用 WaveShare（及兼容）舵机客户端的 ``write_desired_pos_sync`` 
确保每次抓握/释放动作完全完成后再进行下一步。
这避免了基础 ``write_desired_pos`` 路径中异步轨迹被覆盖的问题。

用法:
    python scripts/grip_release.py                           # 物理手，默认配置
    python scripts/grip_release.py --mock                    # 模拟手（无需硬件）
    python scripts/grip_release.py --cycles 5                # 重复 5 次
    python scripts/grip_release.py --hold 1.5                # 每个极端位置保持 1.5 秒
"""

import argparse
import time

import numpy as np

from common import add_hand_arguments, create_hand, connect_hand, shutdown_hand
from orca_core import OrcaJointPositions


# ---------------------------------------------------------------------------
# 姿态定义
# ---------------------------------------------------------------------------

def build_open_pose(hand) -> OrcaJointPositions:
    """构造一个张开手的姿态，各关节保持在 ROM 上限附近。

    当 ORCA 手张开时：
    - 手指伸展（flexion 关节取 ROM 上限附近）
    - 外展关节张开（abd 取正方向上限的 75%）
    - 手腕保持中位（取 ROM 的 45%）
    """
    rom = hand.config.joint_roms_dict
    pose = {}
    for joint in hand.config.joint_ids:
        jmin, jmax = rom.get(joint, (0.0, 1.0))
        # 手腕：稍微偏向屈曲的中位
        if joint == "wrist":
            pose[joint] = (jmin + jmax) * 0.45
        # 外展（abduction）关节：张开时外展到 ROM 的 75%
        elif "_abd" in joint:
            pose[joint] = jmax * 0.75
        # 屈曲（flexion）关节：伸展 = ROM 高值
        else:
            pose[joint] = jmin + (jmax - jmin) * 0.85
    return OrcaJointPositions.from_dict(pose)


def build_fist_pose(hand) -> OrcaJointPositions:
    """构造一个握拳（抓握）姿态，各关节保持在 ROM 下限附近。

    当 ORCA 手握拳时：
    - 手指屈曲（flexion 关节取 ROM 下限 + 10%）
    - 外展关节收拢（取 ROM 的 15%）
    - 手腕轻微屈曲（取 ROM 的 55%）
    """
    rom = hand.config.joint_roms_dict
    pose = {}
    for joint in hand.config.joint_ids:
        jmin, jmax = rom.get(joint, (0.0, 1.0))
        if joint == "wrist":
            # 握拳时手腕轻微屈曲
            pose[joint] = (jmin + jmax) * 0.55
        elif "_abd" in joint:
            # 握拳时外展关节收拢 → 较小的值
            pose[joint] = jmin + (jmax - jmin) * 0.15
        else:
            # 屈曲关节：完全握住
            pose[joint] = jmin + (jmax - jmin) * 0.10
    return OrcaJointPositions.from_dict(pose)


# ---------------------------------------------------------------------------
# 同步运动辅助函数
# ---------------------------------------------------------------------------

def move_sync(
    hand,
    pose: OrcaJointPositions,
    *,
    num_steps: int = 10,
    step_size: float = 0.03,
    **sync_kwargs,
):
    """发送插值后的轨迹点，然后阻塞直到运动结束。

    分两阶段工作：
    1. 发送插值轨迹点（非阻塞写入 + 步进间隔 sleep）
    2. 如果底层舵机客户端支持 ``write_desired_pos_sync``，则调用它来阻塞等待运动完成；
       否则回退到速度轮询方式等待运动静止。

    Args:
        hand: OrcaHand 实例。
        pose: 目标关节位置。
        num_steps: 从当前位置到目标位置的插值步数。
        step_size: 每步之间的间隔（秒）。
        **sync_kwargs: 传递给 ``write_desired_pos_sync`` 的额外参数（如 settle 超时）。
    """
    client = getattr(hand, "_motor_client", None)

    # 阶段 1：发送插值轨迹（非阻塞写入 + sleep 间隔）
    hand.set_joint_positions(pose, num_steps=num_steps, step_size=step_size)

    # 阶段 2：如果客户端支持同步写入，用它来确保运动完成
    if client is not None and hasattr(client, "write_desired_pos_sync"):
        # 将关节位置转换回舵机空间
        motor_pos = hand._joint_to_motor_pos(pose.as_dict())
        # 过滤掉 None 项（未标定的关节）
        valid_ids = []
        valid_pos = []
        for mid, val in zip(hand.config.motor_ids, motor_pos):
            if val is not None and not np.isnan(val):
                valid_ids.append(mid)
                valid_pos.append(np.float64(val))
        if valid_ids:
            client.write_desired_pos_sync(valid_ids, np.array(valid_pos, dtype=np.float64), **sync_kwargs)
        return

    # 回退：通用的速度检测沉降逻辑（与 main_demo 相同）
    _settle_fallback(hand, **sync_kwargs)


def _settle_fallback(
    hand,
    *,
    velocity_threshold: float = 1e-3,
    settle_timeout: float = 6.0,
    settle_samples: int = 3,
    poll_interval: float = 0.03,
):
    """轮询舵机速度直到所有舵机静止（无同步 API 时的回退方案）。

    通过 read_pos_vel_cur() 读取所有舵机的速度绝对值，取最大值。
    当连续 settle_samples 次最大速度低于 velocity_threshold 时，
    认为运动已稳定。

    如果在 settle_timeout 内仍未稳定，则超时返回（不抛出异常）。

    Args:
        hand: OrcaHand 实例。
        velocity_threshold: 速度阈值（rad/s），低于此值认为静止。
        settle_timeout: 等待稳定的最大时间（秒）。
        settle_samples: 需要连续满足条件的采样次数。
        poll_interval: 每次轮询之间的间隔（秒）。
    """
    client = getattr(hand, "_motor_client", None)
    motor_ids = list(hand.config.motor_ids)

    below_count = 0
    t0 = time.monotonic()

    while time.monotonic() - t0 < settle_timeout:
        if client and hasattr(client, "read_pos_vel_cur"):
            _, vel_arr, _ = client.read_pos_vel_cur()
            max_vel = float(np.max(np.abs(vel_arr))) if len(vel_arr) else 0.0
        else:
            # 粗回退：用位置差分估算速度
            max_vel = 0.0
        if max_vel < velocity_threshold:
            below_count += 1
            if below_count >= settle_samples:
                return
        else:
            # 只要有一个采样点速度超标，重置计数器
            below_count = 0
        time.sleep(poll_interval)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main() -> int:
    """主入口：解析命令行参数，执行抓握-释放循环。

    命令行参数:
        config_path:      可选的配置文件路径，省略时使用默认内置模型。
        --mock:           使用 MockOrcaHand 代替物理手（测试用）。
        --cycles N:       重复抓握-释放的次数（默认 3）。
        --hold T:         每个极端位置的保持时间（秒，默认 1.0）。
        --num-steps N:    每次运动的插值步数（默认 10）。
        --step-size T:    插值步之间的间隔（秒，默认 0.03）。

    返回:
        退出码（0 = 成功，130 = 中断）。
    """
    parser = argparse.ArgumentParser(
        description="抓握-释放循环演示（使用同步舵机写入）。"
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--cycles", type=int, default=3,
        help="抓握+释放循环的次数（默认: 3）。"
    )
    parser.add_argument(
        "--hold", type=float, default=1.0,
        help="在每个极端位置的保持时间（秒，默认: 1.0）。"
    )
    parser.add_argument(
        "--num-steps", type=int, default=10,
        help="每次运动的插值步数（默认: 10）。"
    )
    parser.add_argument(
        "--step-size", type=float, default=0.03,
        help="插值步之间的间隔（秒，默认: 0.03）。"
    )
    args = parser.parse_args()

    # 创建手实例（使用 MockOrcaHand 或真实 OrcaHand）
    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        print("正在连接手……")
        connect_hand(hand)
        # init_joints 会执行：
        #   使能扭矩 → 设置控制模式 → 设置最大电流
        #   → 未标定则自动标定 → 移动到中位
        hand.init_joints(force_calibrate=args.mock)

        # 构造两种姿态
        open_pose = build_open_pose(hand)  # 张开
        fist_pose = build_fist_pose(hand)  # 握拳

        print(f"\n开始 {args.cycles} 次抓握-释放循环（保持 {args.hold}s）\n")

        for cycle in range(1, args.cycles + 1):
            # --- 抓握（握拳） ---
            print(f"  第 {cycle}/{args.cycles} 次循环 — 抓握中……", end=" ", flush=True)
            t0 = time.monotonic()
            move_sync(
                hand, fist_pose,
                num_steps=args.num_steps,
                step_size=args.step_size,
            )
            elapsed = time.monotonic() - t0
            print(f"完成（耗时 {elapsed:.2f}s）")
            time.sleep(args.hold)

            # --- 释放（张开） ---
            print(f"  第 {cycle}/{args.cycles} 次循环 — 释放中……", end=" ", flush=True)
            t0 = time.monotonic()
            move_sync(
                hand, open_pose,
                num_steps=args.num_steps,
                step_size=args.step_size,
            )
            elapsed = time.monotonic() - t0
            print(f"完成（耗时 {elapsed:.2f}s）")
            time.sleep(args.hold)

            print()

        # 循环结束后回到中位，使手部处于自然放松状态
        print("回到中位……")
        hand.set_neutral_position(num_steps=args.num_steps, step_size=args.step_size)

        print("完成。")
        return 0

    except KeyboardInterrupt:
        print("\n已中断。")
        return 130
    finally:
        # 无论正常结束还是异常中断，都确保清理资源
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
