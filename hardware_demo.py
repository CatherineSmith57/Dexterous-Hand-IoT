"""
hardware_demo.py — 设备组硬件调试脚本（不依赖 ROS2）

使用方法：
  1. 安装依赖（一次性）:
     pip install numpy pyyaml

  2. 插入 ORCA Hand 硬件，确认串口号:
     Linux:   ls /dev/ttyUSB* /dev/ttyACM*
     Windows: 设备管理器 → 端口 → 找 USB Serial Device (COMx)

  3. 运行:
     python3 hardware_demo.py --port /dev/ttyUSB0

测试流程：
  连接 → 标定(如需要) → 读取关节位置 → 回中位 → 简单运动 → 断开
"""

import sys
import os
import time
import argparse

# ── 路径注入 ─────────────────────────────────────────
_here = os.path.dirname(os.path.abspath(__file__))
_orca = os.path.join(_here, "third_party", "orca_core")
if os.path.isdir(_orca):
    sys.path.insert(0, _orca)

from orca_core import OrcaHand, OrcaJointPositions


def main():
    parser = argparse.ArgumentParser(description="ORCA Hand 硬件调试")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口号")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率")
    parser.add_argument("--skip-calibrate", action="store_true", help="跳过自动标定")
    args = parser.parse_args()

    print("=" * 50)
    print(f"  串口: {args.port}    波特率: {args.baudrate}")
    print("=" * 50)

    # ── 1. 连接 ───────────────────────────────────────
    print("\n[1/6] 创建 OrcaHand 实例...")
    hand = OrcaHand()

    print(f"[2/6] 连接 {args.port}...")
    ok, msg = hand.connect()
    if not ok:
        print(f"  ❌ 连接失败: {msg}")
        print("  排查: 串口线是否插好？设备管理器里 COM 口是否正确？电机上电了吗？")
        return
    print(f"  ✅ {msg}")

    # ── 2. 初始化 ─────────────────────────────────────
    print("[3/6] 初始化关节（使能扭矩 + 检查标定）...")
    if args.skip_calibrate and hand.calibrated:
        print("  已标定，跳过")
        hand.enable_torque()
    else:
        hand.init_joints()

    print(f"  已连接: {hand.is_connected()}")
    print(f"  已标定: {hand.calibrated}")

    # ── 3. 读取当前关节位置 ──────────────────────────
    print("[4/6] 读取关节位置...")
    try:
        jp = hand.get_joint_position()
        positions = jp.as_dict()
        if positions:
            for name, angle in list(positions.items())[:6]:
                print(f"  {name}: {angle:.2f}°")
            print(f"  ... 共 {len(positions)} 个关节")
    except Exception as e:
        print(f"  ⚠️ 读取失败: {e}")

    # ── 4. 回中位 ─────────────────────────────────────
    print("[5/6] 回中位...")
    try:
        hand.set_neutral_position()
        print("  ✅ 已回到中立位置")
    except Exception as e:
        print(f"  ⚠️ 回中失败: {e}")

    # ── 5. 简单运动测试 ──────────────────────────────
    print("[6/6] 食指小幅度运动测试（+10°, 然后回中）...")
    try:
        jp = hand.get_joint_position()
        current = jp.as_dict()
        safe_target = min(current.get("index_mcp", 0.0) + 10.0, 100.0)

        print(f"  index_mcp: {current.get('index_mcp', 0):.1f}° → {safe_target:.1f}°")
        hand.set_joint_positions({"index_mcp": safe_target}, num_steps=25, step_size=0.01)
        time.sleep(0.5)

        hand.set_neutral_position()
        print("  ✅ 运动测试完成")
    except Exception as e:
        print(f"  ⚠️ 运动测试失败: {e}")

    # ── 6. 断开 ──────────────────────────────────────
    print("\n断开连接...")
    hand.disconnect()
    print("✅ 硬件调试完成")


if __name__ == "__main__":
    main()
