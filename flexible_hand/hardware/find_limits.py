"""
find_limits.py --- 手动找电机限位

用法：
  # 找 Motor 3（食指 MCP）的两个方向限位
  python flexible_hand/find_limits.py --id 3 --direction flex

步骤：
  1. 使能扭矩
  2. 每次按 Enter，电机走一步（delta 步长）
  3. 走到机械限位时记录位置
  4. 两个方向都记录完后写入 calibration.yaml

安全：
  - 默认 delta=10（约 0.9 度），速度 15、扭矩 80
  - 随时 Ctrl+C 停止
  - 每次按键才动一步
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "flexible_hand_realtime_package"))

from motor_control import MotorControl

# 关节中文名
JOINT_NAMES = {
    1: "手腕",
    2: "食指PIP", 3: "食指MCP", 4: "食指ABD",
    5: "中指ABD",
    6: "无名指ABD", 7: "无名指MCP", 8: "无名指PIP",
    9: "中指PIP", 10: "中指MCP",
    11: "小指PIP", 12: "小指MCP", 13: "小指ABD",
    14: "拇指ABD", 15: "拇指MCP", 16: "拇指DIP", 17: "拇指CMC",
}

DIRECTION_NAMES = {1: "正转 (+)", -1: "反转 (-)"}


def find_limit(mc: MotorControl, motor_id: int, direction: int,
               delta: int = 10, speed: int = 15, torque: int = 80):
    """手动步进找限位

    Args:
        direction: 1=正转, -1=反转
    """
    joint_name = JOINT_NAMES.get(motor_id, f"Motor {motor_id}")
    dir_name = DIRECTION_NAMES[direction]
    print(f"\n{'='*60}")
    print(f"  Motor {motor_id} ({joint_name}) — {dir_name} 方向")
    print(f"  步长: {delta} 步, 速度: {speed}, 扭矩: {torque}")
    print(f"{'='*60}")
    print("  每按一次 Enter 走一步")
    print("  输入 's' + Enter 记录当前位置作为限位")
    print("  输入 'q' + Enter 退出")
    print()

    pos, ok, msg = mc.read_position(motor_id)
    if not ok:
        print(f"  读取位置失败: {msg}")
        return None
    print(f"  当前位置: {pos}")
    print()

    limit_pos = None
    step_count = 0

    try:
        while True:
            cmd = input(f"  [{step_count}] 按 Enter 走一步（s=记录, q=退出）: ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 's':
                pos, ok, msg = mc.read_position(motor_id)
                if ok:
                    limit_pos = pos
                    print(f"\n  ✅ 记录限位: {pos}")
                else:
                    print(f"  读取失败: {msg}")
                break

            # 走一步
            ok, msg, new_pos = mc.move_jog(
                motor_id, direction * delta,
                speed=speed, torque=torque, wait=True
            )
            if ok:
                step_count += 1
                print(f"    步 {step_count}: 位置 {new_pos}")
            else:
                print(f"    ⚠️  {msg} (可能是堵转/限位了)")
                # 再试一次小步确认
                ok2, msg2, new_pos2 = mc.move_jog(
                    motor_id, direction * delta,
                    speed=5, torque=50, wait=True
                )
                if not ok2:
                    print(f"    确认堵转: {msg2}")
                    print(f"    建议输入 's' 记录当前位置作为限位")
                else:
                    print(f"    还能动: {new_pos2}")

    except KeyboardInterrupt:
        print("\n  用户中断")

    return limit_pos


def main():
    import argparse
    parser = argparse.ArgumentParser(description="手动找电机限位")
    parser.add_argument("--id", type=int, required=True, help="电机 ID")
    parser.add_argument("--delta", type=int, default=10, help="每步步长")
    parser.add_argument("--speed", type=int, default=15, help="速度")
    parser.add_argument("--torque", type=int, default=80, help="扭矩")
    parser.add_argument("--port", type=str, default="COM5", help="串口")
    args = parser.parse_args()

    mc = MotorControl(port=args.port)
    if not mc.connect():
        print("[FAIL] Cannot connect")
        return 1

    joint_name = JOINT_NAMES.get(args.id, f"Motor {args.id}")

    try:
        # 先使能扭矩
        ok, msg = mc.set_torque(args.id, True)
        if not ok:
            print(f"[FAIL] Torque enable failed: {msg}")
            return 1
        print(f"[OK] Motor {args.id} ({joint_name}) torque enabled")

        # 找正转限位
        pos_fwd = find_limit(mc, args.id, 1,
                             delta=args.delta, speed=args.speed, torque=args.torque)

        # 找反转限位
        pos_rev = find_limit(mc, args.id, -1,
                             delta=args.delta, speed=args.speed, torque=args.torque)

        # 输出结果
        print(f"\n{'='*60}")
        print(f"  Motor {args.id} ({joint_name}) 限位结果:")
        if pos_rev is not None and pos_fwd is not None:
            print(f"    反转限位 (min): {min(pos_rev, pos_fwd)}")
            print(f"    正转限位 (max): {max(pos_rev, pos_fwd)}")
            print(f"    范围: {abs(pos_fwd - pos_rev)} 步")
        elif pos_fwd is not None:
            print(f"    正转限位: {pos_fwd} (反转未记录)")
        elif pos_rev is not None:
            print(f"    反转限位: {pos_rev} (正转未记录)")
        else:
            print("    未记录任何限位")

        print(f"\n  手动写入到 hardware/calibration.yaml:")
        if pos_rev is not None and pos_fwd is not None:
            print(f"    motor_limits:")
            print(f"      {args.id}: [{min(pos_rev, pos_fwd)}, {max(pos_rev, pos_fwd)}]")

    finally:
        ok, msg = mc.set_torque(args.id, False)
        mc.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
