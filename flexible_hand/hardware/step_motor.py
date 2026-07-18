"""
step_motor.py --- 纯底层手动步进（不依赖 motor_control）

用法：
  # Motor 3 每次走 50 步
  python flexible_hand/hardware/step_motor.py --id 3 --delta 50 --speed 60

操作：
  按 Enter = 走一步
  输入 r + Enter = 反转方向走一步
  输入 s + Enter = 显示当前位置
  输入 q + Enter = 退出
"""

import sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "third_party" / "orca_core" / "orca_core" / "hardware"))
from feetech import PortHandler, sms_sts

JOINT_NAMES = {
    1: "手腕", 2: "食指PIP", 3: "食指MCP", 4: "食指ABD",
    5: "中指ABD", 6: "无名指ABD", 7: "无名指MCP", 8: "无名指PIP",
    9: "中指PIP", 10: "中指MCP", 11: "小指PIP", 12: "小指MCP",
    13: "小指ABD", 14: "拇指ABD", 15: "拇指MCP", 16: "拇指DIP", 17: "拇指CMC",
}

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--delta", type=int, default=50)
    parser.add_argument("--speed", type=int, default=60)
    parser.add_argument("--torque", type=int, default=200)
    parser.add_argument("--port", default="COM5")
    args = parser.parse_args()

    ph = PortHandler(args.port)
    ph.baudrate = 1000000
    if not ph.openPort():
        print("Open port failed")
        return 1
    pk = sms_sts(ph)

    mid = args.id
    name = JOINT_NAMES.get(mid, "?")
    direction = 1  # 1=正转, -1=反转

    pos, _, _ = pk.ReadPos(mid)
    print(f"Motor {mid} ({name})  当前位置: {pos}")
    print(f"delta={args.delta}, speed={args.speed}, torque={args.torque}")
    print("Enter=走一步, r=换方向, s=显示位置, q=退出")
    print()

    pk.write1ByteTxRx(mid, 40, 1)  # 使能扭矩
    time.sleep(0.1)

    try:
        while True:
            cmd = input(f"[dir={'+' if direction>0 else '-'}] > ").strip().lower()
            if cmd == 'q':
                break
            elif cmd == 's':
                pos, _, _ = pk.ReadPos(mid)
                print(f"  位置: {pos}")
                continue
            elif cmd == 'r':
                direction *= -1
                print(f"  方向切换为 {'+' if direction>0 else '-'}")
                continue

            target = min(4095, max(0, pos + direction * args.delta))
            pk.WritePosEx(mid, target, args.speed, 20, args.torque)
            time.sleep(1.0)
            new_pos, r, e = pk.ReadPos(mid)
            actual_delta = new_pos - pos
            print(f"  目标={target}, 实际位置={new_pos}, 移动={actual_delta}")
            if abs(actual_delta) < 5:
                print(f"  *** 可能到限位了 ***")
            pos = new_pos

    except KeyboardInterrupt:
        print()
    finally:
        pk.write1ByteTxRx(mid, 40, 0)
        ph.closePort()
        print("done")

if __name__ == "__main__":
    sys.exit(main())
