"""quick_calibrate.py — 最小校准脚本

只做校准，不做 tension，不走 setup.py 的完整流程。

用法:
    python scripts/quick_calibrate.py

注意:
    校准前请确保手处于中立位置，手指能自由活动。
    按 Ctrl+C 可随时停止。
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
third_party_path = str(PROJECT_ROOT / "third_party" / "orca_core")
if third_party_path not in sys.path:
    sys.path.insert(0, third_party_path)

from orca_core import OrcaHand
from orca_core.hand_config import OrcaHandConfig


def main():
    config_path = str(
        PROJECT_ROOT
        / "third_party"
        / "orca_core"
        / "orca_core"
        / "models"
        / "v2"
        / "orcahand_right"
        / "config_safe.yaml"
    )

    print("=" * 60)
    print("  最小校准脚本")
    print("=" * 60)

    # 1. 加载配置，强制 motor_type = feetech
    import dataclasses
    config = OrcaHandConfig.from_config_path(config_path)
    config = dataclasses.replace(config, motor_type="feetech")
    print(f"  motor_type: {config.motor_type}")
    print(f"  port: {config.port}")

    # 2. 创建 hand 实例
    hand = OrcaHand(config=config)

    # 3. 连接
    print("\n连接机械手...")
    success, msg = hand.connect()
    print(f"  connect: {msg}")
    if not success:
        print("  连接失败，退出")
        return 1

    # 4. 校准（包含手腕）
    print("\n开始校准...")
    print("  按 Ctrl+C 可跳过当前步骤")
    try:
        hand.calibrate(force_wrist=True)
        print("\n校准完成！")
    except KeyboardInterrupt:
        print("\n校准被用户中断")
    except Exception as e:
        print(f"\n校准出错: {e}")
        import traceback
        traceback.print_exc()

    # 5. 查看校准结果
    print("\n" + "=" * 60)
    print("  校准结果")
    print("=" * 60)
    calib = hand.calibration
    print(f"  总体校准状态: {'✅ 已校准' if calib.calibrated else '❌ 未完成'}")
    print(f"  手腕校准状态: {'✅ 已校准' if calib.wrist_calibrated else '❌ 未完成'}")
    print()

    for motor_id in config.motor_ids:
        limits = calib.motor_limits_dict.get(motor_id, [None, None])
        ratio = calib.joint_to_motor_ratios_dict.get(motor_id, None)
        joint_name = config.motor_to_joint_dict.get(motor_id, "?")
        if limits[0] is not None and limits[1] is not None and ratio is not None and ratio != 0:
            print(f"  ✅ Motor {motor_id:2d} ({joint_name:12s}): limits=[{limits[0]:.3f}, {limits[1]:.3f}], ratio={ratio:.3f}")
        else:
            print(f"  ❌ Motor {motor_id:2d} ({joint_name:12s}): 未校准")

    # 6. 断开
    hand.disconnect()
    print("\n断开连接")

    return 0 if calib.calibrated else 1


if __name__ == "__main__":
    sys.exit(main())
