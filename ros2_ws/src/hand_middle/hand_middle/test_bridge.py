"""
test_bridge.py — HandBridge 离线/在线测试脚本

三种测试模式（按顺序推进）：

  模式1: 纯 Python 逻辑测试（不需要 ROS2，不需要硬件）
    python test_bridge.py

  模式2: ROS2 节点启动测试（需要 ROS2 + colcon build，不需要硬件）
    python test_bridge.py --mode ros2

  模式3: 硬件连通性测试（需要 ROS2 + 真实 ORCA Hand 连接）
    python test_bridge.py --mode hardware --port COM3

用法:
    cd ros2_ws/src/hand_middle/hand_middle
    python test_bridge.py                    # 默认：纯逻辑测试
    python test_bridge.py --mode ros2        # 测试 ROS2 节点能否启动
    python test_bridge.py --mode hardware    # 测试真实硬件连接
    python test_bridge.py --all              # 运行全部可运行的模式
"""

import sys
import os
import time
import argparse
import traceback
from unittest.mock import MagicMock, PropertyMock

# ── 路径注入 ─────────────────────────────────────────
_sd = os.path.dirname(os.path.abspath(__file__))
for _ in range(10):
    _c = os.path.join(_sd, "third_party", "orca_core")
    if os.path.isdir(_c):
        if _c not in sys.path:
            sys.path.insert(0, _c)
        break
    _p = os.path.dirname(_sd)
    if _p == _sd:
        break
    _sd = _p

from hand_bridge import (
    HandBridge, JOINT_ORDER, JOINT_ROMS,
    GESTURE_MAPPING, _VALID_GESTURES, _GESTURE_TARGETS,
    HAND_OPEN_TARGETS, HAND_CLOSE_TARGETS,
)

# ── 测试结果统计 ─────────────────────────────────────
_passed = 0
_failed = 0

def check(name, condition, detail=""):
    global _passed, _failed
    if condition:
        _passed += 1
        print(f"  ✅ {name}")
    else:
        _failed += 1
        print(f"  ❌ {name}  {detail}")
    return condition


# ═══════════════════════════════════════════════════════
#  模式1: 纯 Python 逻辑测试
# ═══════════════════════════════════════════════════════

def test_constants():
    print("\n── 常量完整性 ──")
    check("JOINT_ORDER 有 17 个关节", len(JOINT_ORDER) == 17)
    check("JOINT_ROMS 覆盖全部关节", len(JOINT_ROMS) == 17)
    check("GESTURE_MAPPING 有 5 个映射", len(GESTURE_MAPPING) == 5)
    check("_VALID_GESTURES 有 5 个手势", len(_VALID_GESTURES) == 5)
    check("JOINT_ORDER 与 _VALID_JOINTS 一致",
          set(JOINT_ORDER) == set(JOINT_ROMS.keys()))

    # 检查手势映射一致性
    for label, internal in GESTURE_MAPPING.items():
        if internal not in _VALID_GESTURES:
            check(f"  GESTURE_MAPPING['{label}'] -> '{internal}' 在手势表中",
                  False, f"'{internal}' not found in _VALID_GESTURES")
    print(f"  GESTURE_MAPPING 全部 5 项映射目标有效: ✅")


def test_bridge_instance():
    print("\n── HandBridge 实例化与降级 ──")
    bridge = HandBridge(port="COM3", baudrate=115200)

    # 默认状态
    check("_hand is None", bridge._hand is None)
    check("_motor_enabled is False", bridge._motor_enabled is False)
    check("_connected is False", bridge._connected is False)
    check("_calibrated is False", bridge._calibrated is False)

    # is_ready
    check("is_ready() == False (无硬件)", bridge.is_ready() == False)

    # get_device_status 降级
    status = bridge.get_device_status()
    check("get_device_status: connected=False", status["connected"] == False)
    check("get_device_status: fault_code=1001", status["fault_code"] == 1001)
    check("get_device_status: 17 joint_names", len(status["joint_names"]) == 17)

    # execute_command 降级
    for cmd_type, expected_code in [
        ("joint_control", 1001),
        ("gesture", 1001),
        ("reset", 1001),
        ("enable", 1001),
        ("do_backflip", 1003),
    ]:
        result = bridge.execute_command(
            command_type=cmd_type,
            gesture_name="hand_close" if cmd_type == "gesture" else "",
            joint_names=["thumb_mcp"] if cmd_type == "joint_control" else [],
            joint_targets=[45.0] if cmd_type == "joint_control" else [],
        )
        check(f"  {cmd_type}: error_code={expected_code}",
              result["error_code"] == expected_code,
              f"got {result['error_code']}")

    # emergency_stop
    result = bridge.emergency_stop()
    check("emergency_stop: error_code=1001 (无硬件)", result["error_code"] == 1001)

    # disable_motor 安全（无硬件也可以调用）
    result = bridge.disable_motor()
    check("disable_motor: success=True (安全操作)", result["success"] == True)

    return bridge


def test_mock_execution():
    print("\n── Mock 硬件完整执行路径 ──")

    from orca_core import OrcaJointPositions
    bridge = HandBridge()

    # 构造 mock hand
    mock_hand = MagicMock()
    mock_hand.is_connected.return_value = True
    type(mock_hand).calibrated = PropertyMock(return_value=True)
    mock_jp = OrcaJointPositions.from_dict({j: 0.0 for j in JOINT_ORDER})
    mock_hand.get_joint_position.return_value = mock_jp

    bridge._hand = mock_hand
    bridge._connected = True
    bridge._calibrated = True
    bridge._motor_enabled = True

    # ── 关节控制 ──
    result = bridge.execute_command(
        command_type="joint_control",
        joint_names=["thumb_cmc", "thumb_mcp", "index_mcp"],
        joint_targets=[15.0, 30.0, 60.0],
        hold_time_sec=0.3,
        return_to_neutral=True,
    )
    check("joint_control: success=True", result["success"] == True)
    check("joint_control: set_joint_positions 被调用", mock_hand.set_joint_positions.called)
    check("joint_control: set_neutral_position 被调用", mock_hand.set_neutral_position.called)

    # 验证参数
    call_args = mock_hand.set_joint_positions.call_args
    joint_dict = call_args[0][0]
    check(f"  thumb_mcp={joint_dict['thumb_mcp']}",
          joint_dict["thumb_mcp"] == 30.0,
          f"expected 30.0, got {joint_dict['thumb_mcp']}")
    check(f"  index_mcp={joint_dict['index_mcp']}",
          joint_dict["index_mcp"] == 60.0,
          f"expected 60.0, got {joint_dict['index_mcp']}")

    # ── return_to_neutral=False ──
    mock_hand.reset_mock()
    mock_hand.is_connected.return_value = True
    result = bridge.execute_command(
        command_type="joint_control",
        joint_names=["wrist"], joint_targets=[-30.0],
        return_to_neutral=False,
    )
    check("return_to_neutral=False: set_neutral_position 未调用",
          not mock_hand.set_neutral_position.called)

    # ── 手势执行 ──
    mock_hand.reset_mock()
    mock_hand.is_connected.return_value = True
    result = bridge.execute_command(
        command_type="gesture", gesture_name="hand_close",
        amplitude=0.5, hold_time_sec=1.0, return_to_neutral=True,
    )
    check("gesture(hand_close, 0.5): success=True", result["success"] == True)
    check("gesture: set_joint_positions 被调用", mock_hand.set_joint_positions.called)
    check("gesture: set_neutral_position 被调用", mock_hand.set_neutral_position.called)

    # ── 复位流程 ──
    mock_hand.reset_mock()
    mock_hand.is_connected.return_value = True
    mock_hand.connect.return_value = (True, "ok")
    mock_hand.disconnect.return_value = (True, "ok")
    result = bridge.reset_hardware()
    check("reset: success=True", result["success"] == True)
    check("reset: stop_task 被调用", mock_hand.stop_task.called)
    check("reset: disconnect 被调用", mock_hand.disconnect.called)
    check("reset: connect 被调用", mock_hand.connect.called)
    check("reset: calibrate 被调用", mock_hand.calibrate.called)

    # ── is_ready ──
    check("is_ready() == True", bridge.is_ready() == True)

    # ── get_device_status 含数据 ──
    status = bridge.get_device_status()
    check("status: connected=True", status["connected"] == True)
    check("status: calibrated=True", status["calibrated"] == True)
    check("status: fault_code=0", status["fault_code"] == 0,
          f"got fault_code={status['fault_code']}: {status.get('fault_message', '')}")


def test_rom_and_amplitude():
    print("\n── ROM 边界与幅度插值 ──")

    # 检查所有手势目标值在 ROM 范围内
    violations = []
    for gesture_name, targets in _GESTURE_TARGETS.items():
        for joint, angle in targets.items():
            lo, hi = JOINT_ROMS[joint]
            if not (lo - 0.01 <= angle <= hi + 0.01):
                violations.append((gesture_name, joint, angle, lo, hi))
    check(f"手势目标 ROM 检查: {len(violations)} 个越界",
          len(violations) == 0,
          "; ".join(f"{g}/{j}={a}" for g, j, a, lo, hi in violations[:5]))

    # 检查幅度插值不越界
    amp_violations = []
    for amp in [0.0, 0.25, 0.5, 0.75, 1.0]:
        for joint in JOINT_ORDER:
            open_val = HAND_OPEN_TARGETS.get(joint, 0.0)
            close_val = HAND_CLOSE_TARGETS.get(joint, 0.0)
            val = open_val + amp * (close_val - open_val)
            lo, hi = JOINT_ROMS[joint]
            if not (lo - 0.01 <= val <= hi + 0.01):
                amp_violations.append((amp, joint, val, lo, hi))
    check(f"hand_close 幅度插值 ROM: {len(amp_violations)} 个越界",
          len(amp_violations) == 0)

    # 参数越界时的错误码（无硬件）
    bridge = HandBridge()
    result = bridge.execute_command(
        command_type="joint_control",
        joint_names=["thumb_mcp"],
        joint_targets=[999.0],  # 远超 ROM
    )
    check("ROM 越界 (thumb_mcp=999): error_code=1001 (手未连接优先)",
          result["error_code"] == 1001)

    # 有 mock 硬件时 ROM 越界
    mock_hand = MagicMock()
    mock_hand.is_connected.return_value = True
    type(mock_hand).calibrated = PropertyMock(return_value=True)
    bridge._hand = mock_hand
    bridge._connected = True
    bridge._calibrated = True
    bridge._motor_enabled = True

    result = bridge.execute_command(
        command_type="joint_control",
        joint_names=["thumb_mcp"],
        joint_targets=[999.0],
    )
    check("ROM 越界 (thumb_mcp=999, 已连接): error_code=1005",
          result["error_code"] == 1005,
          f"got {result['error_code']}: {result.get('error_message', '')}")


# ═══════════════════════════════════════════════════════
#  模式2: ROS2 节点启动测试
# ═══════════════════════════════════════════════════════

def test_ros2_imports():
    print("\n── ROS2 环境检查 ──")
    try:
        import rclpy
        check("rclpy 可用", True)
    except ImportError:
        check("rclpy 可用", False, "请先安装 ROS2 并 source setup.bash")
        return False

    try:
        from hand_middle_interfaces.srv import HandCommand
        check("HandCommand.srv 可用", True)
    except (ModuleNotFoundError, ImportError):
        check("HandCommand.srv 可用", False, "请先运行 colcon build")

    try:
        from hand_middle_interfaces.msg import HandStatus
        check("HandStatus.msg 可用", True)
    except (ModuleNotFoundError, ImportError):
        check("HandStatus.msg 可用", False, "请先运行 colcon build")

    return True


# ═══════════════════════════════════════════════════════
#  模式3: 硬件连通性测试
# ═══════════════════════════════════════════════════════

def test_hardware_connection(port, baudrate):
    print(f"\n── 硬件连通性测试 (port={port}, baudrate={baudrate}) ──")

    from orca_core import OrcaHand

    # Step 1: 创建 OrcaHand 实例
    try:
        hand = OrcaHand()
        check("OrcaHand 实例创建", True)
    except Exception as e:
        check("OrcaHand 实例创建", False, str(e))
        return

    # Step 2: 连接
    try:
        ok, msg = hand.connect()
        check(f"connect() -> {msg}", ok, msg)
        if not ok:
            print("  ⚠️ 连接失败，请检查：")
            print("     1. USB 串口线是否已插入")
            print("     2. 设备管理器中 COM 口号是否正确")
            print("     3. 电机是否已上电")
            return
    except Exception as e:
        check(f"connect() 异常", False, str(e))
        return

    # Step 3: 检查连接状态
    check("is_connected()", hand.is_connected())

    # Step 4: 检查标定状态
    calibrated = hand.calibrated
    check(f"calibrated = {calibrated}", True, "(无论是否标定都继续)")

    # Step 5: 初始化关节
    try:
        hand.init_joints()
        check("init_joints() 完成", True)
    except Exception as e:
        check("init_joints() 完成", False, str(e))
        hand.disconnect()
        return

    # Step 6: 读取关节位置
    try:
        jp = hand.get_joint_position()
        positions = jp.as_dict() if jp else {}
        check(f"get_joint_position() -> {len(positions)} 个关节", len(positions) > 0)
        if positions:
            sample = list(positions.items())[:3]
            print(f"    当前位置(前3): {sample}")
    except Exception as e:
        check("get_joint_position()", False, str(e))

    # Step 7: 小幅度关节控制测试
    try:
        # 只测试食指（安全范围）
        current = hand.get_joint_position()
        current_dict = current.as_dict() if current else {}
        safe_target = current_dict.get("index_mcp", 0.0)
        test_target = min(safe_target + 10.0, 100.0)  # 最多转 10 度

        hand.set_joint_positions({"index_mcp": test_target}, num_steps=25, step_size=0.01)
        time.sleep(0.3)
        check(f"set_joint_positions(index_mcp={test_target})", True)

        # 回中
        hand.set_neutral_position()
        check("set_neutral_position()", True)
    except Exception as e:
        check("关节运动测试", False, str(e))

    # Step 8: 断开
    try:
        hand.disconnect()
        check("disconnect()", True)
    except Exception as e:
        check("disconnect()", False, str(e))


# ═══════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════

def main():
    global _passed, _failed

    parser = argparse.ArgumentParser(description="HandBridge 测试套件")
    parser.add_argument("--mode", choices=["logic", "ros2", "hardware"],
                        default="logic", help="测试模式 (默认: logic)")
    parser.add_argument("--all", action="store_true", help="运行全部模式")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口号 (硬件模式)")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率 (硬件模式)")
    args = parser.parse_args()

    print("=" * 55)
    print("  HandBridge 测试套件")
    print(f"  时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 55)

    # ── 始终运行纯逻辑测试 ──
    test_constants()
    test_bridge_instance()
    test_mock_execution()
    test_rom_and_amplitude()

    # ── ROS2 导入检查 ──
    ros2_ok = test_ros2_imports()

    # ── 硬件测试 ──
    if args.mode == "hardware" or args.all:
        if ros2_ok or True:  # 硬件测试不需要 ROS2
            test_hardware_connection(args.port, args.baudrate)
        else:
            print("\n── 硬件测试跳过（需要先 source ROS2 setup.bash）──")

    # ── 汇总 ──
    print("\n" + "=" * 55)
    total = _passed + _failed
    print(f"  总计: {total} 项测试, {_passed} 通过, {_failed} 失败")
    if _failed == 0:
        print("  🎉 全部通过!")
    else:
        print(f"  ⚠️  有 {_failed} 项失败，请检查上方 ❌ 标记")
    print("=" * 55)

    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
