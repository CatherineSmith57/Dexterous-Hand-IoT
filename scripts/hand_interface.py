"""hand_interface.py — 底层硬件与上层团队的联调接口

用法:
    from hand_interface import init, do_gesture, get_status, cleanup

    init()                     # 初始化并连接
    do_gesture("hand_open")    # 执行手势
    status = get_status()      # 获取状态
    cleanup()                  # 关闭
"""

import sys
from pathlib import Path

# 确保能导入 third_party 下的 orca_core
PROJECT_ROOT = Path(__file__).resolve().parents[1]
third_party_path = str(PROJECT_ROOT / "third_party" / "orca_core")
if third_party_path not in sys.path:
    sys.path.insert(0, third_party_path)

from orca_core import OrcaHand

hand = None
_connected = False
_calibrated = False


def init(config_path: str | None = None) -> bool:
    """初始化并连接机械手

    Args:
        config_path: 配置文件路径，默认用 config_safe.yaml
    Returns:
        是否连接成功
    """
    global hand, _connected, _calibrated

    if config_path is None:
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

    hand = OrcaHand(config_path=config_path)
    success, msg = hand.connect()
    print(f"[hand_interface] connect: {msg}")
    if not success:
        _connected = False
        return False
    _connected = True

    hand.init_joints()
    _calibrated = hand.calibrated
    print(f"[hand_interface] calibrated: {_calibrated}")
    return True


def do_gesture(
    gesture_name: str,
    hold_time_sec: float = 2.0,
    return_to_neutral: bool = False,
) -> dict:
    """执行手势

    Args:
        gesture_name: "hand_open" | "hand_close" | "pinch_grasp"
        hold_time_sec: 保持时间(秒)
        return_to_neutral: 完成后是否回中
    Returns:
        执行结果 dict (符合接口协议)
    """
    global _connected
    if not _connected or hand is None:
        return {
            "success": False,
            "execution_status": "failed",
            "error_code": 1001,
            "error_message": "hand is not connected",
        }

    import time

    try:
        if gesture_name == "hand_open":
            hand.set_joint_positions({
                "thumb_mcp": 0, "thumb_dip": 0,
                "index_mcp": 0, "index_pip": 0,
                "middle_mcp": 0, "middle_pip": 0,
                "ring_mcp": 0, "ring_pip": 0,
                "pinky_mcp": 0, "pinky_pip": 0,
            }, num_steps=25)
        elif gesture_name == "hand_close":
            hand.set_joint_positions({
                "thumb_mcp": 80, "thumb_dip": 60,
                "index_mcp": 85, "index_pip": 90,
                "middle_mcp": 85, "middle_pip": 90,
                "ring_mcp": 85, "ring_pip": 90,
                "pinky_mcp": 85, "pinky_pip": 90,
            }, num_steps=25)
        elif gesture_name == "pinch_grasp":
            hand.set_joint_positions({
                "thumb_mcp": 70, "thumb_dip": 50,
                "index_mcp": 80, "index_pip": 60,
            }, num_steps=25)
        else:
            return {
                "success": False,
                "execution_status": "failed",
                "error_code": 1003,
                "error_message": f"unknown gesture: {gesture_name}",
            }

        # 保持指定时间
        time.sleep(hold_time_sec)

        if return_to_neutral:
            hand.set_neutral_position()

        return {
            "success": True,
            "execution_status": "completed",
            "current_action": gesture_name,
            "connected": _connected,
            "calibrated": _calibrated,
            "error_code": 0,
            "error_message": "",
        }

    except Exception as e:
        return {
            "success": False,
            "execution_status": "failed",
            "error_code": 1007,
            "error_message": str(e),
        }


def get_status() -> dict:
    """获取机械手当前状态"""
    global _connected
    if hand is None:
        return {"connected": False}

    try:
        currents = hand.get_motor_current(as_dict=True)
        temps = hand.get_motor_temp(as_dict=True)
        positions = hand.get_motor_pos(as_dict=True)
    except Exception:
        currents = {}
        temps = {}
        positions = {}

    return {
        "connected": _connected,
        "calibrated": _calibrated,
        "motor_currents": currents,
        "motor_temps": temps,
        "motor_positions": positions,
    }


def cleanup():
    """断开连接并清理"""
    global hand, _connected, _calibrated
    if hand is not None:
        try:
            hand.stop_task()
        except Exception:
            pass
        try:
            success, msg = hand.disconnect()
            print(f"[hand_interface] disconnect: {msg}")
        except Exception as e:
            print(f"[hand_interface] disconnect error: {e}")
        hand = None
        _connected = False
        _calibrated = False

# 放在 hand_interface.py 里

def do_joint_command(joint_positions: dict, hold_time_sec: float = 3.0) -> dict:
    """
    接收上层传过来的关节角度字典，驱动对应的关节。

    参数示例:
        joint_positions = {
            "thumb_mcp": 45.0,    # 单位：度
            "index_mcp": 60.0,
            ...
        }

    返回:
        {
            "status": "completed" | "failed",
            "error_code": 0 | 1001 | 1002 | 1007,
            "message": "描述信息"
        }
    """
    # 1. 检查连接
    if not _connected or hand is None:
        return {"status": "failed", "error_code": 1001, "message": "Hand not connected"}

    # 2. 检查校准
    if not _calibrated:
        return {"status": "failed", "error_code": 1002, "message": "Hand not calibrated"}

    # 3. 验证关节名是否合法
    valid_joints = {
        "wrist",                     # ID 1
        "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip",   # ID 17,14,15,16
        "index_abd", "index_mcp", "index_pip",                 # ID 4,3,2
        "middle_abd", "middle_mcp", "middle_pip",              # ID 5,10,9
        "ring_abd", "ring_mcp", "ring_pip",                    # ID 6,7,8
        "pinky_abd", "pinky_mcp", "pinky_pip",                 # ID 13,12,11
    }

    invalid = [j for j in joint_positions if j not in valid_joints]
    if invalid:
        return {
            "status": "failed",
            "error_code": 1003,
            "message": f"Invalid joint names: {invalid}"
        }

    # 4. 检查角度是否在 ROM 范围内
    rom_limits = {
        "wrist":        (-65, 35),
        "thumb_cmc":    (-45, 33),
        "thumb_abd":    (-18, 55),
        "thumb_mcp":    (-60, 90),
        "thumb_dip":    (-55, 107),
        "index_abd":    (-30, 25),
        "index_mcp":    (-60, 100),
        "index_pip":    (-15, 107),
        "middle_abd":   (-27, 27),
        "middle_mcp":   (-60, 100),
        "middle_pip":   (-15, 107),
        "ring_abd":     (-27, 27),
        "ring_mcp":     (-60, 100),
        "ring_pip":     (-15, 107),
        "pinky_abd":    (-30, 30),
        "pinky_mcp":    (-60, 100),
        "pinky_pip":    (-15, 107),
    }

    out_of_range = []
    for joint, angle in joint_positions.items():
        lo, hi = rom_limits[joint]
        if angle < lo or angle > hi:
            out_of_range.append(f"{joint}: {angle}° (range {lo}~{hi}°)")

    if out_of_range:
        return {
            "status": "failed",
            "error_code": 1003,
            "message": "Joint angles out of range: " + "; ".join(out_of_range)
        }

    # 5. 执行关节运动
    try:
        hand.set_joint_positions(
            joint_positions,
            num_steps=25
        )

        import time
        time.sleep(hold_time_sec)

        return {
            "success": True,
            "execution_status": "completed",
            "error_code": 0,
            "error_message": ""
        }
    except Exception as e:
        return {
            "status": "failed",
            "error_code": 1007,
            "message": str(e)
        }
