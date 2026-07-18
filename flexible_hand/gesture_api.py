"""固定手势对上层接口。

优先调用新版 hand_interface.hold_snapshot()；
若项目仍是旧兼容接口，则退化到 do_joint_command()。
"""

from __future__ import annotations

from typing import Dict

try:
    from . import hand_interface
    from .fixed_gestures import (
        get_fixed_gesture,
        list_fixed_gestures,
        normalize_gesture_name,
    )
except ImportError:
    import hand_interface
    from fixed_gestures import (
        get_fixed_gesture,
        list_fixed_gestures,
        normalize_gesture_name,
    )


def execute_fixed_gesture(
    gesture_name: str,
) -> dict:
    """执行一个完整固定手势并保持。

    Args:
        gesture_name:
            hi / yeah / six / point / fist，
            也支持数字 1～5；数字 6 作为 six 的别名。

    Returns:
        统一结果字典。
    """
    name = normalize_gesture_name(gesture_name)
    pose = get_fixed_gesture(name)

    # 新版：清空实时队列并固定当前目标。
    hold_snapshot = getattr(
        hand_interface,
        "hold_snapshot",
        None,
    )
    if callable(hold_snapshot):
        result = hold_snapshot(pose)
        result = dict(result)
        result.setdefault("gesture", name)
        return result

    # 旧兼容版：阻塞式下发完整 17 关节。
    do_joint_command = getattr(
        hand_interface,
        "do_joint_command",
        None,
    )
    if callable(do_joint_command):
        result = do_joint_command(
            pose,
            hold_time_sec=0.0,
        )
        result = dict(result)
        success = (
            result.get("status") == "completed"
            or result.get("success") is True
        )
        return {
            "success": success,
            "gesture": name,
            "status": result.get(
                "status",
                "completed" if success else "failed",
            ),
            "error_code": result.get(
                "error_code",
                0 if success else 1007,
            ),
            "message": result.get(
                "message",
                "",
            ),
            "pose_deg": pose,
            "raw_result": result,
        }

    raise RuntimeError(
        "hand_interface has neither hold_snapshot() "
        "nor do_joint_command()"
    )


__all__ = [
    "execute_fixed_gesture",
    "get_fixed_gesture",
    "list_fixed_gestures",
]
