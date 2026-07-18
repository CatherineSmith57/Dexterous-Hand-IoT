"""固定展示手势定义。

所有值均为机械手关节角，单位为 degree，不是电机 raw。

设计目标：
- 优先保证手势辨识度和稳定性；
- 使用完整 17 关节姿态，避免未指定关节保留上一动作；
- 角度保守，不直接压到 joint ROM 的极限；
- 后续只需修改本文件中的数值即可进行外观微调。
"""

from __future__ import annotations

from copy import deepcopy
from typing import Dict, Mapping


JOINT_NAMES: tuple[str, ...] = (
    "wrist",
    "thumb_cmc",
    "thumb_abd",
    "thumb_mcp",
    "thumb_dip",
    "index_abd",
    "index_mcp",
    "index_pip",
    "middle_abd",
    "middle_mcp",
    "middle_pip",
    "ring_abd",
    "ring_mcp",
    "ring_pip",
    "pinky_abd",
    "pinky_mcp",
    "pinky_pip",
)


def _base_pose() -> Dict[str, float]:
    """生成完整 17 关节基础姿态。"""
    return {
        "wrist": 0.0,
        "thumb_cmc": -32.0,
        "thumb_abd": -15.0,
        "thumb_mcp": -15.0,
        "thumb_dip": -15.0,
        "index_abd":-10.0,
        "index_mcp": -45.0,
        "index_pip": -8.0,
        "middle_abd":-20.0,
        "middle_mcp": -45.0,
        "middle_pip": -8.0,
        "ring_abd": 0.0,
        "ring_mcp": -45.0,
        "ring_pip": -8.0,
        "pinky_abd": 0.0,
        "pinky_mcp": 70.0,  
        "pinky_pip": -8.0,
        # thumb_cmc:-32.0,
        # thumb_abd:-18.0,
        # thumb_mcp:-20.0,
        # thumb_dip=-10.0,
        # index_abd=-10.0,
        # index_mcp=20.0,      # ← 加上
        # index_pip=0.0,      # ← 加上
        # middle_abd=-20.0,
        # middle_mcp=0.0,     # ← 加上
        # middle_pip=0.0,     # ← 加上
        # ring_abd=10.0,
        # ring_mcp=0.0,       # ← 加上
        # ring_pip=0.0,       # ← 加上
        # pinky_abd=10.0,
        # pinky_mcp=70.0,      # ← 加上
        # pinky_pip=0.0,      # ← 加上
    }


def _pose(**updates: float) -> Dict[str, float]:
    result = _base_pose()
    unknown = sorted(set(updates) - set(JOINT_NAMES))
    if unknown:
        raise ValueError(
            f"Unknown joints in fixed gesture: {unknown}"
        )
    result.update(
        {
            name: float(value)
            for name, value in updates.items()
        }
    )
    return result


# 伸直与弯曲采用保守角度，避免直接触碰 ROM 极限。
FIXED_GESTURES: Dict[str, Dict[str, float]] = {
    # 张开手掌，作为开场、过渡和结束姿态。
    "hi": _pose(
        thumb_cmc=-32.0,
        thumb_abd=-18.0,
        thumb_mcp=-20.0,
        thumb_dip=-10.0,
        index_abd=-10.0,
        index_mcp=20.0,      # ← 加上
        index_pip=0.0,      # ← 加上
        middle_abd=-20.0,
        middle_mcp=0.0,     # ← 加上
        middle_pip=0.0,     # ← 加上
        ring_abd=-10.0,
        ring_mcp=0.0,       # ← 加上
        ring_pip=0.0,       # ← 加上
        pinky_abd=10.0,
        pinky_mcp=70.0,      # ← 加上
        pinky_pip=0.0,      # ← 加上
    ),

    # V / Yeah：食指和中指伸直，无名指与小指弯曲，拇指收拢。
    "yeah": _pose(
        # thumb_cmc=-15.0,
        # thumb_abd=5.0,
        # thumb_mcp=60.0,
        # thumb_dip=45.0,

        # thumb_cmc=-5.0,
        # thumb_abd=15.0,
        # thumb_mcp=65.0,
        # thumb_dip=55.0,
        # index_abd=-20.0,
        # index_mcp=-5.0,
        # index_pip=-8.0,
        # middle_abd=-20.0,
        # middle_mcp=-20.0,
        # middle_pip=-8.0,
        # ring_abd=0.0,
        # ring_mcp=76.0,
        # ring_pip=86.0,
        # pinky_abd=0.0,
        # pinky_mcp=78.0,
        # pinky_pip=88.0,

        wrist=0.0,
        thumb_cmc=-5.0,
        thumb_abd=14.9,
        thumb_mcp=64.9,
        thumb_dip=54.9,
        index_abd=-20.0,
        index_mcp=-5.0,
        index_pip=-8.1,
        middle_abd=-20.0,
        middle_mcp=-19.9,
        middle_pip=-8.1,
        ring_abd=10.0,
        ring_mcp=76.0,
        ring_pip=86.0,
        pinky_abd=10.0,
        pinky_mcp=77.8,
        pinky_pip=88.0,
    ),

    # 中文数字 6：拇指和小指伸直，食指、中指、无名指弯曲。
    "six": _pose(
        thumb_cmc=33.0,
        thumb_abd=65.0,
        thumb_mcp=55.0,
        thumb_dip=-10.0,
        index_abd=-20.0,
        index_mcp=78.0,
        index_pip=88.0,
        middle_abd=-20.0,
        middle_mcp=78.0,
        middle_pip=88.0,
        ring_abd=8.0, 
        ring_mcp=-45.0, 
        ring_pip=-8.0,  
        pinky_abd= 10.0,
        pinky_mcp= 78.0,
        pinky_pip=88.0,
    ),

    # 指向：食指伸直，其余手指弯曲。
    "point": _pose(
        # thumb_cmc=-10.0,
        # thumb_abd=10.0,
        # thumb_mcp=55.0,
        # thumb_dip=45.0,
        thumb_cmc=-5.0,
        thumb_abd=15.0,
        thumb_mcp=65.0,
        thumb_dip=55.0,
        index_abd=-20.0,
        index_mcp=-20.0,
        index_pip=-8.0,
        middle_abd=-20.0,
        middle_mcp=78.0,
        middle_pip=88.0,
        ring_abd=10.0,
        ring_mcp=78.0,
        ring_pip=88.0,
        pinky_abd=10.0,
        pinky_mcp=78.0,
        pinky_pip=88.0,
    ),

    # 握拳：四指弯曲，拇指收拢。
    "fist": _pose(
        thumb_cmc=-5.0,
        thumb_abd=15.0,
        thumb_mcp=65.0,
        thumb_dip=55.0,
        index_abd=-10.0,
        index_mcp=82.0,
        index_pip=92.0,
        middle_abd=-20.0,
        middle_mcp=82.0,
        middle_pip=92.0,
        ring_abd=10.0,
        ring_mcp=82.0,
        ring_pip=92.0,
        pinky_abd=10.0,
        pinky_mcp=82.0,
        pinky_pip=92.0,
    ),
}


GESTURE_LABELS: Dict[str, str] = {
    "hi": "Hi / 张开手掌",
    "yeah": "Yeah / V",
    "six": "数字 6",
    "point": "Point / 指向",
    "fist": "Fist / 握拳",
}


GESTURE_ALIASES: Dict[str, str] = {
    "1": "hi",
    "2": "yeah",
    "3": "six",
    "4": "point",
    "5": "fist",
    "6": "six",
    "open": "hi",
    "hand_open": "hi",
    "v": "yeah",
    "victory": "yeah",
    "yeah": "yeah",
    "six": "six",
    "point": "point",
    "finger": "point",
    "fist": "fist",
    "close": "fist",
    "hand_close": "fist",
}


def normalize_gesture_name(name: str) -> str:
    normalized = str(name).strip().lower()
    normalized = GESTURE_ALIASES.get(
        normalized,
        normalized,
    )
    if normalized not in FIXED_GESTURES:
        raise KeyError(
            f"Unknown fixed gesture: {name!r}. "
            f"Available: {list(FIXED_GESTURES)}"
        )
    return normalized


def get_fixed_gesture(
    name: str,
) -> Dict[str, float]:
    """返回姿态副本，避免调用者修改全局定义。"""
    normalized = normalize_gesture_name(name)
    return deepcopy(FIXED_GESTURES[normalized])


def list_fixed_gestures() -> Dict[str, str]:
    return {
        name: GESTURE_LABELS[name]
        for name in FIXED_GESTURES
    }


def validate_fixed_gestures() -> None:
    """静态检查：每个固定手势必须恰好包含 17 个关节。"""
    expected = set(JOINT_NAMES)

    for gesture_name, pose in FIXED_GESTURES.items():
        actual = set(pose)
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)

        if missing or extra:
            raise ValueError(
                f"Gesture {gesture_name!r} invalid: "
                f"missing={missing}, extra={extra}"
            )

        for joint_name, value in pose.items():
            if not isinstance(value, (int, float)):
                raise TypeError(
                    f"{gesture_name}.{joint_name} "
                    f"must be numeric"
                )


validate_fixed_gestures()
