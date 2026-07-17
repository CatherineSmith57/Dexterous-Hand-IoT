"""
hand_config.py --- 17 关节映射、软限位与手势配置

使用步骤：
1. 先继续使用 motor_control.py 单关节测试；
2. 为每个关节记录 raw_min / raw_max / neutral_raw；
3. 若视觉侧输出角度，再填写 angle_min_deg / angle_max_deg；
4. 最后填写 GESTURES_RAW 中的标准手势。

重要：
- 未填写软限位的关节不会被 hand_interface.py 驱动；
- 不要把 0 和 4095 直接当作机械关节限位；
- inverted 表示“角度增大时，电机 raw 值是否反向变化”。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


PORT = "COM5"
BAUDRATE = 1_000_000

# 视觉识别通常以 15~30 FPS 更新。总线先从 20 Hz 开始联调。
CONTROL_HZ = 20.0

# 保守默认运动参数；在单关节测试完成后再逐步调整。
DEFAULT_SPEED = 20
DEFAULT_ACC = 10
DEFAULT_TORQUE = 100

# 相邻两次实时命令允许的最大 raw 跳变量。
# 该限制会把视觉抖动或偶发错误帧变成渐进运动。
MAX_RAW_STEP_PER_UPDATE = 60

# 多久没有收到视觉帧后触发看门狗。
# WATCHDOG_ACTION 可选 "hold" 或 "disable_torque"。
WATCHDOG_TIMEOUT_SEC = 1.0
WATCHDOG_ACTION = "hold"

# 到达目标位置的默认判定。
POSITION_TOLERANCE_RAW = 12
MOVE_TIMEOUT_SEC = 3.0


@dataclass(frozen=True)
class JointSpec:
    name: str
    motor_id: int

    # 必须通过逐关节实验填写。
    raw_min: Optional[int] = None
    raw_max: Optional[int] = None
    neutral_raw: Optional[int] = None

    # 视觉侧若直接输出角度，必须填写关节角范围。
    angle_min_deg: Optional[float] = None
    angle_max_deg: Optional[float] = None

    inverted: bool = False

    @property
    def raw_calibrated(self) -> bool:
        return (
            self.raw_min is not None
            and self.raw_max is not None
            and self.raw_min < self.raw_max
        )

    @property
    def angle_calibrated(self) -> bool:
        return (
            self.raw_calibrated
            and self.angle_min_deg is not None
            and self.angle_max_deg is not None
            and self.angle_min_deg < self.angle_max_deg
        )

    @property
    def fully_calibrated(self) -> bool:
        return self.angle_calibrated and self.neutral_raw is not None


# 关节名称与现有 ORCA Hand 机械结构中的 ID 对应关系。
# 只保留映射；具体限位需要你们逐关节重新测量。
JOINT_SPECS: Dict[str, JointSpec] = {
    "wrist": JointSpec("wrist", 1),
    "index_pip": JointSpec("index_pip", 2),
    "index_mcp": JointSpec("index_mcp", 3),
    "index_abd": JointSpec("index_abd", 4),
    "middle_abd": JointSpec("middle_abd", 5),
    "ring_abd": JointSpec("ring_abd", 6),
    "ring_mcp": JointSpec("ring_mcp", 7),
    "ring_pip": JointSpec("ring_pip", 8),
    "middle_pip": JointSpec("middle_pip", 9),
    "middle_mcp": JointSpec("middle_mcp", 10),
    "pinky_pip": JointSpec("pinky_pip", 11),
    "pinky_mcp": JointSpec("pinky_mcp", 12),
    "pinky_abd": JointSpec("pinky_abd", 13),
    "thumb_abd": JointSpec("thumb_abd", 14),
    "thumb_mcp": JointSpec("thumb_mcp", 15),
    "thumb_dip": JointSpec("thumb_dip", 16),
    "thumb_cmc": JointSpec("thumb_cmc", 17),
}

MOTOR_TO_JOINT = {
    spec.motor_id: name
    for name, spec in JOINT_SPECS.items()
}


# 填写示例（不要直接照抄数值）：
#
# JOINT_SPECS["index_mcp"] = JointSpec(
#     name="index_mcp",
#     motor_id=3,
#     raw_min=1760,
#     raw_max=2480,
#     neutral_raw=1820,
#     angle_min_deg=0.0,
#     angle_max_deg=90.0,
#     inverted=False,
# )


# 标准手势使用 motor_id -> raw_position。
# 允许先只填写一根手指，之后逐步补成完整 17 电机手势。
GESTURES_RAW: Dict[str, Dict[int, int]] = {
    "hand_open": {},
    "hand_close": {},
    "pinch_grasp": {},
}


def get_neutral_raw() -> Dict[int, int]:
    """返回所有已填写 neutral_raw 的关节。"""
    return {
        spec.motor_id: spec.neutral_raw
        for spec in JOINT_SPECS.values()
        if spec.neutral_raw is not None
    }


def raw_calibration_complete() -> bool:
    return all(spec.raw_calibrated for spec in JOINT_SPECS.values())


def angle_calibration_complete() -> bool:
    return all(spec.angle_calibrated for spec in JOINT_SPECS.values())


def full_calibration_complete() -> bool:
    return all(spec.fully_calibrated for spec in JOINT_SPECS.values())
