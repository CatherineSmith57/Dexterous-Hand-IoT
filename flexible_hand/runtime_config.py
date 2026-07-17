"""
runtime_config.py --- config_safe.yaml 与人工标定数据的轻量加载器

只复用 ORCA 配置中的静态信息：
- port / baudrate
- motor_ids
- joint_to_motor_map
- reverse_joints
- joint_roms
- neutral_position

不调用 OrcaHand，不运行旧自动撞限位校准。
电机 raw 软限位由 realtime_calibration.yaml 人工填写。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Optional

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
_MODEL_DIR = (
    PROJECT_ROOT
    / "third_party"
    / "orca_core"
    / "orca_core"
    / "models"
    / "v2"
    / "orcahand_right"
)
DEFAULT_CONFIG_PATH = (
    _MODEL_DIR / "config_safe.yaml"
    if (_MODEL_DIR / "config_safe.yaml").exists()
    else _MODEL_DIR / "config.yaml"
)
DEFAULT_CALIBRATION_PATH = (
    Path(__file__).resolve().parent
    / "hardware"
    / "calibration.yaml"
)


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class JointCalibration:
    joint_name: str
    motor_id: int
    raw_min: Optional[int]
    raw_max: Optional[int]

    @property
    def valid(self) -> bool:
        return (
            self.raw_min is not None
            and self.raw_max is not None
            and 0 <= self.raw_min < self.raw_max <= 4095
        )


@dataclass(frozen=True)
class RuntimeConfig:
    config_path: Path
    calibration_path: Path
    port: str
    baudrate: int
    motor_ids: tuple[int, ...]
    joint_ids: tuple[str, ...]
    joint_to_motor_map: Dict[str, int]
    motor_to_joint_map: Dict[int, str]
    reverse_joints: frozenset[str]
    joint_roms: Dict[str, tuple[float, float]]
    neutral_position: Dict[str, float]
    calibrations: Dict[str, JointCalibration]

    @property
    def calibrated(self) -> bool:
        return (
            set(self.calibrations) == set(self.joint_ids)
            and all(
                self.calibrations[joint].valid
                for joint in self.joint_ids
            )
        )

    def validate_joint_names(
        self,
        values: Mapping[str, float],
    ) -> None:
        invalid = sorted(set(values) - set(self.joint_ids))
        if invalid:
            raise ConfigError(
                f"Invalid joint names: {invalid}"
            )

    def validate_angles(
        self,
        values: Mapping[str, float],
    ) -> None:
        self.validate_joint_names(values)

        out_of_range = []
        for joint_name, angle in values.items():
            lo, hi = self.joint_roms[joint_name]
            angle = float(angle)
            if angle < lo or angle > hi:
                out_of_range.append(
                    f"{joint_name}: {angle}° "
                    f"(range {lo}~{hi}°)"
                )

        if out_of_range:
            raise ConfigError(
                "Joint angles out of range: "
                + "; ".join(out_of_range)
            )

    def angle_to_raw(
        self,
        joint_name: str,
        angle_deg: float,
    ) -> int:
        calibration = self.calibrations.get(joint_name)
        if calibration is None or not calibration.valid:
            raise ConfigError(
                f"Joint '{joint_name}' has no valid raw calibration"
            )

        lo_deg, hi_deg = self.joint_roms[joint_name]
        angle = min(
            hi_deg,
            max(lo_deg, float(angle_deg)),
        )
        ratio = (angle - lo_deg) / (hi_deg - lo_deg)

        if joint_name in self.reverse_joints:
            ratio = 1.0 - ratio

        assert calibration.raw_min is not None
        assert calibration.raw_max is not None

        raw = round(
            calibration.raw_min
            + ratio
            * (calibration.raw_max - calibration.raw_min)
        )
        return max(
            calibration.raw_min,
            min(calibration.raw_max, raw),
        )

    def angles_to_raw(
        self,
        joint_angles: Mapping[str, float],
    ) -> Dict[int, int]:
        self.validate_angles(joint_angles)
        return {
            self.joint_to_motor_map[joint_name]:
                self.angle_to_raw(joint_name, angle)
            for joint_name, angle in joint_angles.items()
        }

    def neutral_raw(self) -> Dict[int, int]:
        return self.angles_to_raw(self.neutral_position)


def _read_yaml(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(str(path))

    with path.open("r", encoding="utf-8-sig") as file:
        data = yaml.safe_load(file) or {}

    if not isinstance(data, dict):
        raise ConfigError(
            f"YAML root must be a mapping: {path}"
        )
    return data


def load_runtime_config(
    config_path: str | Path | None = None,
    calibration_path: str | Path | None = None,
) -> RuntimeConfig:
    resolved_config = (
        Path(config_path).resolve()
        if config_path is not None
        else DEFAULT_CONFIG_PATH.resolve()
    )
    resolved_calibration = (
        Path(calibration_path).resolve()
        if calibration_path is not None
        else DEFAULT_CALIBRATION_PATH.resolve()
    )

    base = _read_yaml(resolved_config)
    calibration_data = _read_yaml(resolved_calibration)

    rom_key = (
        "joint_roms"
        if "joint_roms" in base
        else "joint_roms_dict"
    )
    required = {
        "port",
        "baudrate",
        "motor_ids",
        "joint_ids",
        "joint_to_motor_map",
        rom_key,
        "neutral_position",
    }
    missing = sorted(required - set(base))
    if missing:
        raise ConfigError(
            f"Missing config keys: {missing}"
        )

    motor_ids = tuple(int(mid) for mid in base["motor_ids"])
    joint_ids = tuple(str(joint) for joint in base["joint_ids"])

    raw_mapping = {
        str(joint): int(motor_id)
        for joint, motor_id
        in base["joint_to_motor_map"].items()
    }
    joint_to_motor_map = {
        joint: abs(motor_id)
        for joint, motor_id in raw_mapping.items()
    }

    reverse_joints = {
        str(joint)
        for joint in base.get("reverse_joints", [])
    }
    reverse_joints.update(
        joint
        for joint, motor_id in raw_mapping.items()
        if motor_id < 0
    )

    joint_roms: Dict[str, tuple[float, float]] = {}
    for joint, values in base[rom_key].items():
        if not isinstance(values, list) or len(values) != 2:
            raise ConfigError(
                f"Invalid ROM for joint '{joint}': {values}"
            )
        lo, hi = float(values[0]), float(values[1])
        if lo >= hi:
            raise ConfigError(
                f"Invalid ROM for joint '{joint}': {values}"
            )
        joint_roms[str(joint)] = (lo, hi)

    neutral_position = {
        str(joint): float(value)
        for joint, value in base["neutral_position"].items()
    }

    if set(joint_ids) != set(joint_to_motor_map):
        raise ConfigError(
            "joint_ids and joint_to_motor_map do not match"
        )
    if set(joint_ids) != set(joint_roms):
        raise ConfigError(
            "joint_ids and joint_roms do not match"
        )
    if set(joint_ids) != set(neutral_position):
        raise ConfigError(
            "joint_ids and neutral_position do not match"
        )
    if set(motor_ids) != set(joint_to_motor_map.values()):
        raise ConfigError(
            "motor_ids and joint_to_motor_map values do not match"
        )

    # 支持两种 limits 格式：
    #
    # 1. 关节名格式：
    # joints:
    #   index_mcp:
    #     raw_min: 10
    #     raw_max: 2942
    #
    # 2. 电机 ID 格式：
    # motor_limits:
    #   3: [10, 2942]
    named_section = calibration_data.get("joints")
    numeric_section = calibration_data.get("motor_limits")

    if named_section is not None and not isinstance(named_section, dict):
        raise ConfigError(
            "The 'joints' calibration section must be a mapping"
        )
    if numeric_section is not None and not isinstance(numeric_section, dict):
        raise ConfigError(
            "The 'motor_limits' calibration section must be a mapping"
        )
    if named_section is None and numeric_section is None:
        # 兼容旧的、直接以关节名作为根键的格式。
        named_section = calibration_data

    calibrations: Dict[str, JointCalibration] = {}
    for joint_name in joint_ids:
        motor_id = joint_to_motor_map[joint_name]
        raw_min = None
        raw_max = None

        if isinstance(named_section, dict):
            entry = named_section.get(joint_name)
            if entry is not None:
                if not isinstance(entry, dict):
                    raise ConfigError(
                        f"Invalid calibration entry for '{joint_name}'"
                    )
                raw_min = entry.get("raw_min")
                raw_max = entry.get("raw_max")

        if (
            raw_min is None
            and raw_max is None
            and isinstance(numeric_section, dict)
        ):
            # YAML 数字键通常会被解析为 int，同时兼容字符串键。
            limits = numeric_section.get(
                motor_id,
                numeric_section.get(str(motor_id)),
            )
            if limits is not None:
                if (
                    not isinstance(limits, (list, tuple))
                    or len(limits) != 2
                ):
                    raise ConfigError(
                        f"Invalid motor_limits entry for motor {motor_id}: "
                        f"{limits}"
                    )
                raw_min, raw_max = limits

        calibrations[joint_name] = JointCalibration(
            joint_name=joint_name,
            motor_id=motor_id,
            raw_min=(
                None if raw_min is None else int(raw_min)
            ),
            raw_max=(
                None if raw_max is None else int(raw_max)
            ),
        )

    return RuntimeConfig(
        config_path=resolved_config,
        calibration_path=resolved_calibration,
        port=str(base["port"]),
        baudrate=int(base["baudrate"]),
        motor_ids=motor_ids,
        joint_ids=joint_ids,
        joint_to_motor_map=joint_to_motor_map,
        motor_to_joint_map={
            motor_id: joint
            for joint, motor_id
            in joint_to_motor_map.items()
        },
        reverse_joints=frozenset(reverse_joints),
        joint_roms=joint_roms,
        neutral_position=neutral_position,
        calibrations=calibrations,
    )


GESTURE_ANGLES: Dict[str, Dict[str, float]] = {
    "hand_open": {
        "thumb_mcp": 0.0,
        "thumb_dip": 0.0,
        "index_mcp": 0.0,
        "index_pip": 0.0,
        "middle_mcp": 0.0,
        "middle_pip": 0.0,
        "ring_mcp": 0.0,
        "ring_pip": 0.0,
        "pinky_mcp": 0.0,
        "pinky_pip": 0.0,
    },
    "hand_close": {
        "thumb_mcp": 80.0,
        "thumb_dip": 60.0,
        "index_mcp": 85.0,
        "index_pip": 90.0,
        "middle_mcp": 85.0,
        "middle_pip": 90.0,
        "ring_mcp": 85.0,
        "ring_pip": 90.0,
        "pinky_mcp": 85.0,
        "pinky_pip": 90.0,
    },
    "pinch_grasp": {
        "thumb_mcp": 70.0,
        "thumb_dip": 50.0,
        "index_mcp": 80.0,
        "index_pip": 60.0,
    },
}
