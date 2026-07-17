"""
hand_bridge.py — ORCA灵巧手中间适配层核心模块

职责：
1. 将ROS2 Service命令转换为orca_core底层库调用
2. 管理设备完整生命周期：连接、标定、执行、复位、使能/去使能、安全停止
3. 统一错误处理，按接口协议返回execution_result
4. 屏蔽orca_core异常细节，对上暴露标准字典接口
5. 扩展状态字段：温度、力矩、故障标记（orca_core未暴露的字段使用模拟占位）

设计原则：
- 单例模式管理OrcaHand实例（全局唯一设备连接）
- 所有方法返回协议兼容的字典格式
- 异常统一映射为error_code + error_message
- 分级日志：DEBUG(内部细节)/INFO(关键流程)/WARN(可恢复异常)/ERROR(需人工介入)
"""

import time
import random
import logging
from datetime import datetime, timezone, timedelta
from typing import Dict, List, Optional, Tuple

# ── 路径注入：向上搜索 third_party/orca_core/（嵌套包结构）─────
import os as _os, sys as _sys
_sd = _os.path.dirname(_os.path.abspath(__file__))
for _ in range(10):
    _c = _os.path.join(_sd, "third_party", "orca_core")
    if _os.path.isdir(_c):
        _pkg = _os.path.join(_c, "orca_core", "__init__.py")
        if _os.path.isfile(_pkg):
            if _c not in _sys.path:
                _sys.path.insert(0, _c)
            break
    _p = _os.path.dirname(_sd)
    if _p == _sd:
        break
    _sd = _p

from orca_core import OrcaHand, OrcaJointPositions
from orca_core.hand_config import OrcaHandConfig

logger = logging.getLogger(__name__)

# ── 北京时间时区 ───────────────────────────────────────────
_CST = timezone(timedelta(hours=8))

# ── 固定关节顺序（与 HandStatus.msg 索引一致）──────────────
JOINT_ORDER: List[str] = [
    "thumb_mcp", "thumb_pip",
    "index_mcp", "index_pip",
    "middle_mcp", "middle_pip",
    "ring_mcp", "ring_pip",
    "pinky_mcp", "pinky_pip",
]

# ── 手势映射表 ─────────────────────────────────────────────
GESTURE_MAPPING = {
    "open_palm": "hand_open",
    "fist": "hand_close",
    "pinch": "pinch_grasp",
    "two_finger": "two_finger_pose",
    "point": "point_pose",
}

# ── 手势对应的全关节目标值（来自 orca_hand.py _gesture_to_targets）────
HAND_OPEN_TARGETS: Dict[str, float] = {
    "thumb_mcp": 0, "thumb_pip": 0,
    "index_mcp": 0, "index_pip": 0,
    "middle_mcp": 0, "middle_pip": 0,
    "ring_mcp": 0, "ring_pip": 0,
    "pinky_mcp": 0, "pinky_pip": 0,
}

HAND_CLOSE_TARGETS: Dict[str, float] = {
    "thumb_mcp": 50, "thumb_pip": 80,
    "index_mcp": 80, "index_pip": 90,
    "middle_mcp": 80, "middle_pip": 90,
    "ring_mcp": 80, "ring_pip": 90,
    "pinky_mcp": 80, "pinky_pip": 90,
}



# ── 更多手势目标值（deg）───────────────────────────────
PINCH_GRASP_TARGETS: Dict[str, float] = {
    "thumb_mcp": 45.0, "thumb_pip": 70.0,
    "index_mcp": 60.0, "index_pip": 90.0,
    "middle_mcp": 0.0, "middle_pip": 0.0,
    "ring_mcp": 0.0, "ring_pip": 0.0,
    "pinky_mcp": 0.0, "pinky_pip": 0.0,
}

TWO_FINGER_POSE_TARGETS: Dict[str, float] = {
    "thumb_mcp": 0.0, "thumb_pip": 0.0,
    "index_mcp": 60.0, "index_pip": 90.0,
    "middle_mcp": 60.0, "middle_pip": 90.0,
    "ring_mcp": 0.0, "ring_pip": 0.0,
    "pinky_mcp": 0.0, "pinky_pip": 0.0,
}

POINT_POSE_TARGETS: Dict[str, float] = {
    "thumb_mcp": 0.0, "thumb_pip": 0.0,
    "index_mcp": 45.0, "index_pip": 90.0,
    "middle_mcp": 0.0, "middle_pip": 0.0,
    "ring_mcp": 0.0, "ring_pip": 0.0,
    "pinky_mcp": 0.0, "pinky_pip": 0.0,
}

# ── 内部常量（原由 orca_core.orca_hand 提供，现本地维护）────
_VALID_JOINTS = set(JOINT_ORDER)

JOINT_ROMS: Dict[str, tuple] = {
    "thumb_mcp": (0.0, 90.0), "thumb_pip": (0.0, 90.0),
    "index_mcp": (0.0, 90.0), "index_pip": (0.0, 90.0),
    "middle_mcp": (0.0, 90.0), "middle_pip": (0.0, 90.0),
    "ring_mcp": (0.0, 90.0), "ring_pip": (0.0, 90.0),
    "pinky_mcp": (0.0, 90.0), "pinky_pip": (0.0, 90.0),
}

_GESTURE_TARGETS: Dict[str, Dict[str, float]] = {
    "hand_open": HAND_OPEN_TARGETS,
    "hand_close": HAND_CLOSE_TARGETS,
    "pinch_grasp": PINCH_GRASP_TARGETS,
    "two_finger_pose": TWO_FINGER_POSE_TARGETS,
    "point_pose": POINT_POSE_TARGETS,
}

_VALID_GESTURES = set(_GESTURE_TARGETS.keys())

class HandBridge:
    """
    ORCA灵巧手中间适配层（增强版）。

    对上暴露标准协议接口，对下封装orca_core调用。
    在原 OrcaBridge 基础上新增：
    - 使能/去使能电机
    - 硬件复位
    - 开合幅度插值
    - 温度/力矩/故障标记（扩展字段，当前使用模拟值占位）
    - 分级异常日志

    Parameters
    ----------
    port : str
        串口号, Linux默认为 ``/dev/ttyUSB0``, Windows为 ``COM3``
    baudrate : int
        波特率, 默认115200
    temperature_limit : float
        温度上限 (°C), 超出触发 fault_code=1
    torque_limit : float
        力矩上限 (N·m), 超出触发 fault_code=2
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        temperature_limit: float = 75.0,
        torque_limit: float = 2.5,
    ):
        self._port = port
        self._baudrate = baudrate
        self._temperature_limit = temperature_limit
        self._torque_limit = torque_limit
        self._hand: Optional[OrcaHand] = None
        self._initialized = False
        self._motor_enabled = False  # 电机使能状态

        # ── 模拟温度/力矩数据（接入真实硬件后替换为编码器/电流读取）──
        # 每个关节的"基线"温度与力矩，叠加小幅随机波动模拟真实传感器
        self._sim_temperatures: Dict[str, float] = {
            joint: 35.0 + random.uniform(-2.0, 5.0) for joint in JOINT_ORDER
        }
        self._sim_torques: Dict[str, float] = {
            joint: 0.05 + random.uniform(-0.03, 0.08) for joint in JOINT_ORDER
        }

        logger.info(
            f"HandBridge created (port={port}, baudrate={baudrate}, "
            f"temp_limit={temperature_limit}°C, torque_limit={torque_limit}N·m)"
        )

    # ── 生命周期管理 ────────────────────────────────────────

    def initialize(self) -> bool:
        """
        初始化设备：创建OrcaHand实例 → 连接 → 标定 → 使能电机。

        此方法应在ROS2节点启动时调用一次。

        Returns
        -------
        bool
            初始化是否全部成功

        Raises
        ------
        OrcaSerialError
            串口连接失败时抛出，由上层节点捕获
        """
        try:
            logger.info(f"Initializing ORCA hand on port {self._port}...")
            self._hand = OrcaHand(config=OrcaHandConfig(port=self._port, baudrate=self._baudrate))
            logger.debug("OrcaHand instance created")

            self._hand.connect()
            logger.info(f"Serial connection established on {self._port}")

            self._hand.calibrate()
            logger.info("Device calibration complete")

            # 标定完成后使能电机
            self._motor_enabled = True
            self._initialized = True
            logger.info("HandBridge initialization complete — device ready")
            return True

        except IOError as e:
            logger.error(f"[FATAL] Serial connection failed on {self._port}: {e}")
            self._initialized = False
            self._motor_enabled = False
            raise

        except Exception as e:
            logger.error(f"[FATAL] ORCA initialization error (code={e.error_code}): {e}")
            self._initialized = False
            self._motor_enabled = False
            return False

        except Exception as e:
            logger.exception(f"[FATAL] Unexpected error during initialization: {e}")
            self._initialized = False
            self._motor_enabled = False
            return False

    def shutdown(self) -> None:
        """安全关闭：去使能电机 → 断开设备连接。"""
        logger.info("Shutting down HandBridge...")
        if self._motor_enabled:
            self.disable_motor()
        if self._hand is not None:
            try:
                self._hand.disconnect()
                logger.info("Device disconnected")
            except Exception as e:
                logger.warning(f"Non-critical error during disconnect: {e}")
        self._initialized = False
        logger.info("HandBridge shutdown complete")

    # ── 状态查询 ────────────────────────────────────────────

    def is_ready(self) -> bool:
        """设备是否就绪（已连接+已标定+电机使能）。"""
        if self._hand is None:
            return False
        return (
            self._hand.connected
            and self._hand.calibrated
            and self._motor_enabled
        )

    def get_device_status(self) -> Dict:
        """
        获取设备完整状态（扩展版）。

        除 orca_core 原有字段外，额外返回温度、力矩、故障标记。
        温度/力矩当前使用模拟值，接入真实硬件后替换读取逻辑。

        Returns
        -------
        dict
            包含:
            - connected, calibrated, motor_enabled
            - joint_names, joint_positions
            - joint_temperatures, joint_torques
            - fault_code, fault_message
            - current_action, execution_status
        """
        # 默认空状态（设备未创建时返回）
        if self._hand is None:
            return {
                "connected": False,
                "calibrated": False,
                "motor_enabled": False,
                "joint_names": list(JOINT_ORDER),
                "joint_positions": [0.0] * 10,
                "joint_temperatures": [0.0] * 10,
                "joint_torques": [0.0] * 10,
                "fault_code": 1001,  # 未连接
                "fault_message": "Device not initialized — OrcaHand is None",
                "current_action": "idle",
                "execution_status": "idle",
            }

        # ── 从orca_core读取真实状态 ──────────────────────────
        try:
            core_status = self._hand.get_status()
        except Exception as e:
            logger.error(f"Failed to read device status from orca_core: {e}")
            # 读取失败时返回上一次已知状态并标记故障
            return self._build_fallback_status(str(e))

        # ── 更新模拟传感器数据（随机微调模拟真实波动）─────────
        self._update_sim_sensors()

        # ── 组装扩展状态字典 ─────────────────────────────────
        joint_names = list(JOINT_ORDER)
        joint_positions = [
            core_status.joint_positions.get(j, 0.0) for j in JOINT_ORDER
        ]
        joint_temperatures = [
            round(self._sim_temperatures.get(j, 0.0), 1) for j in JOINT_ORDER
        ]
        joint_torques = [
            round(self._sim_torques.get(j, 0.0), 3) for j in JOINT_ORDER
        ]

        # ── 故障检测 ─────────────────────────────────────────
        fault_code, fault_message = self._detect_fault(
            core_status=core_status,
            temperatures=joint_temperatures,
            torques=joint_torques,
        )

        return {
            "connected": core_status.connected,
            "calibrated": core_status.calibrated,
            "motor_enabled": self._motor_enabled,
            "joint_names": joint_names,
            "joint_positions": joint_positions,
            "joint_temperatures": joint_temperatures,
            "joint_torques": joint_torques,
            "fault_code": fault_code,
            "fault_message": fault_message,
            "current_action": core_status.current_action,
            "execution_status": core_status.execution_status,
        }

    def _build_fallback_status(self, error_reason: str) -> Dict:
        """构建读取出错时的降级状态字典。"""
        return {
            "connected": self._hand.connected if self._hand else False,
            "calibrated": self._hand.calibrated if self._hand else False,
            "motor_enabled": self._motor_enabled,
            "joint_names": list(JOINT_ORDER),
            "joint_positions": [0.0] * 10,
            "joint_temperatures": [0.0] * 10,
            "joint_torques": [0.0] * 10,
            "fault_code": 1006,  # 读取超时
            "fault_message": f"Status read error: {error_reason}",
            "current_action": "idle",
            "execution_status": "idle",
        }

    def _update_sim_sensors(self) -> None:
        """
        更新模拟温度/力矩传感器数据。

        在真实硬件环境中，此方法应替换为：
        - 编码器温度读取 (SPI/I2C)
        - 电机电流 → 力矩估算
        """
        for joint in JOINT_ORDER:
            # 温度：基线 + 小幅随机漂移
            self._sim_temperatures[joint] += random.uniform(-0.3, 0.3)
            self._sim_temperatures[joint] = max(
                20.0, min(90.0, self._sim_temperatures[joint])
            )
            # 力矩：基线 + 小幅随机波动（有负载时偏高）
            self._sim_torques[joint] += random.uniform(-0.02, 0.02)
            self._sim_torques[joint] = max(
                0.0, min(5.0, self._sim_torques[joint])
            )

    def _detect_fault(
        self,
        core_status,
        temperatures: List[float],
        torques: List[float],
    ) -> Tuple[int, str]:
        """
        综合故障检测。

        检查项（按优先级）：
        1. 设备未连接 → 1001
        2. 设备未标定 → 1002
        3. 任一关节过温 → 1
        4. 任一关节过力矩 → 2
        5. 执行失败 → 根据 execution_status 判断

        Returns
        -------
        (fault_code, fault_message)
            fault_code=0 表示正常
        """
        # 连接性检查
        if not core_status.connected:
            return (1001, "Device not connected — check serial cable")
        if not core_status.calibrated:
            return (1002, "Device not calibrated — run calibration first")
        if not self._motor_enabled:
            return (1008, "Motor disabled — send 'enable' command first")

        # 过温检查
        for i, joint in enumerate(JOINT_ORDER):
            if temperatures[i] > self._temperature_limit:
                msg = (
                    f"Joint '{joint}' over-temperature: "
                    f"{temperatures[i]:.1f}°C > {self._temperature_limit}°C limit"
                )
                logger.warning(f"[FAULT] {msg}")
                return (1, msg)

        # 过力矩检查
        for i, joint in enumerate(JOINT_ORDER):
            if torques[i] > self._torque_limit:
                msg = (
                    f"Joint '{joint}' over-torque: "
                    f"{torques[i]:.3f}N·m > {self._torque_limit}N·m limit"
                )
                logger.warning(f"[FAULT] {msg}")
                return (2, msg)

        # 执行状态检查
        if core_status.execution_status == "failed":
            return (1006, "Last action execution failed")
        if core_status.execution_status == "aborted":
            return (1008, "Last action was aborted (emergency stop)")

        return (0, "")

    # ── 统一指令执行入口 ────────────────────────────────────

    def execute_command(
        self,
        command_type: str,
        gesture_name: str = "",
        amplitude: float = 1.0,
        joint_names: Optional[List[str]] = None,
        joint_targets: Optional[List[float]] = None,
        hold_time_sec: float = 2.0,
        return_to_neutral: bool = True,
    ) -> Dict:
        """
        统一指令执行入口。

        根据 command_type 分发到对应的处理方法。

        Parameters
        ----------
        command_type : str
            "joint_control" | "gesture" | "reset" | "enable" | "disable"
        gesture_name : str
            预定义手势名
        amplitude : float
            开合幅度 0.0(全开) ~ 1.0(全闭)
        joint_names : list or None
            目标关节名称列表
        joint_targets : list or None
            目标关节角度列表
        hold_time_sec : float
            动作保持时间 (秒)
        return_to_neutral : bool
            动作后是否回中

        Returns
        -------
        dict
            标准 execution_result 字典
        """
        logger.info(
            f"Execute command: type={command_type}, gesture={gesture_name}, "
            f"amplitude={amplitude}, hold={hold_time_sec}s, "
            f"return_neutral={return_to_neutral}"
        )

        if command_type == "joint_control":
            return self._execute_joint_control(
                joint_names=joint_names or [],
                joint_targets=joint_targets or [],
                hold_time_sec=hold_time_sec,
                return_to_neutral=return_to_neutral,
            )

        elif command_type == "gesture":
            return self._execute_gesture_with_amplitude(
                gesture_name=gesture_name,
                amplitude=amplitude,
                hold_time_sec=hold_time_sec,
                return_to_neutral=return_to_neutral,
            )

        elif command_type == "reset":
            return self.reset_hardware()

        elif command_type == "enable":
            return self.enable_motor()

        elif command_type == "disable":
            return self.disable_motor()

        else:
            logger.error(f"Unknown command_type: '{command_type}'")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=1003,
                error_message=(
                    f"Unknown command_type '{command_type}'. "
                    f"Valid: joint_control, gesture, reset, enable, disable"
                ),
            )

    # ── 关节精细控制 ────────────────────────────────────────

    def _execute_joint_control(
        self,
        joint_names: List[str],
        joint_targets: List[float],
        hold_time_sec: float,
        return_to_neutral: bool,
    ) -> Dict:
        """
        执行精细关节角度控制。

        将 ROS Service 的数组参数转换为 orca_core 的 Dict[str, float] 格式。

        Parameters
        ----------
        joint_names : list[str]
            目标关节名称列表（空串表示跳过该索引）
        joint_targets : list[float]
            目标角度列表，与 joint_names 一一对应

        Returns
        -------
        dict
            标准 execution_result
        """
        # ── 前置校验：设备就绪 ─────────────────────────────────
        if self._hand is None:
            logger.error("Joint control failed: hand is None")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=1001,
                error_message="Device not initialized — call initialize() first",
            )
        if not self._hand.connected:
            logger.error("Joint control failed: not connected")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=1001,
                error_message="Device not connected",
            )
        if not self._hand.calibrated:
            logger.error("Joint control failed: not calibrated")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=1002,
                error_message="Device not calibrated",
            )

        # ── 参数校验：数组长度 ─────────────────────────────────
        if len(joint_names) != len(joint_targets):
            msg = (
                f"joint_names length ({len(joint_names)}) "
                f"!= joint_targets length ({len(joint_targets)})"
            )
            logger.error(f"Parameter mismatch: {msg}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=1004,
                error_message=msg,
            )

        # ── 组装关节目标字典（过滤空名称）──────────────────────
        joint_dict: Dict[str, float] = {}
        for name, angle in zip(joint_names, joint_targets):
            name = name.strip()
            if not name:
                continue  # 跳过空串（未使用的数组槽位）
            if abs(angle) < 0.001:
                # 角度为0也接受（回中位）
                pass
            joint_dict[name] = float(angle)

        if not joint_dict:
            logger.warning("Joint control: no valid joint targets after filtering")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=1004,
                error_message="No valid joint targets provided (all names empty)",
            )

        logger.debug(f"Parsed joint targets: {joint_dict}")

        # ── 参数越界检查（ROM校验）─────────────────────────────
        # JOINT_ROMS/_VALID_JOINTS from orca_core config

        for joint, angle in joint_dict.items():
            if joint not in _VALID_JOINTS:
                msg = f"Unknown joint '{joint}'. Valid: {sorted(_VALID_JOINTS)}"
                logger.error(f"Invalid joint: {msg}")
                return self._make_result(
                    success=False,
                    execution_status="failed",
                    error_code=1004,
                    error_message=msg,
                )
            lo, hi = JOINT_ROMS[joint]
            if angle < lo or angle > hi:
                msg = f"Joint '{joint}' angle {angle}° out of ROM [{lo}, {hi}]"
                logger.error(f"ROM violation: {msg}")
                return self._make_result(
                    success=False,
                    execution_status="failed",
                    error_code=1005,
                    error_message=msg,
                )

        # ── 调用orca_core底层执行 ─────────────────────────────
        try:
            logger.info(f"Executing joint control: {joint_dict}")
            self._hand.execute_joint_targets(
                joint_targets=joint_dict,
                hold_time_sec=hold_time_sec,
                return_to_neutral=return_to_neutral,
            )
            logger.info("Joint control completed successfully")
            return self._make_result(
                success=True,
                execution_status="completed",
            )

        except ValueError as e:
            logger.error(f"Joint ROM error (code={e.error_code}): {e}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=e.error_code,
                error_message=str(e),
            )
        except ValueError as e:
            logger.error(f"Invalid joint error (code={e.error_code}): {e}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=e.error_code,
                error_message=str(e),
            )
        except TimeoutError as e:
            logger.error(f"Timeout error (code={e.error_code}): {e}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=e.error_code,
                error_message=str(e),
            )
        except IOError as e:
            logger.error(f"Serial error (code={e.error_code}): {e}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=e.error_code,
                error_message=f"Serial communication error: {e}",
            )
        except RuntimeError as e:
            logger.error(f"Not connected (code={e.error_code}): {e}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=e.error_code,
                error_message=str(e),
            )
        except RuntimeError as e:
            logger.error(f"Not calibrated (code={e.error_code}): {e}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=e.error_code,
                error_message=str(e),
            )
        except RuntimeError as e:
            logger.error(f"Safety error (code={e.error_code}): {e}")
            return self._make_result(
                success=False,
                execution_status="aborted",
                error_code=e.error_code,
                error_message=str(e),
            )
        except Exception as e:
            logger.exception(f"Unexpected error during joint control: {e}")
            return self._make_result(
                success=False,
                execution_status="failed",
                error_code=1006,
                error_message=f"Unexpected error: {e}",
            )

    # ── 手势控制（带开合幅度插值）────────────────────────────

    def _execute_gesture_with_amplitude(
        self,
        gesture_name: str,
        amplitude: float,
        hold_time_sec: float,
        return_to_neutral: bool,
    ) -> Dict:
        """
        执行手势动作，支持开合幅度插值。

        当 amplitude 在 (0.0, 1.0) 之间时，在 hand_open (全开) 和
        目标手势（全闭）之间线性插值，实现精细的开合控制。

        Parameters
        ----------
        gesture_name : str
            预定义手势名
        amplitude : float
            0.0 = 全开(hand_open), 1.0 = 全闭(目标手势)
        """
        # ── 前置校验 ─────────────────────────────────────────
        if self._hand is None:
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1001, error_message="Device not initialized",
            )

        # ── 参数校验 ─────────────────────────────────────────
        # _VALID_GESTURES from local mapping

        # 先将上层手势标签映射为orca_core内部名
        internal_name = GESTURE_MAPPING.get(gesture_name, gesture_name)

        if internal_name not in _VALID_GESTURES:
            valid_list = sorted(_VALID_GESTURES)
            msg = (
                f"Unknown gesture '{gesture_name}' (internal: '{internal_name}'). "
                f"Valid: {valid_list}"
            )
            logger.error(f"Invalid gesture: {msg}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1003, error_message=msg,
            )

        # 幅度越界检查
        if amplitude < 0.0 or amplitude > 1.0:
            msg = f"Amplitude {amplitude} out of range [0.0, 1.0]"
            logger.error(f"Parameter out of bounds: {msg}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1005, error_message=msg,
            )

        # ── 幅度插值：amplitude=0 → hand_open, amplitude=1 → 目标手势 ─
        try:
            # 获取目标手势的关节角度
            target_angles = self._get_gesture_joint_targets(internal_name)
            open_angles = dict(HAND_OPEN_TARGETS)

            # 线性插值：angle = open + amplitude * (target - open)
            interpolated = {}
            for joint in JOINT_ORDER:
                open_angle = open_angles.get(joint, 0.0)
                target_angle = target_angles.get(joint, 0.0)
                interpolated[joint] = open_angle + amplitude * (target_angle - open_angle)

            # 预计算前3个关节的插值角度字符串（避免f-string嵌套兼容性问题）
            _sample = ", ".join(
                f"{j}={interpolated[j]:.1f}" for j in list(interpolated.keys())[:3]
            )
            logger.debug(
                f"Gesture '{gesture_name}' amplitude={amplitude:.2f} -> "
                f"interpolated angles: {{{_sample}}}..."
            )

            # 通过关节控制接口执行插值后的目标
            self._hand.execute_joint_targets(
                joint_targets=interpolated,
                hold_time_sec=hold_time_sec,
                return_to_neutral=return_to_neutral,
            )

            logger.info(
                f"Gesture '{gesture_name}' (amplitude={amplitude:.2f}) completed"
            )
            return self._make_result(success=True, execution_status="completed")

        except ValueError as e:
            logger.error(f"Invalid gesture (code={e.error_code}): {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=e.error_code, error_message=str(e),
            )
        except RuntimeError as e:
            logger.error(f"Not connected (code={e.error_code}): {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=e.error_code, error_message=str(e),
            )
        except RuntimeError as e:
            logger.error(f"Not calibrated (code={e.error_code}): {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=e.error_code, error_message=str(e),
            )
        except IOError as e:
            logger.error(f"Serial error (code={e.error_code}): {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=e.error_code, error_message=f"Serial error: {e}",
            )
        except Exception as e:
            logger.exception(f"Unexpected error during gesture execution: {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1006, error_message=f"Unexpected error: {e}",
            )

    @staticmethod
    def _get_gesture_joint_targets(gesture_name: str) -> Dict[str, float]:
        """
        获取预定义手势对应的全关节目标角度。

        复用 orca_core 中的 _gesture_to_targets 静态映射表。
        """
        # _OH: local gesture table used instead
        return _GESTURE_TARGETS.get(gesture_name)

    # ── 硬件复位 ────────────────────────────────────────────

    def reset_hardware(self) -> Dict:
        """
        硬件复位：先紧急停止 → 断开 → 重新连接 → 重新标定 → 使能。

        用于从异常状态中恢复设备。
        """
        logger.info("Hardware reset requested")

        if self._hand is None:
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1001, error_message="Device not initialized",
            )

        try:
            # 1. 紧急停止当前动作
            logger.debug("Step 1/4: Emergency stop")
            self._hand.stop_task()
            time.sleep(0.2)

            # 2. 断开连接
            logger.debug("Step 2/4: Disconnect")
            self._hand.disconnect()
            time.sleep(0.3)

            # 3. 重新连接
            logger.debug("Step 3/4: Reconnect")
            self._hand.connect()

            # 4. 重新标定
            logger.debug("Step 4/4: Recalibrate")
            self._hand.calibrate()

            # 恢复使能
            self._motor_enabled = True

            logger.info("Hardware reset complete — device re-initialized")
            return self._make_result(success=True, execution_status="completed")

        except IOError as e:
            logger.error(f"Reset failed: serial error (code={e.error_code}): {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=e.error_code,
                error_message=f"Serial error during reset: {e}",
            )
        except Exception as e:
            logger.error(f"Reset failed: ORCA error (code={e.error_code}): {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=e.error_code, error_message=str(e),
            )
        except Exception as e:
            logger.exception(f"Reset failed: unexpected error: {e}")
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1006, error_message=f"Unexpected error during reset: {e}",
            )

    # ── 电机使能/去使能 ─────────────────────────────────────

    def enable_motor(self) -> Dict:
        """
        使能电机（上电）。

        设备已连接+标定后调用，解除电机锁定。
        """
        logger.info("Motor enable requested")

        if self._hand is None:
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1001, error_message="Device not initialized",
            )
        if not self._hand.connected:
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1001, error_message="Device not connected",
            )
        if not self._hand.calibrated:
            return self._make_result(
                success=False, execution_status="failed",
                error_code=1002,
                error_message="Device not calibrated — cannot enable motors",
            )

        self._motor_enabled = True
        logger.info("Motor enabled")
        return self._make_result(success=True, execution_status="completed")

    def disable_motor(self) -> Dict:
        """
        去使能电机（下电，安全锁定）。

        先紧急停止当前动作，再锁定电机。
        """
        logger.info("Motor disable requested")

        if self._hand is not None:
            try:
                self._hand.stop_task()
                logger.debug("Emergency stop executed before motor disable")
            except Exception as e:
                logger.warning(f"Non-critical error during pre-disable stop: {e}")

        self._motor_enabled = False
        logger.info("Motor disabled — safe lock engaged")
        return self._make_result(success=True, execution_status="completed")

    # ── 紧急停止 ────────────────────────────────────────────

    def emergency_stop(self) -> Dict:
        """紧急停止所有电机。"""
        logger.warning("EMERGENCY STOP triggered!")

        if self._hand is not None:
            try:
                self._hand.stop_task()
                self._motor_enabled = False
                logger.warning("Emergency stop executed — all motors halted")
                return self._make_result(
                    success=True, execution_status="aborted",
                    error_code=0, error_message="Emergency stop activated",
                )
            except Exception as e:
                logger.error(f"Emergency stop error: {e}")
                return self._make_result(
                    success=False, execution_status="aborted",
                    error_code=1008, error_message=f"Emergency stop failed: {e}",
                )
        else:
            return self._make_result(
                success=False, execution_status="aborted",
                error_code=1001,
                error_message="Emergency stop: device not initialized",
            )

    # ── 内部工具方法 ────────────────────────────────────────

    def _make_result(
        self,
        success: bool,
        execution_status: str,
        error_code: int = 0,
        error_message: str = "",
    ) -> Dict:
        """构建标准 execution_result 字典。"""
        connected = self._hand.connected if self._hand else False
        calibrated = self._hand.calibrated if self._hand else False
        return {
            "success": success,
            "execution_status": execution_status,
            "connected": connected,
            "calibrated": calibrated,
            "error_code": error_code,
            "error_message": error_message,
            "timestamp": datetime.now(_CST).isoformat(),
        }
