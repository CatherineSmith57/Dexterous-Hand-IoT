"""
hand_interface.py --- 给视觉识别和上层业务使用的稳定接口

总体数据流：

视觉识别
    -> 17 个关节角 / 归一化弯曲度
    -> HandRuntime 映射、限位、限速
    -> 最新帧覆盖旧帧的实时调度器
    -> MultiMotorControl.set_positions_sync()
    -> COM5 单总线
    -> 17 个 STS3215

兼容原接口：
    from hand_interface import init, do_gesture, get_status, cleanup

新增实时接口：
    set_motor_positions(...)
    set_joint_angles(...)
    set_joint_normalized(...)
    update_from_vision(...)
    emergency_stop()
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import asdict, dataclass
from typing import Dict, Mapping, Optional

import realtime_hand_config as hand_config
from realtime_hand_config import JointSpec
from multi_motor_control import (
    MultiMotorControl,
    MultiMotorResult,
)


logger = logging.getLogger("hand_interface")


class HandInterfaceError(RuntimeError):
    pass


class CalibrationRequiredError(HandInterfaceError):
    pass


@dataclass
class RuntimeStatistics:
    submitted_frames: int = 0
    sent_frames: int = 0
    dropped_frames: int = 0
    failed_frames: int = 0
    last_submit_time: Optional[float] = None
    last_send_time: Optional[float] = None
    last_error: Optional[str] = None


class HandRuntime:
    """17 关节实时控制运行时。

    实时流采用 latest-wins：
    当视觉帧产生速度快于串口发送速度时，只保留最新一帧，
    不让旧姿态在队列中堆积造成明显延迟。
    """

    def __init__(
        self,
        port: str = hand_config.PORT,
        baudrate: int = hand_config.BAUDRATE,
        control_hz: float = hand_config.CONTROL_HZ,
    ):
        if control_hz <= 0:
            raise ValueError("control_hz must be > 0")

        self.port = port
        self.baudrate = baudrate
        self.control_hz = control_hz

        self._mc = MultiMotorControl(
            port=port,
            baudrate=baudrate,
        )

        self._lifecycle_lock = threading.RLock()
        self._command_lock = threading.RLock()
        self._pending_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._pending_event = threading.Event()
        self._stream_paused = threading.Event()

        self._worker: Optional[threading.Thread] = None
        self._pending_targets: Optional[Dict[int, int]] = None

        self._connected = False
        self._torque_enabled = False
        self._watchdog_torque_disabled = False

        self._last_commanded: Dict[int, int] = {}
        self._last_hardware_positions: Dict[int, int] = {}
        self._stats = RuntimeStatistics()

    # ── 生命周期 ────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._connected and self._mc.is_connected

    def start(self) -> None:
        """连接机械手并启动实时调度线程；不会自动移动。"""
        with self._lifecycle_lock:
            if self.connected:
                return

            if not self._mc.connect():
                raise ConnectionError(
                    f"Unable to connect to {self.port}"
                )

            self._connected = True

            positions, failures = self._mc.read_positions()
            self._last_hardware_positions = positions
            self._last_commanded = dict(positions)

            if failures:
                logger.warning(
                    "Initial position read failures: %s",
                    failures,
                )

            self._stop_event.clear()
            self._worker = threading.Thread(
                target=self._worker_loop,
                name="hand-realtime-scheduler",
                daemon=True,
            )
            self._worker.start()

    def cleanup(self) -> None:
        """停止调度、关闭扭矩并释放串口。"""
        with self._lifecycle_lock:
            self._stop_event.set()
            self._pending_event.set()

            if (
                self._worker is not None
                and self._worker.is_alive()
                and self._worker is not threading.current_thread()
            ):
                self._worker.join(timeout=2.0)

            self._worker = None

            if self._mc.is_connected:
                ok, failures = self._mc.emergency_stop()
                if not ok:
                    logger.error(
                        "Torque disable failures during cleanup: %s",
                        failures,
                    )
                self._mc.disconnect()

            self._connected = False
            self._torque_enabled = False

    # ── 配置、映射与安全约束 ────────────────────────────────────────

    @staticmethod
    def _spec_by_motor_id(motor_id: int) -> JointSpec:
        joint_name = hand_config.MOTOR_TO_JOINT.get(motor_id)
        if joint_name is None:
            raise ValueError(f"Unknown motor id: {motor_id}")
        return hand_config.JOINT_SPECS[joint_name]

    @staticmethod
    def _angle_to_raw(
        spec: JointSpec,
        angle_deg: float,
    ) -> int:
        if not spec.angle_calibrated:
            raise CalibrationRequiredError(
                f"Joint '{spec.name}' has no complete "
                "angle/raw calibration"
            )

        assert spec.angle_min_deg is not None
        assert spec.angle_max_deg is not None
        assert spec.raw_min is not None
        assert spec.raw_max is not None

        clipped_angle = min(
            spec.angle_max_deg,
            max(spec.angle_min_deg, float(angle_deg)),
        )
        ratio = (
            (clipped_angle - spec.angle_min_deg)
            / (spec.angle_max_deg - spec.angle_min_deg)
        )
        if spec.inverted:
            ratio = 1.0 - ratio

        return round(
            spec.raw_min
            + ratio * (spec.raw_max - spec.raw_min)
        )

    @staticmethod
    def _normalized_to_raw(
        spec: JointSpec,
        value: float,
    ) -> int:
        if not spec.raw_calibrated:
            raise CalibrationRequiredError(
                f"Joint '{spec.name}' has no raw soft limits"
            )

        assert spec.raw_min is not None
        assert spec.raw_max is not None

        ratio = min(1.0, max(0.0, float(value)))
        if spec.inverted:
            ratio = 1.0 - ratio

        return round(
            spec.raw_min
            + ratio * (spec.raw_max - spec.raw_min)
        )

    def _validate_and_clamp_raw(
        self,
        targets: Mapping[int, int],
        require_all: bool,
    ) -> Dict[int, int]:
        normalized = {
            int(motor_id): int(position)
            for motor_id, position in targets.items()
        }

        if require_all:
            expected = set(range(1, 18))
            actual = set(normalized)
            if actual != expected:
                missing = sorted(expected - actual)
                extra = sorted(actual - expected)
                raise ValueError(
                    f"Expected all 17 motor IDs. "
                    f"missing={missing}, extra={extra}"
                )

        safe: Dict[int, int] = {}
        for motor_id, position in normalized.items():
            spec = self._spec_by_motor_id(motor_id)
            if not spec.raw_calibrated:
                raise CalibrationRequiredError(
                    f"Joint '{spec.name}' / Motor {motor_id} "
                    "has no raw_min/raw_max. "
                    "Calibrate this joint in hand_config.py first."
                )

            assert spec.raw_min is not None
            assert spec.raw_max is not None
            safe[motor_id] = min(
                spec.raw_max,
                max(spec.raw_min, position),
            )

        return safe

    def _apply_rate_limit(
        self,
        targets: Mapping[int, int],
    ) -> Dict[int, int]:
        max_step = int(
            hand_config.MAX_RAW_STEP_PER_UPDATE
        )
        if max_step <= 0:
            return dict(targets)

        limited: Dict[int, int] = {}
        for motor_id, target in targets.items():
            previous = self._last_commanded.get(
                motor_id,
                self._last_hardware_positions.get(
                    motor_id,
                    target,
                ),
            )
            limited[motor_id] = max(
                previous - max_step,
                min(previous + max_step, target),
            )

        return limited

    def _ensure_torque(self, motor_ids) -> None:
        if (
            self._torque_enabled
            and not self._watchdog_torque_disabled
        ):
            return

        ok, failures = self._mc.set_torque_many(
            motor_ids,
            True,
        )
        if not ok:
            raise HandInterfaceError(
                f"Cannot enable torque: {failures}"
            )

        self._torque_enabled = True
        self._watchdog_torque_disabled = False

    # ── 实时最新帧调度 ──────────────────────────────────────────────

    def submit_raw(
        self,
        targets: Mapping[int, int],
        require_all: bool = False,
    ) -> int:
        """提交实时 raw 位置；立即返回，不等待电机完成。

        当旧帧还没有发送时，新帧会覆盖旧帧。
        """
        if not self.connected:
            raise HandInterfaceError("Hand is not initialized")

        safe_targets = self._validate_and_clamp_raw(
            targets,
            require_all=require_all,
        )
        safe_targets = self._apply_rate_limit(
            safe_targets
        )

        with self._pending_lock:
            if self._pending_targets is not None:
                self._stats.dropped_frames += 1

            self._pending_targets = dict(safe_targets)
            self._stats.submitted_frames += 1
            self._stats.last_submit_time = time.time()
            sequence = self._stats.submitted_frames

        self._pending_event.set()
        return sequence

    def _take_pending(self) -> Optional[Dict[int, int]]:
        with self._pending_lock:
            targets = self._pending_targets
            self._pending_targets = None
            self._pending_event.clear()
            return targets

    def _worker_loop(self) -> None:
        period = 1.0 / self.control_hz
        next_allowed_send = time.monotonic()

        while not self._stop_event.is_set():
            self._pending_event.wait(timeout=period)

            if self._stop_event.is_set():
                break

            if self._stream_paused.is_set():
                time.sleep(min(period, 0.02))
                continue

            now = time.monotonic()
            if now < next_allowed_send:
                time.sleep(next_allowed_send - now)

            targets = self._take_pending()
            if targets is not None:
                try:
                    with self._command_lock:
                        self._ensure_torque(targets.keys())
                        result = self._mc.set_positions_sync(
                            targets,
                            speed=hand_config.DEFAULT_SPEED,
                            acc=hand_config.DEFAULT_ACC,
                            torque=hand_config.DEFAULT_TORQUE,
                            wait=False,
                        )

                    if result.success:
                        self._last_commanded.update(
                            result.targets
                        )
                        self._stats.sent_frames += 1
                        self._stats.last_send_time = time.time()
                        self._stats.last_error = None
                    else:
                        self._stats.failed_frames += 1
                        self._stats.last_error = (
                            f"{result.message}: "
                            f"{result.failures}"
                        )
                        logger.error(
                            "Realtime write failed: %s",
                            self._stats.last_error,
                        )
                except Exception as exc:
                    self._stats.failed_frames += 1
                    self._stats.last_error = str(exc)
                    logger.exception(
                        "Realtime scheduler failure"
                    )

                next_allowed_send = (
                    time.monotonic() + period
                )
            else:
                self._run_watchdog_if_needed()

    def _run_watchdog_if_needed(self) -> None:
        timeout = float(
            hand_config.WATCHDOG_TIMEOUT_SEC
        )
        if timeout <= 0:
            return

        last_submit = self._stats.last_submit_time
        if last_submit is None:
            return

        if time.time() - last_submit < timeout:
            return

        if hand_config.WATCHDOG_ACTION == "hold":
            # 不发送新指令，舵机保持最后目标。
            return

        if (
            hand_config.WATCHDOG_ACTION
            == "disable_torque"
            and self._torque_enabled
            and not self._watchdog_torque_disabled
        ):
            with self._command_lock:
                ok, failures = self._mc.emergency_stop()
            if ok:
                self._watchdog_torque_disabled = True
            else:
                logger.error(
                    "Watchdog torque disable failed: %s",
                    failures,
                )

    # ── 角度、归一化值与阻塞动作 ────────────────────────────────────

    def submit_angles(
        self,
        joint_angles_deg: Mapping[str, float],
        require_all: bool = False,
    ) -> int:
        raw_targets = {
            hand_config.JOINT_SPECS[joint_name].motor_id:
                self._angle_to_raw(
                    hand_config.JOINT_SPECS[joint_name],
                    angle,
                )
            for joint_name, angle in joint_angles_deg.items()
        }

        if require_all and set(joint_angles_deg) != set(
            hand_config.JOINT_SPECS
        ):
            missing = sorted(
                set(hand_config.JOINT_SPECS)
                - set(joint_angles_deg)
            )
            raise ValueError(
                f"Missing joint angles: {missing}"
            )

        return self.submit_raw(
            raw_targets,
            require_all=require_all,
        )

    def submit_normalized(
        self,
        joint_values: Mapping[str, float],
        require_all: bool = False,
    ) -> int:
        raw_targets = {
            hand_config.JOINT_SPECS[joint_name].motor_id:
                self._normalized_to_raw(
                    hand_config.JOINT_SPECS[joint_name],
                    value,
                )
            for joint_name, value in joint_values.items()
        }

        if require_all and set(joint_values) != set(
            hand_config.JOINT_SPECS
        ):
            missing = sorted(
                set(hand_config.JOINT_SPECS)
                - set(joint_values)
            )
            raise ValueError(
                f"Missing normalized joints: {missing}"
            )

        return self.submit_raw(
            raw_targets,
            require_all=require_all,
        )

    def execute_raw(
        self,
        targets: Mapping[int, int],
        wait: bool = True,
        require_all: bool = False,
        speed: int = hand_config.DEFAULT_SPEED,
        acc: int = hand_config.DEFAULT_ACC,
        torque: int = hand_config.DEFAULT_TORQUE,
    ) -> MultiMotorResult:
        """执行一个确定姿态；执行期间暂停实时视觉流。"""
        if not self.connected:
            raise HandInterfaceError("Hand is not initialized")

        safe_targets = self._validate_and_clamp_raw(
            targets,
            require_all=require_all,
        )

        self._stream_paused.set()
        with self._pending_lock:
            self._pending_targets = None
            self._pending_event.clear()

        try:
            with self._command_lock:
                self._ensure_torque(safe_targets.keys())
                result = self._mc.set_positions_sync(
                    safe_targets,
                    speed=speed,
                    acc=acc,
                    torque=torque,
                    wait=wait,
                    timeout=hand_config.MOVE_TIMEOUT_SEC,
                    tolerance=hand_config.POSITION_TOLERANCE_RAW,
                )

            if result.success:
                self._last_commanded.update(
                    result.targets
                )
            else:
                self._stats.last_error = (
                    f"{result.message}: "
                    f"{result.failures}"
                )
            return result
        finally:
            self._stream_paused.clear()

    def do_gesture(
        self,
        name: str,
        hold_time_sec: float = 0.0,
        return_to_neutral: bool = False,
    ) -> MultiMotorResult:
        if name not in hand_config.GESTURES_RAW:
            raise KeyError(
                f"Unknown gesture '{name}'. "
                f"Available: "
                f"{sorted(hand_config.GESTURES_RAW)}"
            )

        targets = hand_config.GESTURES_RAW[name]
        if not targets:
            raise CalibrationRequiredError(
                f"Gesture '{name}' has no motor positions. "
                "Fill GESTURES_RAW in hand_config.py first."
            )

        result = self.execute_raw(
            targets,
            wait=True,
            require_all=False,
        )
        if not result.success:
            return result

        if hold_time_sec > 0:
            time.sleep(hold_time_sec)

        if return_to_neutral:
            neutral = hand_config.get_neutral_raw()
            if not neutral:
                raise CalibrationRequiredError(
                    "No neutral_raw values configured"
                )
            return self.execute_raw(
                neutral,
                wait=True,
                require_all=False,
            )

        return result

    # ── 状态、暂停与急停 ────────────────────────────────────────────

    def pause_streaming(self) -> None:
        self._stream_paused.set()

    def resume_streaming(self) -> None:
        self._stream_paused.clear()
        self._pending_event.set()

    def emergency_stop(self) -> Dict[str, object]:
        self._stream_paused.set()
        with self._pending_lock:
            self._pending_targets = None
            self._pending_event.clear()

        with self._command_lock:
            ok, failures = self._mc.emergency_stop()

        if ok:
            self._torque_enabled = False
            self._watchdog_torque_disabled = True

        return {
            "success": ok,
            "failures": failures,
        }

    def get_status(
        self,
        refresh_hardware: bool = False,
    ) -> Dict[str, object]:
        status: Dict[str, object] = {
            "connected": self.connected,
            "calibrated": (
                hand_config.full_calibration_complete()
            ),
            "raw_calibrated": (
                hand_config.raw_calibration_complete()
            ),
            "angle_calibrated": (
                hand_config.angle_calibration_complete()
            ),
            "streaming_paused": (
                self._stream_paused.is_set()
            ),
            "torque_enabled": self._torque_enabled,
            "control_hz": self.control_hz,
            "last_commanded_positions": dict(
                self._last_commanded
            ),
            "statistics": asdict(self._stats),
        }

        if refresh_hardware and self.connected:
            with self._command_lock:
                positions, position_failures = (
                    self._mc.read_positions()
                )
                currents, current_failures = (
                    self._mc.read_currents_raw()
                )

            self._last_hardware_positions = positions
            status.update(
                {
                    "motor_positions": positions,
                    "motor_position_failures":
                        position_failures,
                    "motor_currents_raw": currents,
                    "motor_current_failures":
                        current_failures,
                }
            )

        return status


_RUNTIME: Optional[HandRuntime] = None
_RUNTIME_LOCK = threading.RLock()


def _require_runtime() -> HandRuntime:
    if _RUNTIME is None or not _RUNTIME.connected:
        raise HandInterfaceError(
            "Call init() before controlling the hand"
        )
    return _RUNTIME


def init(
    port: Optional[str] = None,
    baudrate: Optional[int] = None,
    control_hz: Optional[float] = None,
) -> Dict[str, object]:
    """连接机械手并启动实时调度；不会自动移动或自动撞限位校准。"""
    global _RUNTIME

    with _RUNTIME_LOCK:
        if _RUNTIME is not None:
            _RUNTIME.cleanup()

        _RUNTIME = HandRuntime(
            port=port or hand_config.PORT,
            baudrate=baudrate or hand_config.BAUDRATE,
            control_hz=control_hz or hand_config.CONTROL_HZ,
        )
        _RUNTIME.start()
        return _RUNTIME.get_status(
            refresh_hardware=True
        )


def do_gesture(
    name: str,
    hold_time_sec: float = 0.0,
    return_to_neutral: bool = False,
) -> Dict[str, object]:
    result = _require_runtime().do_gesture(
        name,
        hold_time_sec=hold_time_sec,
        return_to_neutral=return_to_neutral,
    )
    return asdict(result)


def set_motor_positions(
    motor_positions: Mapping[int, int],
    realtime: bool = True,
    wait: bool = False,
    require_all: bool = False,
) -> object:
    """下发 motor_id -> raw_position。

    realtime=True：
        最新帧异步调度，适合视觉循环。
    realtime=False：
        立即执行确定姿态，适合标定结果和标准手势。
    """
    runtime = _require_runtime()

    if realtime:
        return runtime.submit_raw(
            motor_positions,
            require_all=require_all,
        )

    return asdict(
        runtime.execute_raw(
            motor_positions,
            wait=wait,
            require_all=require_all,
        )
    )


def set_joint_angles(
    joint_angles_deg: Mapping[str, float],
    require_all: bool = False,
) -> int:
    """提交视觉侧输出的关节角，单位 degree。"""
    return _require_runtime().submit_angles(
        joint_angles_deg,
        require_all=require_all,
    )


def set_joint_normalized(
    joint_values: Mapping[str, float],
    require_all: bool = False,
) -> int:
    """提交 0~1 归一化关节弯曲度。"""
    return _require_runtime().submit_normalized(
        joint_values,
        require_all=require_all,
    )


def update_from_vision(
    joint_angles_deg: Mapping[str, float],
    confidence: Optional[float] = None,
    min_confidence: float = 0.5,
    require_all: bool = True,
) -> Optional[int]:
    """视觉线程的推荐入口。

    低置信度帧直接丢弃；高置信度帧采用 latest-wins 调度。
    """
    if (
        confidence is not None
        and confidence < min_confidence
    ):
        return None

    return set_joint_angles(
        joint_angles_deg,
        require_all=require_all,
    )


def get_status(
    refresh_hardware: bool = False,
) -> Dict[str, object]:
    return _require_runtime().get_status(
        refresh_hardware=refresh_hardware
    )


def pause_streaming() -> None:
    _require_runtime().pause_streaming()


def resume_streaming() -> None:
    _require_runtime().resume_streaming()


def emergency_stop() -> Dict[str, object]:
    return _require_runtime().emergency_stop()


def cleanup() -> None:
    global _RUNTIME

    with _RUNTIME_LOCK:
        if _RUNTIME is not None:
            _RUNTIME.cleanup()
            _RUNTIME = None
