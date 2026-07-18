"""
hand_interface.py --- 上层统一接口与实时视觉调度

兼容固定接口：
    init()
    do_gesture(...)
    get_status()
    cleanup()

额外提供：
    do_joint_command(...)
    update_from_vision(...)
    emergency_stop()

本文件不创建 OrcaHand，也不运行旧的自动撞限位校准。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Dict, Mapping, Optional

try:
    from .multi_motor_control import (
        MultiMotorControl,
        MultiMotorResult,
    )
    from .runtime_config import (
        ConfigError,
        GESTURE_ANGLES,
        RuntimeConfig,
        load_runtime_config,
    )
except ImportError:
    from multi_motor_control import (
        MultiMotorControl,
        MultiMotorResult,
    )
    from runtime_config import (
        ConfigError,
        GESTURE_ANGLES,
        RuntimeConfig,
        load_runtime_config,
    )


logger = logging.getLogger("hand_interface")

ERROR_NOT_CONNECTED = 1001
ERROR_NOT_CALIBRATED = 1002
ERROR_INVALID_COMMAND = 1003
ERROR_EXECUTION = 1007

DEFAULT_SPEED = 20
DEFAULT_ACC = 10
DEFAULT_TORQUE = 100
DEFAULT_CONTROL_HZ = 20.0

# 运行参数。可由 configure_motion() 在 init() 前或运行中修改。
_MOTION_SPEED = DEFAULT_SPEED
_MOTION_ACC = DEFAULT_ACC
_MOTION_TORQUE = DEFAULT_TORQUE
_CONTROL_HZ = DEFAULT_CONTROL_HZ


@dataclass
class _StreamStats:
    submitted: int = 0
    sent: int = 0
    dropped: int = 0
    failed: int = 0


class _LatestFrameScheduler:
    """视觉侧帧率高于总线时，仅保留最新的一帧。"""

    def __init__(
        self,
        controller: MultiMotorControl,
        bus_lock: threading.RLock,
        control_hz: float = DEFAULT_CONTROL_HZ,
    ):
        if control_hz <= 0:
            raise ValueError("control_hz must be > 0")

        self._controller = controller
        self._bus_lock = bus_lock
        self._period = 1.0 / control_hz

        self._pending_lock = threading.Lock()
        self._pending: Optional[Dict[int, int]] = None

        self._event = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.stats = _StreamStats()
        self.last_error = ""

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="hand-latest-frame-scheduler",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._event.set()

        if (
            self._thread is not None
            and self._thread.is_alive()
            and self._thread is not threading.current_thread()
        ):
            self._thread.join(timeout=2.0)

        self._thread = None

    def submit(self, targets: Mapping[int, int]) -> int:
        with self._pending_lock:
            if self._pending is not None:
                self.stats.dropped += 1

            self._pending = {
                int(mid): int(pos)
                for mid, pos in targets.items()
            }
            self.stats.submitted += 1
            sequence = self.stats.submitted

        self._event.set()
        return sequence

    def clear(self) -> None:
        with self._pending_lock:
            self._pending = None
            self._event.clear()

    def _take(self) -> Optional[Dict[int, int]]:
        with self._pending_lock:
            command = self._pending
            self._pending = None
            self._event.clear()
            return command

    def _run(self) -> None:
        next_send = time.monotonic()

        while not self._stop.is_set():
            self._event.wait(timeout=self._period)

            if self._stop.is_set():
                break

            now = time.monotonic()
            if now < next_send:
                time.sleep(next_send - now)

            targets = self._take()
            if targets is None:
                continue

            try:
                with self._bus_lock:
                    torque_ok, torque_failures = (
                        self._controller.set_torque_many(
                            targets.keys(),
                            True,
                        )
                    )
                    if not torque_ok:
                        raise RuntimeError(
                            "torque_enable_failed: "
                            f"{torque_failures}"
                        )

                    result = self._controller.set_positions_sync(
                        targets,
                        speed=_MOTION_SPEED,
                        acc=_MOTION_ACC,
                        torque=_MOTION_TORQUE,
                        wait=False,
                    )

                if result.success:
                    self.stats.sent += 1
                    self.last_error = ""
                else:
                    self.stats.failed += 1
                    self.last_error = (
                        f"{result.message}: {result.failures}"
                    )
            except Exception as exc:
                self.stats.failed += 1
                self.last_error = str(exc)
                logger.exception("Realtime command failed")

            next_send = time.monotonic() + self._period


hand: Optional[MultiMotorControl] = None
_config: Optional[RuntimeConfig] = None
_scheduler: Optional[_LatestFrameScheduler] = None

_connected = False
_calibrated = False
_last_error = ""

_state_lock = threading.RLock()
_bus_lock = threading.RLock()


def _failed(
    error_code: int,
    message: str,
) -> dict:
    return {
        "success": False,
        "execution_status": "failed",
        "error_code": error_code,
        "error_message": message,
    }


def _execute_angles(
    joint_positions: Mapping[str, float],
    wait: bool = True,
) -> MultiMotorResult:
    if hand is None or _config is None:
        raise RuntimeError("Hand is not initialized")

    raw_targets = _config.angles_to_raw(joint_positions)

    with _bus_lock:
        torque_ok, torque_failures = hand.set_torque_many(
            raw_targets.keys(),
            True,
        )
        if not torque_ok:
            return MultiMotorResult(
                success=False,
                message="torque_enable_failed",
                targets=raw_targets,
                failures=torque_failures,
            )

        return hand.set_positions_sync(
            raw_targets,
            speed=_MOTION_SPEED,
            acc=_MOTION_ACC,
            torque=_MOTION_TORQUE,
            wait=wait,
            timeout=50.0,
            tolerance=30,
        )



def configure_motion(
    speed: int | None = None,
    acc: int | None = None,
    torque: int | None = None,
    control_hz: float | None = None,
) -> dict:
    """配置实时运动参数。

    speed:
        Feetech WritePosEx 的速度字段。值越大通常运动越快，
        但这里不假定它等于某个固定 deg/s。
    acc:
        加速度字段，范围 0~254。
    torque:
        WritePosEx 的扭矩限制字段，范围 0~1000。
    control_hz:
        视觉目标下发频率，不是电机物理转速。
        已连接后不能改变；需要重新 init。
    """
    global _MOTION_SPEED
    global _MOTION_ACC
    global _MOTION_TORQUE
    global _CONTROL_HZ

    if speed is not None:
        value = int(speed)
        if value <= 0:
            raise ValueError("speed must be > 0")
        _MOTION_SPEED = value

    if acc is not None:
        value = int(acc)
        if not 0 <= value <= 254:
            raise ValueError("acc must be in [0, 254]")
        _MOTION_ACC = value

    if torque is not None:
        value = int(torque)
        if not 0 <= value <= 1000:
            raise ValueError("torque must be in [0, 1000]")
        _MOTION_TORQUE = value

    if control_hz is not None:
        value = float(control_hz)
        if not 1.0 <= value <= 100.0:
            raise ValueError("control_hz must be in [1, 100]")
        if _scheduler is not None:
            raise RuntimeError(
                "control_hz cannot change while scheduler is running; "
                "cleanup() and init() again"
            )
        _CONTROL_HZ = value

    return {
        "speed": _MOTION_SPEED,
        "acc": _MOTION_ACC,
        "torque": _MOTION_TORQUE,
        "control_hz": _CONTROL_HZ,
    }


def pause_realtime() -> dict:
    """停止接收队列中的旧视觉帧，但保持当前电机目标和扭矩。"""
    if hand is None or not _connected:
        return {
            "success": False,
            "error_code": ERROR_NOT_CONNECTED,
            "message": "Hand not connected",
        }

    if _scheduler is not None:
        _scheduler.clear()

    return {
        "success": True,
        "error_code": 0,
        "message": "Realtime queue cleared; holding last target",
    }


def hold_snapshot(
    joint_positions: Mapping[str, float],
) -> dict:
    """把当前视觉姿态下发一次，并保持该固定目标。

    与 update_from_vision() 不同，本函数不会持续接收后续视觉帧。
    """
    global _last_error

    if hand is None or not _connected:
        return {
            "success": False,
            "status": "failed",
            "error_code": ERROR_NOT_CONNECTED,
            "message": "Hand not connected",
        }

    if _config is None or not _calibrated:
        return {
            "success": False,
            "status": "failed",
            "error_code": ERROR_NOT_CALIBRATED,
            "message": "Hand not calibrated",
        }

    try:
        if _scheduler is not None:
            _scheduler.clear()

        _config.validate_angles(joint_positions)
        result = _execute_angles(
            joint_positions,
            wait=False,
        )

        if not result.success:
            raise RuntimeError(
                f"{result.message}: {result.failures}"
            )

        return {
            "success": True,
            "status": "snapshot_hold",
            "error_code": 0,
            "message": "Snapshot target sent and held",
            "target_raw": dict(result.targets),
            "speed": _MOTION_SPEED,
            "acc": _MOTION_ACC,
            "torque": _MOTION_TORQUE,
        }

    except ConfigError as exc:
        _last_error = str(exc)
        return {
            "success": False,
            "status": "failed",
            "error_code": ERROR_INVALID_COMMAND,
            "message": str(exc),
        }
    except Exception as exc:
        _last_error = str(exc)
        return {
            "success": False,
            "status": "failed",
            "error_code": ERROR_EXECUTION,
            "message": str(exc),
        }


def preview_joint_targets(
    joint_positions: Mapping[str, float],
    read_actual: bool = True,
) -> dict:
    """输出角度到电机 raw 的映射，便于判断方向是否反了。"""
    if _config is None:
        return {
            "success": False,
            "error_code": ERROR_NOT_CALIBRATED,
            "message": "Runtime config is not loaded",
        }

    try:
        _config.validate_angles(joint_positions)
        target_raw = _config.angles_to_raw(
            joint_positions
        )

        actual_positions: Dict[int, int] = {}
        failures: Dict[int, str] = {}

        if read_actual and hand is not None and _connected:
            with _bus_lock:
                actual_positions, failures = (
                    hand.read_positions(
                        target_raw.keys()
                    )
                )

        joints = {}
        for joint_name, angle in joint_positions.items():
            motor_id = _config.joint_to_motor_map[
                joint_name
            ]
            target = target_raw[motor_id]
            actual = actual_positions.get(motor_id)

            joints[joint_name] = {
                "angle_deg": float(angle),
                "motor_id": motor_id,
                "target_raw": target,
                "actual_raw": actual,
                "delta_raw": (
                    None
                    if actual is None
                    else target - actual
                ),
                "reversed": (
                    joint_name
                    in _config.reverse_joints
                ),
                "rom_deg": list(
                    _config.joint_roms[joint_name]
                ),
                "raw_limits": [
                    _config.calibrations[
                        joint_name
                    ].raw_min,
                    _config.calibrations[
                        joint_name
                    ].raw_max,
                ],
            }

        return {
            "success": True,
            "error_code": 0,
            "joints": joints,
            "read_failures": failures,
            "motion": {
                "speed": _MOTION_SPEED,
                "acc": _MOTION_ACC,
                "torque": _MOTION_TORQUE,
                "control_hz": _CONTROL_HZ,
            },
        }

    except Exception as exc:
        return {
            "success": False,
            "error_code": ERROR_INVALID_COMMAND,
            "message": str(exc),
        }

def init(
    config_path: str | None = None,
    calibration_path: str | None = None,
) -> bool:
    """加载配置、连接、验证人工 limits，并同步回中。"""
    global hand, _config, _scheduler
    global _connected, _calibrated, _last_error

    with _state_lock:
        cleanup()
        _last_error = ""

        try:
            _config = load_runtime_config(
                config_path=config_path,
                calibration_path=calibration_path,
            )
            hand = MultiMotorControl(
                port=_config.port,
                baudrate=_config.baudrate,
            )

            if not hand.connect():
                raise ConnectionError(
                    f"cannot connect to {_config.port}"
                )

            _connected = True
            _calibrated = _config.calibrated

            print(
                f"[hand_interface] connected: "
                f"{_config.port} @ {_config.baudrate}"
            )
            print(
                f"[hand_interface] calibrated: "
                f"{_calibrated}"
            )

            if not _calibrated:
                invalid = [
                    joint_name
                    for joint_name, calibration
                    in _config.calibrations.items()
                    if not calibration.valid
                ]
                raise ConfigError(
                    "manual raw calibration incomplete: "
                    + ", ".join(invalid)
                )

            neutral_result = _execute_angles(
                _config.neutral_position,
                wait=True,
            )
            if not neutral_result.success:
                raise RuntimeError(
                    f"neutral failed: "
                    f"{neutral_result.message}; "
                    f"{neutral_result.failures}"
                )

            _scheduler = _LatestFrameScheduler(
                controller=hand,
                bus_lock=_bus_lock,
                control_hz=_CONTROL_HZ,
            )
            _scheduler.start()
            return True

        except Exception as exc:
            _last_error = str(exc)
            print(f"[hand_interface] init failed: {_last_error}")
            cleanup()
            return False


def do_gesture(
    gesture_name: str,
    hold_time_sec: float = 2.0,
    return_to_neutral: bool = False,
) -> dict:
    global _last_error

    if not _connected or hand is None:
        return _failed(
            ERROR_NOT_CONNECTED,
            "hand is not connected",
        )
    if not _calibrated or _config is None:
        return _failed(
            ERROR_NOT_CALIBRATED,
            "hand is not calibrated",
        )
    if gesture_name not in GESTURE_ANGLES:
        return _failed(
            ERROR_INVALID_COMMAND,
            f"unknown gesture: {gesture_name}",
        )

    try:
        if _scheduler is not None:
            _scheduler.clear()

        result = _execute_angles(
            GESTURE_ANGLES[gesture_name],
            wait=True,
        )
        if not result.success:
            raise RuntimeError(
                f"{result.message}: {result.failures}"
            )

        if hold_time_sec > 0:
            time.sleep(float(hold_time_sec))

        if return_to_neutral:
            neutral_result = _execute_angles(
                _config.neutral_position,
                wait=True,
            )
            if not neutral_result.success:
                raise RuntimeError(
                    "return to neutral failed: "
                    f"{neutral_result.message}; "
                    f"{neutral_result.failures}"
                )

        return {
            "success": True,
            "execution_status": "completed",
            "current_action": gesture_name,
            "connected": True,
            "calibrated": True,
            "error_code": 0,
            "error_message": "",
        }

    except ConfigError as exc:
        _last_error = str(exc)
        return _failed(ERROR_INVALID_COMMAND, str(exc))
    except Exception as exc:
        _last_error = str(exc)
        return _failed(ERROR_EXECUTION, str(exc))


def do_joint_command(
    joint_positions: dict,
    hold_time_sec: float = 3.0,
) -> dict:
    """阻塞式确定姿态接口，保留给非实时上层。"""
    global _last_error

    if not _connected or hand is None:
        return {
            "status": "failed",
            "error_code": ERROR_NOT_CONNECTED,
            "message": "Hand not connected",
        }
    if not _calibrated or _config is None:
        return {
            "status": "failed",
            "error_code": ERROR_NOT_CALIBRATED,
            "message": "Hand not calibrated",
        }

    try:
        _config.validate_angles(joint_positions)
        if _scheduler is not None:
            _scheduler.clear()

        result = _execute_angles(
            joint_positions,
            wait=True,
        )
        if not result.success:
            raise RuntimeError(
                f"{result.message}: {result.failures}"
            )

        if hold_time_sec > 0:
            time.sleep(float(hold_time_sec))

        return {
            "status": "completed",
            "error_code": 0,
            "message": "Joint command executed",
        }

    except ConfigError as exc:
        _last_error = str(exc)
        return {
            "status": "failed",
            "error_code": ERROR_INVALID_COMMAND,
            "message": str(exc),
        }
    except Exception as exc:
        _last_error = str(exc)
        return {
            "status": "failed",
            "error_code": ERROR_EXECUTION,
            "message": str(exc),
        }


def update_from_vision(
    joint_positions: Mapping[str, float],
    confidence: Optional[float] = None,
    min_confidence: float = 0.5,
) -> dict:
    """非阻塞视觉帧入口；旧帧会被新帧覆盖。"""
    if not _connected or hand is None:
        return {
            "status": "failed",
            "error_code": ERROR_NOT_CONNECTED,
            "message": "Hand not connected",
        }
    if not _calibrated or _config is None:
        return {
            "status": "failed",
            "error_code": ERROR_NOT_CALIBRATED,
            "message": "Hand not calibrated",
        }
    if confidence is not None and confidence < min_confidence:
        return {
            "status": "ignored",
            "error_code": 0,
            "message": "Low-confidence frame ignored",
        }

    try:
        raw_targets = _config.angles_to_raw(joint_positions)
        if _scheduler is None:
            raise RuntimeError("Realtime scheduler is not running")

        sequence = _scheduler.submit(raw_targets)
        return {
            "status": "queued",
            "error_code": 0,
            "message": "Latest vision frame queued",
            "sequence": sequence,
        }
    except ConfigError as exc:
        return {
            "status": "failed",
            "error_code": ERROR_INVALID_COMMAND,
            "message": str(exc),
        }
    except Exception as exc:
        return {
            "status": "failed",
            "error_code": ERROR_EXECUTION,
            "message": str(exc),
        }


def get_status() -> dict:
    if hand is None:
        return {
            "connected": False,
            "calibrated": False,
            "last_error": _last_error,
        }

    positions: Dict[int, int] = {}
    currents: Dict[int, int] = {}
    temperatures: Dict[int, int] = {}
    position_failures: Dict[int, str] = {}
    current_failures: Dict[int, str] = {}
    temperature_failures: Dict[int, str] = {}

    if _connected:
        try:
            with _bus_lock:
                positions, position_failures = (
                    hand.read_positions()
                )
                currents, current_failures = (
                    hand.read_currents_raw()
                )
                temperatures, temperature_failures = (
                    hand.read_temperatures()
                )
        except Exception as exc:
            logger.exception("Status read failed")

    result = {
        "connected": _connected,
        "calibrated": _calibrated,
        "motor_positions": positions,
        "motor_currents": currents,
        "motor_currents_raw": currents,
        "motor_temps": temperatures,
        "position_failures": position_failures,
        "current_failures": current_failures,
        "temperature_failures": temperature_failures,
        "last_error": _last_error,
        "motion_config": {
            "speed": _MOTION_SPEED,
            "acc": _MOTION_ACC,
            "torque": _MOTION_TORQUE,
            "control_hz": _CONTROL_HZ,
        },
    }

    if _scheduler is not None:
        result["stream_statistics"] = {
            "submitted": _scheduler.stats.submitted,
            "sent": _scheduler.stats.sent,
            "dropped": _scheduler.stats.dropped,
            "failed": _scheduler.stats.failed,
            "last_error": _scheduler.last_error,
        }

    return result


def emergency_stop() -> dict:
    if hand is None or not _connected:
        return {
            "success": False,
            "error_code": ERROR_NOT_CONNECTED,
            "message": "Hand not connected",
        }

    if _scheduler is not None:
        _scheduler.clear()

    with _bus_lock:
        ok, failures = hand.emergency_stop()

    return {
        "success": ok,
        "error_code": 0 if ok else ERROR_EXECUTION,
        "message": "" if ok else str(failures),
        "failures": failures,
    }


def cleanup() -> None:
    global hand, _config, _scheduler
    global _connected, _calibrated

    with _state_lock:
        if _scheduler is not None:
            _scheduler.stop()
            _scheduler = None

        if hand is not None:
            try:
                if hand.is_connected:
                    with _bus_lock:
                        hand.emergency_stop()
            except Exception:
                logger.exception("Failed to disable torque")

            try:
                hand.disconnect()
            except Exception:
                logger.exception("Failed to disconnect")

        hand = None
        _config = None
        _connected = False
        _calibrated = False
