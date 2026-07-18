"""
hand_interface.py --- 与上层既有接口兼容的轻量底层实现

上层原调用方式无需修改：

    from hand_interface import init, do_gesture, get_status, cleanup

    init()
    do_gesture("hand_open")
    status = get_status()
    cleanup()

额外兼容原上层文件中的：
    do_joint_command(joint_positions, hold_time_sec=3.0)

底层不再创建 OrcaHand，不运行旧自动撞限位校准。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Optional

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


logger = logging.getLogger("hand_interface")

ERROR_NOT_CONNECTED = 1001
ERROR_NOT_CALIBRATED = 1002
ERROR_INVALID_COMMAND = 1003
ERROR_EXECUTION = 1007

DEFAULT_SPEED = 80
DEFAULT_ACC = 30
DEFAULT_TORQUE = 200
DEFAULT_CONTROL_HZ = 20.0


@dataclass
class _StreamStats:
    submitted: int = 0
    sent: int = 0
    dropped: int = 0
    failed: int = 0


class _LatestFrameScheduler:
    """视觉实时控制使用的 latest-wins 调度器。"""

    def __init__(
        self,
        controller: MultiMotorControl,
        bus_lock: threading.RLock,
        control_hz: float = DEFAULT_CONTROL_HZ,
    ):
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
                    self._controller.set_torque_many(
                        targets.keys(),
                        True,
                    )
                    result = self._controller.set_positions_sync(
                        targets,
                        speed=DEFAULT_SPEED,
                        acc=DEFAULT_ACC,
                        torque=DEFAULT_TORQUE,
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
                logger.exception(
                    "Realtime command failed"
                )

            next_send = time.monotonic() + self._period


hand: Optional[MultiMotorControl] = None
_config: Optional[RuntimeConfig] = None
_scheduler: Optional[_LatestFrameScheduler] = None

_connected = False
_calibrated = False
_last_error = ""

_state_lock = threading.RLock()
_bus_lock = threading.RLock()


def _failed_gesture(
    error_code: int,
    message: str,
) -> dict:
    return {
        "success": False,
        "execution_status": "failed",
        "error_code": error_code,
        "error_message": message,
    }


def _require_ready() -> Optional[dict]:
    if not _connected or hand is None:
        return _failed_gesture(
            ERROR_NOT_CONNECTED,
            "hand is not connected",
        )

    if not _calibrated or _config is None:
        return _failed_gesture(
            ERROR_NOT_CALIBRATED,
            "hand is not calibrated",
        )

    return None


def _execute_angles(
    joint_positions: Mapping[str, float],
    wait: bool = True,
) -> MultiMotorResult:
    if hand is None or _config is None:
        raise RuntimeError("Hand is not initialized")

    raw_targets = _config.angles_to_raw(
        joint_positions
    )

    with _bus_lock:
        torque_ok, torque_failures = (
            hand.set_torque_many(
                raw_targets.keys(),
                True,
            )
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
            speed=DEFAULT_SPEED,
            acc=DEFAULT_ACC,
            torque=DEFAULT_TORQUE,
            wait=wait,
            timeout= 50.0,
            tolerance=30,
        )


def init(
    config_path: str | None = None,
    calibration_path: str | None = None,
) -> bool:
    """加载 config_safe.yaml、连接、验证人工标定并回中。

    不运行旧 OrcaHand 自动撞限位校准。

    Returns:
        连接、标定验证和回中全部成功时返回 True。
    """
    global hand
    global _config
    global _scheduler
    global _connected
    global _calibrated
    global _last_error

    with _state_lock:
        cleanup()
        _last_error = ""

        try:
            _config = load_runtime_config(
                config_path=config_path,
                calibration_path=calibration_path,
            )
        except Exception as exc:
            _last_error = f"config load failed: {exc}"
            print(f"[hand_interface] {_last_error}")
            return False

        hand = MultiMotorControl(
            port=_config.port,
            baudrate=_config.baudrate,
        )

        if not hand.connect():
            _connected = False
            _last_error = (
                f"cannot connect to {_config.port}"
            )
            print(
                f"[hand_interface] connect: "
                f"{_last_error}"
            )
            return False

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
            _last_error = (
                "manual raw calibration incomplete: "
                + ", ".join(invalid)
            )
            print(f"[hand_interface] {_last_error}")
            return False

        try:
            neutral_result = _execute_angles(
                _config.neutral_position,
                wait=True,
            )
            if not neutral_result.success:
                _last_error = (
                    f"neutral failed: "
                    f"{neutral_result.message}; "
                    f"{neutral_result.failures}"
                )
                print(f"[hand_interface] {_last_error}")
                return False
        except Exception as exc:
            _last_error = f"neutral failed: {exc}"
            print(f"[hand_interface] {_last_error}")
            return False

        _scheduler = _LatestFrameScheduler(
            controller=hand,
            bus_lock=_bus_lock,
            control_hz=DEFAULT_CONTROL_HZ,
        )
        _scheduler.start()

        return True


def do_gesture(
    gesture_name: str,
    hold_time_sec: float = 2.0,
    return_to_neutral: bool = False,
) -> dict:
    """执行 hand_open / hand_close / pinch_grasp。"""
    global _last_error

    readiness_error = _require_ready()
    if readiness_error is not None:
        return readiness_error

    if gesture_name not in GESTURE_ANGLES:
        return _failed_gesture(
            ERROR_INVALID_COMMAND,
            f"unknown gesture: {gesture_name}",
        )

    try:
        assert _scheduler is not None
        assert _config is not None

        _scheduler.clear()

        result = _execute_angles(
            GESTURE_ANGLES[gesture_name],
            wait=True,
        )
        if not result.success:
            message = (
                f"{result.message}: {result.failures}"
            )
            _last_error = message
            return _failed_gesture(
                ERROR_EXECUTION,
                message,
            )

        if hold_time_sec > 0:
            time.sleep(float(hold_time_sec))

        if return_to_neutral:
            neutral_result = _execute_angles(
                _config.neutral_position,
                wait=True,
            )
            if not neutral_result.success:
                message = (
                    f"return to neutral failed: "
                    f"{neutral_result.message}; "
                    f"{neutral_result.failures}"
                )
                _last_error = message
                return _failed_gesture(
                    ERROR_EXECUTION,
                    message,
                )

        return {
            "success": True,
            "execution_status": "completed",
            "current_action": gesture_name,
            "connected": _connected,
            "calibrated": _calibrated,
            "error_code": 0,
            "error_message": "",
        }

    except ConfigError as exc:
        _last_error = str(exc)
        return _failed_gesture(
            ERROR_INVALID_COMMAND,
            str(exc),
        )
    except Exception as exc:
        _last_error = str(exc)
        return _failed_gesture(
            ERROR_EXECUTION,
            str(exc),
        )


def get_status() -> dict:
    """返回与上层旧接口相同的核心字段。"""
    if hand is None:
        return {"connected": False}

    currents: Dict[int, int] = {}
    temperatures: Dict[int, int] = {}
    positions: Dict[int, int] = {}

    current_failures: Dict[int, str] = {}
    temperature_failures: Dict[int, str] = {}
    position_failures: Dict[int, str] = {}

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
            global _last_error
            _last_error = str(exc)

    status = {
        "connected": _connected,
        "calibrated": _calibrated,
        # 保留上层旧字段名。当前值是寄存器 raw，避免假装成 mA。
        "motor_currents": currents,
        "motor_temps": temperatures,
        "motor_positions": positions,
        # 新增字段不会破坏旧上层。
        "motor_currents_raw": currents,
        "position_failures": position_failures,
        "current_failures": current_failures,
        "temperature_failures": temperature_failures,
        "last_error": _last_error,
    }

    if _scheduler is not None:
        status["stream_statistics"] = {
            "submitted": _scheduler.stats.submitted,
            "sent": _scheduler.stats.sent,
            "dropped": _scheduler.stats.dropped,
            "failed": _scheduler.stats.failed,
            "last_error": _scheduler.last_error,
        }

    return status


def cleanup() -> None:
    """停止实时线程、关闭全部扭矩并释放 COM 口。"""
    global hand
    global _config
    global _scheduler
    global _connected
    global _calibrated

    with _state_lock:
        if _scheduler is not None:
            _scheduler.stop()
            _scheduler = None

        if hand is not None:
            try:
                if hand.is_connected:
                    with _bus_lock:
                        ok, failures = (
                            hand.emergency_stop()
                        )
                        if not ok:
                            logger.error(
                                "Torque disable failures: %s",
                                failures,
                            )
            except Exception:
                logger.exception(
                    "Failed to disable torque"
                )

            try:
                hand.disconnect()
            except Exception:
                logger.exception(
                    "Failed to disconnect"
                )

        hand = None
        _config = None
        _connected = False
        _calibrated = False


def do_joint_command(
    joint_positions: dict,
    hold_time_sec: float = 3.0,
) -> dict:
    """兼容上层原有的关节角度控制接口。"""
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

        assert _scheduler is not None
        _scheduler.clear()

        result = _execute_angles(
            joint_positions,
            wait=True,
        )
        if not result.success:
            message = (
                f"{result.message}: {result.failures}"
            )
            _last_error = message
            return {
                "status": "failed",
                "error_code": ERROR_EXECUTION,
                "message": message,
            }

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
    """非阻塞实时接口：视觉帧只保留最新一帧。

    这是实时模仿推荐使用的入口；旧上层仍可继续调用
    do_joint_command()。
    """
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

    if (
        confidence is not None
        and confidence < min_confidence
    ):
        return {
            "status": "ignored",
            "error_code": 0,
            "message": "Low-confidence frame ignored",
        }

    try:
        raw_targets = _config.angles_to_raw(
            joint_positions
        )
        assert _scheduler is not None
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


def emergency_stop() -> dict:
    """扩展接口：立即清空视觉帧并关闭 1~17 扭矩。"""
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
