"""
multi_motor_control.py --- 17 电机同步总线控制层

职责：
- 接受 motor_id -> raw_position 的多电机目标；
- 优先使用 Feetech GroupSyncWrite，一包下发多个电机目标；
- 不支持同步写时自动退化为快速顺序写；
- 统一管理多电机扭矩、读取和急停；
- 整个 COM5 总线只有一个控制对象，禁止为 17 个电机创建 17 个线程。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Mapping, Optional, Tuple

from .motor_control import (
    COMM_SUCCESS,
    DEFAULT_ACC,
    DEFAULT_SPEED,
    DEFAULT_TORQUE,
    MOTOR_ID_MAX,
    MOTOR_ID_MIN,
    POS_MAX,
    POS_MIN,
    MotorControl,
)

# motor_control 导入后，Feetech SDK 路径已经加入 sys.path。
from feetech import (
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_TEMPERATURE,
)  # type: ignore  # noqa: E402


logger = logging.getLogger("multi_motor_control")


@dataclass(frozen=True)
class MultiMotorResult:
    success: bool
    message: str
    targets: Dict[int, int] = field(default_factory=dict)
    failures: Dict[int, str] = field(default_factory=dict)
    used_sync_write: bool = False


class MultiMotorControl(MotorControl):
    """在 MotorControl 之上提供多电机同步接口。"""

    def __init__(
        self,
        port: str = "COM5",
        baudrate: int = 1_000_000,
    ):
        super().__init__(port=port, baudrate=baudrate)
        self._bus_lock = threading.RLock()

    @staticmethod
    def _normalize_targets(
        targets: Mapping[int, int],
    ) -> Dict[int, int]:
        if not targets:
            raise ValueError("targets cannot be empty")

        normalized: Dict[int, int] = {}
        for motor_id, position in targets.items():
            motor_id = int(motor_id)
            position = int(position)

            if not MOTOR_ID_MIN <= motor_id <= MOTOR_ID_MAX:
                raise ValueError(
                    f"motor_id must be in "
                    f"[{MOTOR_ID_MIN}, {MOTOR_ID_MAX}], got {motor_id}"
                )

            normalized[motor_id] = max(
                POS_MIN,
                min(POS_MAX, position),
            )

        return dict(sorted(normalized.items()))

    def set_torque_many(
        self,
        motor_ids: Iterable[int],
        enable: bool,
    ) -> Tuple[bool, Dict[int, str]]:
        """依次可靠地设置多个电机扭矩。"""
        failures: Dict[int, str] = {}

        with self._bus_lock:
            for motor_id in sorted(set(int(mid) for mid in motor_ids)):
                ok, message = super().set_torque(
                    motor_id,
                    enable,
                )
                if not ok:
                    failures[motor_id] = message

        return not failures, failures

    def set_positions_sync(
        self,
        targets: Mapping[int, int],
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        torque: Optional[int] = None,
        wait: bool = False,
        timeout: float = 3.0,
        tolerance: int = 12,
    ) -> MultiMotorResult:
        """通过一个同步写数据包下发多个目标位置。

        同步写是“同一串口包中携带多个电机目标”，适合一根手指、
        多根手指和 17 关节手势。同步写本身没有逐电机应答，因此，
        真正的状态确认通过后续 read_positions / wait_until_reached 完成。
        """
        self._check()
        assert self._pk is not None

        normalized = self._normalize_targets(targets)
        speed = DEFAULT_SPEED if speed is None else int(speed)
        acc = DEFAULT_ACC if acc is None else int(acc)
        torque = DEFAULT_TORQUE if torque is None else int(torque)

        if speed < 0:
            raise ValueError("speed must be >= 0")
        if not 0 <= acc <= 254:
            raise ValueError("acc must be in [0, 254]")
        if not 0 <= torque <= 1000:
            raise ValueError("torque must be in [0, 1000]")

        sync_group = getattr(self._pk, "groupSyncWrite", None)
        sync_add = getattr(self._pk, "SyncWritePosEx", None)

        # 首选 SDK 自带同步写。
        if sync_group is not None and callable(sync_add):
            add_failures: Dict[int, str] = {}

            with self._bus_lock:
                sync_group.clearParam()
                try:
                    for motor_id, position in normalized.items():
                        added = sync_add(
                            motor_id,
                            position,
                            speed,
                            acc,
                            torque,
                        )
                        # 一些 SDK 版本返回 None，一些可能返回 bool。
                        if added is False:
                            add_failures[motor_id] = "sync_add_failed"

                    if add_failures:
                        return MultiMotorResult(
                            success=False,
                            message="failed to build sync packet",
                            targets=normalized,
                            failures=add_failures,
                            used_sync_write=True,
                        )

                    result = sync_group.txPacket()
                finally:
                    sync_group.clearParam()

            if result != COMM_SUCCESS:
                return MultiMotorResult(
                    success=False,
                    message=f"sync_write_comm_fail={result}",
                    targets=normalized,
                    used_sync_write=True,
                )

            if wait:
                reached, failures = self.wait_until_reached(
                    normalized,
                    timeout=timeout,
                    tolerance=tolerance,
                )
                return MultiMotorResult(
                    success=reached,
                    message="done" if reached else "wait_timeout",
                    targets=normalized,
                    failures=failures,
                    used_sync_write=True,
                )

            return MultiMotorResult(
                success=True,
                message="ok",
                targets=normalized,
                used_sync_write=True,
            )

        # SDK 不含同步写时，退化为快速顺序下发；仍由单一总线对象串行访问。
        failures: Dict[int, str] = {}
        with self._bus_lock:
            for motor_id, position in normalized.items():
                ok, message = super().set_position(
                    motor_id=motor_id,
                    position=position,
                    speed=speed,
                    acc=acc,
                    torque=torque,
                    wait=False,
                )
                if not ok:
                    failures[motor_id] = message

        if failures:
            return MultiMotorResult(
                success=False,
                message="sequential_write_failed",
                targets=normalized,
                failures=failures,
                used_sync_write=False,
            )

        if wait:
            reached, wait_failures = self.wait_until_reached(
                normalized,
                timeout=timeout,
                tolerance=tolerance,
            )
            return MultiMotorResult(
                success=reached,
                message="done" if reached else "wait_timeout",
                targets=normalized,
                failures=wait_failures,
                used_sync_write=False,
            )

        return MultiMotorResult(
            success=True,
            message="ok_fallback_sequential",
            targets=normalized,
            used_sync_write=False,
        )

    def read_positions(
        self,
        motor_ids: Iterable[int] = range(1, 18),
    ) -> Tuple[Dict[int, int], Dict[int, str]]:
        positions: Dict[int, int] = {}
        failures: Dict[int, str] = {}

        with self._bus_lock:
            for motor_id in sorted(set(int(mid) for mid in motor_ids)):
                position, ok, message = super().read_position(
                    motor_id
                )
                if ok:
                    positions[motor_id] = position
                else:
                    failures[motor_id] = message

        return positions, failures

    def read_currents_raw(
        self,
        motor_ids: Iterable[int] = range(1, 18),
    ) -> Tuple[Dict[int, int], Dict[int, str]]:
        """读取电流寄存器原始值。

        当前项目尚未确认 raw 值到 mA 的准确换算，因此不伪装成 mA。
        """
        self._check()
        assert self._pk is not None

        currents: Dict[int, int] = {}
        failures: Dict[int, str] = {}

        with self._bus_lock:
            for motor_id in sorted(set(int(mid) for mid in motor_ids)):
                raw, result, error = self._pk.read2ByteTxRx(
                    motor_id,
                    SMS_STS_PRESENT_CURRENT_L,
                )
                ok, message = self._check_response(
                    motor_id,
                    result,
                    error,
                    "read_current",
                )
                if ok:
                    currents[motor_id] = int(
                        self._pk.scs_tohost(raw, 15)
                    )
                else:
                    failures[motor_id] = message

        return currents, failures


    def read_temperatures(
        self,
        motor_ids: Iterable[int] = range(1, 18),
    ) -> Tuple[Dict[int, int], Dict[int, str]]:
        """读取多个电机温度，单位 °C。"""
        self._check()
        assert self._pk is not None

        temperatures: Dict[int, int] = {}
        failures: Dict[int, str] = {}

        with self._bus_lock:
            for motor_id in sorted(set(int(mid) for mid in motor_ids)):
                raw, result, error = self._pk.read1ByteTxRx(
                    motor_id,
                    SMS_STS_PRESENT_TEMPERATURE,
                )
                ok, message = self._check_response(
                    motor_id,
                    result,
                    error,
                    "read_temperature",
                )
                if ok:
                    temperatures[motor_id] = int(raw)
                else:
                    failures[motor_id] = message

        return temperatures, failures

    def wait_until_reached(
        self,
        targets: Mapping[int, int],
        timeout: float = 3.0,
        tolerance: int = 12,
        poll_period: float = 0.05,
    ) -> Tuple[bool, Dict[int, str]]:
        """等待所有目标关节到达给定容差。"""
        normalized = self._normalize_targets(targets)
        deadline = time.monotonic() + timeout
        last_failures: Dict[int, str] = {}

        while time.monotonic() < deadline:
            positions, read_failures = self.read_positions(
                normalized.keys()
            )
            last_failures = dict(read_failures)

            not_reached = {
                motor_id: (
                    f"target={target}, "
                    f"actual={positions.get(motor_id, 'N/A')}"
                )
                for motor_id, target in normalized.items()
                if (
                    motor_id not in positions
                    or abs(positions[motor_id] - target) > tolerance
                )
            }

            if not read_failures and not not_reached:
                return True, {}

            last_failures.update(not_reached)
            time.sleep(poll_period)

        return False, last_failures

    def emergency_stop(self) -> Tuple[bool, Dict[int, str]]:
        """关闭 1~17 号电机扭矩。"""
        return self.set_torque_many(
            range(MOTOR_ID_MIN, MOTOR_ID_MAX + 1),
            False,
        )
