"""
motor_control.py --- 单电机控制层（修正版）

基于 Feetech STS3215 底层协议，封装单电机控制接口。
所有位置值使用原始单位（0~4095 = 1 圈），不使用弧度转换。

本版本修复：
  1. 避免 test 模式重复打开 COM 口；
  2. 所有读写统一检查通信结果和状态位；
  3. 仅忽略本项目已确认的 ERRBIT_VOLTAGE；
  4. Ctrl+C 或异常退出时可靠释放串口；
  5. test 默认只测试一个电机，并使用较小步长、速度和扭矩；
  6. 命令行运动后默认关闭扭矩。

示例：
  python flexible_hand/motor_control.py info --id 1
  python flexible_hand/motor_control.py scan
  python flexible_hand/motor_control.py test --id 3 --delta 20
  python flexible_hand/motor_control.py jog --id 3 --delta 20
  python flexible_hand/motor_control.py pos --id 3 --pos 2100
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Callable, Optional, Tuple, TypeVar

# -- 添加 feetech 库路径 --
HARDWARE_DIR = (
    Path(__file__).resolve().parents[1]
    / "third_party"
    / "orca_core"
    / "orca_core"
    / "hardware"
)
if str(HARDWARE_DIR) not in sys.path:
    sys.path.insert(0, str(HARDWARE_DIR))

from feetech import (  # noqa: E402
    COMM_SUCCESS,
    PortHandler,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_VOLTAGE,
    SMS_STS_TORQUE_ENABLE,
    sms_sts,
)
from feetech.protocol_packet_handler import ERRBIT_VOLTAGE  # noqa: E402

logger = logging.getLogger("motor_control")

# -- STS3215 / 本项目配置 --
MOTOR_ID_MIN = 1
MOTOR_ID_MAX = 17

POS_MIN = 0
POS_MAX = 4095
POS_MID = 2048

DEFAULT_BAUDRATE = 1_000_000

# 单关节初调采用保守参数；后续确认关节方向和软限位后再调整。
DEFAULT_SPEED = 20
DEFAULT_ACC = 10
DEFAULT_TORQUE = 100

# 老师已确认当前机械手使用 12V 供电。
# 对本项目，仅忽略 STS3215 返回包中的电压状态位；其他状态位仍报错。
IGNORED_ERROR_BITS = ERRBIT_VOLTAGE

T = TypeVar("T")


class MotorControl:
    """单电机控制接口。

    注意：
    - ``set_position`` 和 ``move_jog`` 不会自动使能扭矩；
    - 命令行入口会在运动前使能、结束后关闭扭矩；
    - 作为库使用时，应由调用者显式管理扭矩。
    """

    def __init__(
        self,
        port: str = "COM5",
        baudrate: int = DEFAULT_BAUDRATE,
    ):
        self.port_name = port
        self.baudrate = baudrate
        self._ph: Optional[PortHandler] = None
        self._pk: Optional[sms_sts] = None
        self._connected = False

    # ── 连接管理 ────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        """打开串口并初始化协议处理器。"""
        if self._connected:
            logger.warning("Already connected to %s", self.port_name)
            return True

        self._ph = PortHandler(self.port_name)
        self._ph.baudrate = self.baudrate

        try:
            if not self._ph.openPort():
                logger.error("Failed to open port %s", self.port_name)
                self._ph = None
                return False
        except Exception as exc:
            # 常见情况：COM 口被其他程序占用。
            logger.error(
                "Cannot open %s: %s",
                self.port_name,
                exc,
            )
            self._ph = None
            return False

        self._pk = sms_sts(self._ph)
        self._connected = True
        logger.info(
            "Connected to %s at %d baud",
            self.port_name,
            self.baudrate,
        )
        return True

    def disconnect(self) -> None:
        """关闭串口；即使上一次通信被 Ctrl+C 中断也尽量释放端口。"""
        ph = self._ph

        try:
            if ph is not None:
                # Feetech SDK 在收发途中被 KeyboardInterrupt 打断时，
                # is_using 可能残留为 True，导致后续一直 COMM_PORT_BUSY。
                if getattr(ph, "is_using", False):
                    logger.warning(
                        "Port was left busy by an interrupted transaction; "
                        "forcing release before close."
                    )
                    ph.is_using = False

                ph.closePort()
        except Exception as exc:
            logger.warning(
                "Error while closing %s: %s",
                self.port_name,
                exc,
            )
        finally:
            was_connected = self._connected
            self._connected = False
            self._pk = None
            self._ph = None
            if was_connected:
                logger.info("Disconnected")

    def __enter__(self) -> "MotorControl":
        if not self.connect():
            raise ConnectionError(
                f"Unable to connect to {self.port_name} "
                f"at {self.baudrate} baud"
            )
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.disconnect()
        return False

    # ── 输入和返回状态检查 ──────────────────────────────────────────

    @staticmethod
    def _validate_motor_id(motor_id: int) -> None:
        if not MOTOR_ID_MIN <= motor_id <= MOTOR_ID_MAX:
            raise ValueError(
                f"motor_id must be in "
                f"[{MOTOR_ID_MIN}, {MOTOR_ID_MAX}], got {motor_id}"
            )

    @staticmethod
    def _is_real_error(error_byte: int) -> bool:
        """是否包含本项目不能忽略的舵机状态位。"""
        return (error_byte & ~IGNORED_ERROR_BITS) != 0

    @staticmethod
    def _ignored_status_present(error_byte: int) -> bool:
        return (error_byte & IGNORED_ERROR_BITS) != 0

    def _check(self) -> None:
        if not self._connected or self._pk is None:
            raise RuntimeError(
                "Not connected. Call connect() first."
            )

    def _check_response(
        self,
        motor_id: int,
        result: int,
        error: int,
        operation: str,
    ) -> Tuple[bool, str]:
        """统一判断通信结果和舵机状态位。"""
        if result != COMM_SUCCESS:
            return False, f"{operation}_comm_fail={result}"

        if self._is_real_error(error):
            return False, f"{operation}_err=0x{error:X}"

        if self._ignored_status_present(error):
            logger.debug(
                "Motor %d %s returned ignored status bits: 0x%X",
                motor_id,
                operation,
                error,
            )

        return True, "ok"

    # ── 扭矩控制 ────────────────────────────────────────────────────

    def set_torque(
        self,
        motor_id: int,
        enable: bool,
    ) -> Tuple[bool, str]:
        """启用或禁用单个电机扭矩。"""
        self._check()
        self._validate_motor_id(motor_id)
        assert self._pk is not None

        result, error = self._pk.write1ByteTxRx(
            motor_id,
            SMS_STS_TORQUE_ENABLE,
            int(enable),
        )
        return self._check_response(
            motor_id,
            result,
            error,
            "torque",
        )

    # ── 位置控制 ────────────────────────────────────────────────────

    def set_position(
        self,
        motor_id: int,
        position: int,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        torque: Optional[int] = None,
        wait: bool = False,
        timeout: float = 3.0,
    ) -> Tuple[bool, str]:
        """控制单电机移动到指定原始位置。

        本方法不自动使能扭矩。
        """
        self._check()
        self._validate_motor_id(motor_id)
        assert self._pk is not None

        position = max(POS_MIN, min(POS_MAX, int(position)))
        speed = DEFAULT_SPEED if speed is None else int(speed)
        acc = DEFAULT_ACC if acc is None else int(acc)
        torque = DEFAULT_TORQUE if torque is None else int(torque)

        if speed < 0:
            raise ValueError("speed must be >= 0")
        if not 0 <= acc <= 254:
            raise ValueError("acc must be in [0, 254]")
        if not 0 <= torque <= 1000:
            raise ValueError("torque must be in [0, 1000]")

        result, error = self._pk.WritePosEx(
            motor_id,
            position,
            speed,
            acc,
            torque,
        )

        ok, msg = self._check_response(
            motor_id,
            result,
            error,
            "write_position",
        )
        if not ok:
            return False, msg

        if wait:
            return self._wait_for_stop(
                motor_id,
                timeout=timeout,
            )

        return True, "ok"

    def move_jog(
        self,
        motor_id: int,
        delta: int,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        torque: Optional[int] = None,
        wait: bool = True,
        timeout: float = 3.0,
    ) -> Tuple[bool, str, int]:
        """按相对步长移动单个电机。"""
        self._check()
        self._validate_motor_id(motor_id)

        current, ok, msg = self.read_position(motor_id)
        if not ok:
            return False, msg, 0

        new_pos = max(
            POS_MIN,
            min(POS_MAX, current + int(delta)),
        )

        ok, msg = self.set_position(
            motor_id=motor_id,
            position=new_pos,
            speed=speed,
            acc=acc,
            torque=torque,
            wait=wait,
            timeout=timeout,
        )
        return ok, msg, new_pos

    # ── 状态读取 ────────────────────────────────────────────────────

    def read_position(
        self,
        motor_id: int,
    ) -> Tuple[int, bool, str]:
        """读取当前位置，返回 ``(position, success, message)``。"""
        self._check()
        self._validate_motor_id(motor_id)
        assert self._pk is not None

        pos, result, error = self._pk.ReadPos(motor_id)
        ok, msg = self._check_response(
            motor_id,
            result,
            error,
            "read_position",
        )
        if not ok:
            return 0, False, msg

        return int(pos), True, "ok"

    def read_voltage(
        self,
        motor_id: int,
    ) -> Tuple[float, bool, str]:
        """读取当前电压，单位 V。"""
        self._check()
        self._validate_motor_id(motor_id)
        assert self._pk is not None

        raw, result, error = self._pk.read1ByteTxRx(
            motor_id,
            SMS_STS_PRESENT_VOLTAGE,
        )
        ok, msg = self._check_response(
            motor_id,
            result,
            error,
            "read_voltage",
        )
        if not ok:
            return 0.0, False, msg

        return raw / 10.0, True, "ok"

    def read_temperature(
        self,
        motor_id: int,
    ) -> Tuple[int, bool, str]:
        """读取当前温度，单位 °C。"""
        self._check()
        self._validate_motor_id(motor_id)
        assert self._pk is not None

        temp, result, error = self._pk.read1ByteTxRx(
            motor_id,
            SMS_STS_PRESENT_TEMPERATURE,
        )
        ok, msg = self._check_response(
            motor_id,
            result,
            error,
            "read_temperature",
        )
        if not ok:
            return 0, False, msg

        return int(temp), True, "ok"

    # ── 内部辅助 ────────────────────────────────────────────────────

    def _wait_for_stop(
        self,
        motor_id: int,
        timeout: float = 3.0,
        stable_samples: int = 5,
        position_tolerance: int = 1,
    ) -> Tuple[bool, str]:
        """通过连续位置判稳等待电机停止。"""
        start = time.monotonic()
        stable_count = 0
        last_pos: Optional[int] = None
        last_error = ""

        while time.monotonic() - start < timeout:
            pos, ok, msg = self.read_position(motor_id)

            if not ok:
                last_error = msg
                stable_count = 0
                time.sleep(0.05)
                continue

            if (
                last_pos is not None
                and abs(pos - last_pos) <= position_tolerance
            ):
                stable_count += 1
            else:
                stable_count = 0

            last_pos = pos

            if stable_count >= stable_samples:
                return True, "done"

            time.sleep(0.05)

        if last_error:
            return False, f"timeout; last_error={last_error}"
        return False, "timeout"

    def print_info(self, motor_id: int) -> None:
        """打印单个电机的位置、电压和温度。"""
        pos, pos_ok, pos_msg = self.read_position(motor_id)
        pos_str = str(pos) if pos_ok else pos_msg

        volt, volt_ok, volt_msg = self.read_voltage(motor_id)
        volt_str = f"{volt:.1f}V" if volt_ok else volt_msg

        temp, temp_ok, temp_msg = self.read_temperature(motor_id)
        temp_str = f"{temp}C" if temp_ok else temp_msg

        print(
            f"  Motor {motor_id:2d}: "
            f"pos={pos_str}  "
            f"volt={volt_str}  "
            f"temp={temp_str}"
        )


def confirm_motion(message: str, assume_yes: bool) -> bool:
    """在发送运动命令前要求人工确认。"""
    if assume_yes:
        return True

    print()
    print(message)
    response = input(
        "确认机械手周围无人、关节不会撞限位后，输入 y 继续: "
    )
    return response.strip().lower() in {"y", "yes"}


def quick_test(
    mc: MotorControl,
    motor_id: int,
    delta: int,
    speed: int,
    acc: int,
    torque: int,
    return_to_start: bool = True,
) -> bool:
    """只测试一个电机：相对移动后可返回起始位置。"""
    print("=" * 60)
    print(
        f"Single Motor Test: ID={motor_id}, "
        f"delta={delta}, speed={speed}, torque={torque}"
    )
    print("=" * 60)

    mc.print_info(motor_id)

    start_pos, ok, msg = mc.read_position(motor_id)
    if not ok:
        print(f"[FAIL] Cannot read start position: {msg}")
        return False

    torque_enabled = False
    try:
        ok, msg = mc.set_torque(motor_id, True)
        if not ok:
            print(f"[FAIL] Cannot enable torque: {msg}")
            return False
        torque_enabled = True

        ok, msg, new_pos = mc.move_jog(
            motor_id=motor_id,
            delta=delta,
            speed=speed,
            acc=acc,
            torque=torque,
            wait=True,
        )
        print(
            f"[{'OK' if ok else 'FAIL'}] "
            f"Motor {motor_id} jog {delta} -> {new_pos}: {msg}"
        )
        if not ok:
            return False

        time.sleep(0.3)

        if return_to_start:
            ok, msg = mc.set_position(
                motor_id=motor_id,
                position=start_pos,
                speed=speed,
                acc=acc,
                torque=torque,
                wait=True,
            )
            print(
                f"[{'OK' if ok else 'FAIL'}] "
                f"Return to start {start_pos}: {msg}"
            )
            return ok

        return True
    finally:
        if torque_enabled:
            ok, msg = mc.set_torque(motor_id, False)
            if not ok:
                logger.error(
                    "Failed to disable torque for motor %d: %s",
                    motor_id,
                    msg,
                )


def print_scan(mc: MotorControl) -> None:
    """通过 MotorControl 做简易只读扫描。"""
    print("ID  | Position | Voltage | Temp")
    print("-" * 48)

    for motor_id in range(MOTOR_ID_MIN, MOTOR_ID_MAX + 1):
        pos, pos_ok, pos_msg = mc.read_position(motor_id)
        voltage, volt_ok, volt_msg = mc.read_voltage(motor_id)
        temp, temp_ok, temp_msg = mc.read_temperature(motor_id)

        pos_s = str(pos) if pos_ok else pos_msg
        voltage_s = f"{voltage:.1f}V" if volt_ok else volt_msg
        temp_s = f"{temp}C" if temp_ok else temp_msg

        print(
            f"{motor_id:3d} | "
            f"{pos_s:>8} | "
            f"{voltage_s:>8} | "
            f"{temp_s}"
        )


def run_motion_with_torque(
    mc: MotorControl,
    motor_id: int,
    action: Callable[[], T],
) -> T:
    """命令行运动的安全包装：使能扭矩，结束时关闭。"""
    ok, msg = mc.set_torque(motor_id, True)
    if not ok:
        raise RuntimeError(
            f"Cannot enable torque for motor {motor_id}: {msg}"
        )

    try:
        return action()
    finally:
        ok, msg = mc.set_torque(motor_id, False)
        if not ok:
            logger.error(
                "Failed to disable torque for motor %d: %s",
                motor_id,
                msg,
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Single motor control for the flexible hand"
    )
    parser.add_argument(
        "action",
        nargs="?",
        default="info",
        choices=["test", "pos", "jog", "info", "scan"],
        help="Action to perform",
    )
    parser.add_argument(
        "--id",
        type=int,
        default=1,
        help="Motor ID (1~17)",
    )
    parser.add_argument(
        "--pos",
        type=int,
        default=POS_MID,
        help="Target position (0~4095)",
    )
    parser.add_argument(
        "--delta",
        type=int,
        default=20,
        help="Relative jog step; initial debugging should use a small value",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=DEFAULT_SPEED,
        help="Motion speed",
    )
    parser.add_argument(
        "--acc",
        type=int,
        default=DEFAULT_ACC,
        help="Acceleration (0~254)",
    )
    parser.add_argument(
        "--torque",
        type=int,
        default=DEFAULT_TORQUE,
        help="Torque limit (0~1000)",
    )
    parser.add_argument(
        "--port",
        type=str,
        default="COM5",
        help="Serial port",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="Serial baudrate",
    )
    parser.add_argument(
        "--wait",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Wait for motion to become stable",
    )
    parser.add_argument(
        "--return-to-start",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="In test mode, return to the position measured before the test",
    )
    parser.add_argument(
        "-y",
        "--yes",
        action="store_true",
        help="Skip the motion confirmation prompt",
    )
    return parser


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(message)s",
    )
    args = build_parser().parse_args()

    MotorControl._validate_motor_id(args.id)

    mc = MotorControl(
        port=args.port,
        baudrate=args.baudrate,
    )

    try:
        if not mc.connect():
            return 1

        if args.action == "info":
            mc.print_info(args.id)
            return 0

        if args.action == "scan":
            print_scan(mc)
            return 0

        motion_message = (
            f"即将控制 Motor {args.id}。"
            f" speed={args.speed}, acc={args.acc}, "
            f"torque={args.torque}。"
        )
        if args.action == "jog":
            motion_message += f" 相对移动 delta={args.delta}。"
        elif args.action == "pos":
            motion_message += f" 移动到 position={args.pos}。"
        else:
            motion_message += (
                f" 测试移动 delta={args.delta}"
                f"{' 并返回起点' if args.return_to_start else ''}。"
            )

        if not confirm_motion(motion_message, args.yes):
            print("Cancelled.")
            return 0

        if args.action == "test":
            return 0 if quick_test(
                mc=mc,
                motor_id=args.id,
                delta=args.delta,
                speed=args.speed,
                acc=args.acc,
                torque=args.torque,
                return_to_start=args.return_to_start,
            ) else 1

        if args.action == "pos":
            ok, msg = run_motion_with_torque(
                mc,
                args.id,
                lambda: mc.set_position(
                    motor_id=args.id,
                    position=args.pos,
                    speed=args.speed,
                    acc=args.acc,
                    torque=args.torque,
                    wait=args.wait,
                ),
            )
            print(
                f"[{'OK' if ok else 'FAIL'}] "
                f"Motor {args.id} -> {args.pos}: {msg}"
            )
            mc.print_info(args.id)
            return 0 if ok else 1

        if args.action == "jog":
            ok, msg, new_pos = run_motion_with_torque(
                mc,
                args.id,
                lambda: mc.move_jog(
                    motor_id=args.id,
                    delta=args.delta,
                    speed=args.speed,
                    acc=args.acc,
                    torque=args.torque,
                    wait=args.wait,
                ),
            )
            print(
                f"[{'OK' if ok else 'FAIL'}] "
                f"Motor {args.id} jog {args.delta} -> "
                f"{new_pos}: {msg}"
            )
            mc.print_info(args.id)
            return 0 if ok else 1

        return 1

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130
    except Exception as exc:
        logger.exception("Operation failed: %s", exc)
        return 1
    finally:
        mc.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
