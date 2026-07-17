"""
scan_motors.py --- 只读扫描 1~17 号电机（修正版）

功能：
  1. 依次 ping 1~17 号电机；
  2. 分别检查每一次读取的 result 和 error；
  3. 读取型号、位置、电压、温度和电流；
  4. 仅忽略本项目已确认的 ERRBIT_VOLTAGE；
  5. 任何退出路径都释放串口；
  6. 本脚本只读，不使能扭矩、不发送运动命令。

用法：
  python flexible_hand/scan_motors.py
  python flexible_hand/scan_motors.py --port COM5
  python flexible_hand/scan_motors.py --voltage-min 11 --voltage-max 13
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

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
    COMM_PORT_BUSY,
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    PortHandler,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_VOLTAGE,
    sms_sts,
)
from feetech.protocol_packet_handler import ERRBIT_VOLTAGE  # noqa: E402

DEFAULT_PORT = "COM5"
DEFAULT_BAUDRATE = 1_000_000
IGNORED_ERROR_BITS = ERRBIT_VOLTAGE

MOTOR_INFO = {
    1: ("wrist", "手腕屈伸"),
    2: ("index_pip", "食指近端"),
    3: ("index_mcp", "食指掌指"),
    4: ("index_abd", "食指外展"),
    5: ("middle_abd", "中指外展"),
    6: ("ring_abd", "无名指外展"),
    7: ("ring_mcp", "无名指掌指"),
    8: ("ring_pip", "无名指近端"),
    9: ("middle_pip", "中指近端"),
    10: ("middle_mcp", "中指掌指"),
    11: ("pinky_pip", "小指近端"),
    12: ("pinky_mcp", "小指掌指"),
    13: ("pinky_abd", "小指外展"),
    14: ("thumb_abd", "拇指外展"),
    15: ("thumb_mcp", "拇指掌指"),
    16: ("thumb_dip", "拇指远端"),
    17: ("thumb_cmc", "拇指根部"),
}


def is_real_error(error_byte: int) -> bool:
    """仅把非 ERRBIT_VOLTAGE 状态位视为本项目中的真实异常。"""
    return (error_byte & ~IGNORED_ERROR_BITS) != 0


def has_ignored_voltage_bit(error_byte: int) -> bool:
    return (error_byte & ERRBIT_VOLTAGE) != 0


def response_ok(result: int, error: int) -> bool:
    return result == COMM_SUCCESS and not is_real_error(error)


def format_comm_result(result: int) -> str:
    if result == COMM_RX_TIMEOUT:
        return "TIMEOUT"
    if result == COMM_PORT_BUSY:
        return "BUSY"
    return f"COMM={result}"


def read_status_text(
    result: int,
    error: int,
    label: str,
) -> Optional[str]:
    """返回需要显示在 Status 栏中的异常文本；正常则返回 None。"""
    if result != COMM_SUCCESS:
        return f"{label}:{format_comm_result(result)}"
    if is_real_error(error):
        return f"{label}:ERR=0x{error:X}"
    return None


def note_voltage_bit(
    error: int,
    voltage_bit_seen: bool,
) -> bool:
    return voltage_bit_seen or has_ignored_voltage_bit(error)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only scan for flexible-hand motors"
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help="Serial port",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="Serial baudrate",
    )
    parser.add_argument(
        "--start-id",
        type=int,
        default=1,
        help="First motor ID",
    )
    parser.add_argument(
        "--end-id",
        type=int,
        default=17,
        help="Last motor ID",
    )
    parser.add_argument(
        "--voltage-min",
        type=float,
        default=None,
        help="Optional expected minimum voltage",
    )
    parser.add_argument(
        "--voltage-max",
        type=float,
        default=None,
        help="Optional expected maximum voltage",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if not 1 <= args.start_id <= args.end_id <= 253:
        print(
            "[FAIL] Require 1 <= start-id <= end-id <= 253"
        )
        return 2

    if (args.voltage_min is None) != (args.voltage_max is None):
        print(
            "[FAIL] --voltage-min and --voltage-max "
            "must be provided together"
        )
        return 2

    if (
        args.voltage_min is not None
        and args.voltage_min > args.voltage_max
    ):
        print("[FAIL] voltage-min cannot exceed voltage-max")
        return 2

    print("=" * 100)
    print("  Flexible Hand -- Read-only Motor Scan")
    print("=" * 100)
    print(
        f"  Port: {args.port}  |  "
        f"Baudrate: {args.baudrate}"
    )
    print(
        "  No torque enable and no movement command "
        "will be sent."
    )
    print()

    ph = PortHandler(args.port)
    ph.baudrate = args.baudrate

    try:
        try:
            if not ph.openPort():
                print(f"[FAIL] Cannot open port {args.port}")
                return 1
        except Exception as exc:
            print(
                f"[FAIL] Cannot open port {args.port}: {exc}"
            )
            print(
                "Check whether another Python process or servo tool "
                "is already using this COM port."
            )
            return 1

        print("[OK] Port opened")
        print()

        pk = sms_sts(ph)

        header = (
            f"{'ID':>3} | "
            f"{'Name':<12} | "
            f"{'Ping':<4} | "
            f"{'Model':>6} | "
            f"{'Pos':>7} | "
            f"{'Volt':>7} | "
            f"{'Temp':>6} | "
            f"{'Cur':>7} | "
            f"Status"
        )
        print(header)
        print("-" * len(header))

        found = 0
        voltage_warnings = []

        for motor_id in range(
            args.start_id,
            args.end_id + 1,
        ):
            name_en = MOTOR_INFO.get(
                motor_id,
                (f"motor_{motor_id}", ""),
            )[0]

            status_parts = []
            voltage_bit_seen = False

            # Ping
            model, ping_result, ping_error = pk.ping(motor_id)
            voltage_bit_seen = note_voltage_bit(
                ping_error,
                voltage_bit_seen,
            )

            if ping_result != COMM_SUCCESS:
                ping_s = "NO"
                model_s = "N/A"
                pos_s = "N/A"
                volt_s = "N/A"
                temp_s = "N/A"
                cur_s = "N/A"
                status_parts.append(
                    format_comm_result(ping_result)
                )
            elif is_real_error(ping_error):
                # 收到数据包，但电机报告了非电压状态位。
                ping_s = "YES"
                model_s = str(model) if model > 0 else "N/A"
                pos_s = "N/A"
                volt_s = "N/A"
                temp_s = "N/A"
                cur_s = "N/A"
                status_parts.append(
                    f"PING_ERR=0x{ping_error:X}"
                )
                found += 1
            else:
                found += 1
                ping_s = "YES"
                model_s = str(model) if model > 0 else "N/A"

                # Position
                pos_raw, pos_result, pos_error = (
                    pk.read2ByteTxRx(
                        motor_id,
                        SMS_STS_PRESENT_POSITION_L,
                    )
                )
                voltage_bit_seen = note_voltage_bit(
                    pos_error,
                    voltage_bit_seen,
                )
                pos_status = read_status_text(
                    pos_result,
                    pos_error,
                    "POS",
                )
                if response_ok(pos_result, pos_error):
                    pos = pk.scs_tohost(pos_raw, 15)
                    pos_s = str(pos)
                else:
                    pos_s = "N/A"
                    if pos_status:
                        status_parts.append(pos_status)

                # Voltage
                volt_raw, volt_result, volt_error = (
                    pk.read1ByteTxRx(
                        motor_id,
                        SMS_STS_PRESENT_VOLTAGE,
                    )
                )
                voltage_bit_seen = note_voltage_bit(
                    volt_error,
                    voltage_bit_seen,
                )
                volt_status = read_status_text(
                    volt_result,
                    volt_error,
                    "VOLT",
                )
                if response_ok(volt_result, volt_error):
                    voltage = volt_raw / 10.0
                    volt_s = f"{voltage:.1f}V"

                    if (
                        args.voltage_min is not None
                        and not (
                            args.voltage_min
                            <= voltage
                            <= args.voltage_max
                        )
                    ):
                        voltage_warnings.append(
                            (motor_id, voltage)
                        )
                else:
                    volt_s = "N/A"
                    if volt_status:
                        status_parts.append(volt_status)

                # Temperature
                temp, temp_result, temp_error = (
                    pk.read1ByteTxRx(
                        motor_id,
                        SMS_STS_PRESENT_TEMPERATURE,
                    )
                )
                voltage_bit_seen = note_voltage_bit(
                    temp_error,
                    voltage_bit_seen,
                )
                temp_status = read_status_text(
                    temp_result,
                    temp_error,
                    "TEMP",
                )
                if response_ok(temp_result, temp_error):
                    temp_s = f"{temp}C"
                else:
                    temp_s = "N/A"
                    if temp_status:
                        status_parts.append(temp_status)

                # Current
                cur_raw, cur_result, cur_error = (
                    pk.read2ByteTxRx(
                        motor_id,
                        SMS_STS_PRESENT_CURRENT_L,
                    )
                )
                voltage_bit_seen = note_voltage_bit(
                    cur_error,
                    voltage_bit_seen,
                )
                cur_status = read_status_text(
                    cur_result,
                    cur_error,
                    "CUR",
                )
                if response_ok(cur_result, cur_error):
                    current = pk.scs_tohost(cur_raw, 15)
                    cur_s = str(current)
                else:
                    cur_s = "N/A"
                    if cur_status:
                        status_parts.append(cur_status)

            if not status_parts:
                status = (
                    "OK(VBIT)"
                    if voltage_bit_seen
                    else "OK"
                )
            else:
                if voltage_bit_seen:
                    status_parts.append("VBIT")
                status = ",".join(status_parts)

            print(
                f"{motor_id:3d} | "
                f"{name_en:<12} | "
                f"{ping_s:<4} | "
                f"{model_s:>6} | "
                f"{pos_s:>7} | "
                f"{volt_s:>7} | "
                f"{temp_s:>6} | "
                f"{cur_s:>7} | "
                f"{status}"
            )

        expected_count = args.end_id - args.start_id + 1

        print()
        print("=" * 100)
        print(
            f"  Responded: {found}/{expected_count}"
        )

        if (
            args.voltage_min is not None
            and args.voltage_max is not None
        ):
            print(
                "  Expected voltage range: "
                f"{args.voltage_min:.1f}~"
                f"{args.voltage_max:.1f}V"
            )
            if voltage_warnings:
                print("  [WARN] Voltage outside configured range:")
                for motor_id, voltage in voltage_warnings:
                    print(
                        f"    Motor {motor_id:2d}: "
                        f"{voltage:.1f}V"
                    )
            else:
                print(
                    "  All readable voltages are within "
                    "the configured range"
                )
        else:
            print(
                "  Voltage is displayed only; no range "
                "judgment was applied."
            )

        print()
        print(
            "  NOTE: VBIT means ERRBIT_VOLTAGE was returned "
            "and ignored only for this project configuration."
        )
        print(
            "  Other status bits and communication failures "
            "remain visible as errors."
        )

        return 0 if found > 0 else 1

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130
    finally:
        if getattr(ph, "is_using", False):
            ph.is_using = False

        try:
            ph.closePort()
        except Exception:
            pass

        print("  Port closed")


if __name__ == "__main__":
    raise SystemExit(main())
