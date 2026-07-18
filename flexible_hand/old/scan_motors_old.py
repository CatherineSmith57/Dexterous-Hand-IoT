"""
scan_motors.py --- 只读扫描 1~17 号电机

功能：
  1. 依次 ping 1~17 号电机
  2. 读取每个电机的当前位置、电压、温度
  3. 显示结果表格

用法：
  python flexible_hand/scan_motors.py
"""

import sys
from pathlib import Path

HARDWARE_DIR = (
    Path(__file__).resolve().parents[1]
    / "third_party" / "orca_core" / "orca_core" / "hardware"
)
if str(HARDWARE_DIR) not in sys.path:
    sys.path.insert(0, str(HARDWARE_DIR))

from feetech import (
    PortHandler,
    sms_sts,
    COMM_SUCCESS,
    COMM_RX_TIMEOUT,
    COMM_PORT_BUSY,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_VOLTAGE,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_MODEL_L,
)

MOTOR_INFO = {
    1:  ("wrist",        "手腕屈伸"),
    2:  ("index_pip",    "食指近端"),
    3:  ("index_mcp",    "食指掌指"),
    4:  ("index_abd",    "食指外展"),
    5:  ("middle_abd",   "中指外展"),
    6:  ("ring_abd",     "无名指外展"),
    7:  ("ring_mcp",     "无名指掌指"),
    8:  ("ring_pip",     "无名指近端"),
    9:  ("middle_pip",   "中指近端"),
    10: ("middle_mcp",   "中指掌指"),
    11: ("pinky_pip",    "小指近端"),
    12: ("pinky_mcp",    "小指掌指"),
    13: ("pinky_abd",    "小指外展"),
    14: ("thumb_abd",    "拇指外展"),
    15: ("thumb_mcp",    "拇指掌指"),
    16: ("thumb_dip",    "拇指远端"),
    17: ("thumb_cmc",    "拇指根部"),
}


def main():
    PORT = "COM5"
    BAUDRATE = 1000000

    print("=" * 80)
    print("  ORCA Hand v2 Right -- Motor Scan")
    print("=" * 80)
    print("  Port: %s  |  Baudrate: %d" % (PORT, BAUDRATE))
    print()

    # 1. Open port
    ph = PortHandler(PORT)
    ph.baudrate = BAUDRATE
    if not ph.openPort():
        print("[FAIL] Cannot open port %s" % PORT)
        return 1
    print("[OK] Port opened")
    print()

    # 2. Create protocol handler
    pk = sms_sts(ph)

    # 3. Scan
    hdr = "%3s | %-12s | %-4s | %6s | %6s | %6s | %4s | %6s | %s" % (
        "ID", "Name", "Ping", "Model", "Pos", "Volt", "Temp", "Cur", "Status")
    print(hdr)
    print("-" * len(hdr))

    found = 0
    voltage_warn = []

    for mid in range(1, 18):
        name_en = MOTOR_INFO[mid][0]

        # Ping
        model, pr, pe = pk.ping(mid)
        ok = (pr == COMM_SUCCESS)

        if ok:
            found += 1
            ping_s = "YES"

            # Model
            model_s = "%6d" % model if model > 0 else "  N/A"

            # Position -- read2ByteTxRx returns (value, result, error) directly
            pos_raw, _, pe2 = pk.read2ByteTxRx(mid, SMS_STS_PRESENT_POSITION_L)
            if pr == COMM_SUCCESS:
                pos = pk.scs_tohost(pos_raw, 15)
                pos_s = "%6d" % pos
            else:
                pos_s = "  N/A"

            # Voltage -- read1ByteTxRx returns (value, result, error) directly
            volt_raw, _, ve = pk.read1ByteTxRx(mid, SMS_STS_PRESENT_VOLTAGE)
            if ve == 0:
                v = volt_raw / 10.0
                volt_s = "%5.1fV" % v
                if v < 6.0 or v > 8.4:
                    voltage_warn.append((mid, v))
            else:
                volt_s = "err=%d" % ve

            # Temperature
            temp, _, te = pk.read1ByteTxRx(mid, SMS_STS_PRESENT_TEMPERATURE)
            temp_s = "%3dC" % temp if temp > 0 and te == 0 else " N/A"

            # Current
            cur_raw, _, ce = pk.read2ByteTxRx(mid, SMS_STS_PRESENT_CURRENT_L)
            if ce == 0:
                cur = pk.scs_tohost(cur_raw, 15)
                cur_s = "%6d" % cur if cur != 0 else "     0"
            else:
                cur_s = "  N/A"

            status = "OK" if pe == 0 else "err=0x%X" % pe
        else:
            ping_s = "NO"
            model_s = "  N/A"
            pos_s = "  N/A"
            volt_s = "  N/A"
            temp_s = " N/A"
            cur_s = "  N/A"
            if pr == COMM_RX_TIMEOUT:
                status = "TIMEOUT"
            elif pr == COMM_PORT_BUSY:
                status = "BUSY"
            else:
                status = "comm=%d" % pr

        line = "%3d | %-12s | %-4s | %6s | %6s | %6s | %4s | %6s | %s" % (
            mid, name_en, ping_s, model_s, pos_s, volt_s, temp_s, cur_s, status)
        print(line)

    # Summary
    print()
    print("=" * 80)
    print("  Responded: %d/17" % found)
    if voltage_warn:
        print("  [WARN] Voltage anomalies:")
        for mid, v in voltage_warn:
            print("    Motor %2d: %.1fV (rated 6.0~8.4V)" % (mid, v))
    else:
        print("  All voltages in range")
    print()
    print("  NOTE: STS3215 ping error=1 = ERRBIT_VOLTAGE (bit 0)")
    print("  This is a status flag, not an error for Feetech STS series.")

    ph.closePort()
    print("  Port closed")
    return 0 if found > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
