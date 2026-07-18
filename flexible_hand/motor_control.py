"""
motor_control.py --- 单电机控制层

基于 Feetech STS3215 底层协议，封装单电机控制接口。
所有位置值使用原始单位 (0~4095 = 1圈)，不使用弧度转换。

功能：
  - connect / disconnect
  - set_torque
  - set_position (单电机)
  - read_position / read_voltage / read_temperature
  - move_jog (正转/反转/步进)

用法示例：
  from motor_control import MotorControl
  mc = MotorControl("COM5")
  mc.connect()
  mc.set_position(1, 2048)       # Motor 1 转到中位
  mc.move_jog(1, 500, 30)        # Motor 1 正转 500 步
  mc.move_jog(1, -300, 30)       # Motor 1 反转 300 步
  pos = mc.read_position(1)      # 读取当前位置
  mc.disconnect()
"""

from __future__ import annotations

import sys
import time
import logging
from pathlib import Path
from typing import Optional, Tuple

# -- 添加 feetech 库路径 --
HARDWARE_DIR = (
    Path(__file__).resolve().parents[1]
    / "third_party" / "orca_core" / "orca_core" / "hardware"
)
if str(HARDWARE_DIR) not in sys.path:
    sys.path.insert(0, str(HARDWARE_DIR))

MOTOR_ID_MIN = 1
MOTOR_ID_MAX = 17

from feetech import (
    PortHandler,
    sms_sts,
    COMM_SUCCESS,
    COMM_RX_TIMEOUT,
    COMM_PORT_BUSY,
    SMS_STS_TORQUE_ENABLE,
    SMS_STS_ACC,
    SMS_STS_GOAL_POSITION_L,
    SMS_STS_GOAL_SPEED_L,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_VOLTAGE,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_SPEED_L,
    SMS_STS_MODE,
)
from feetech.protocol_packet_handler import ERRBIT_VOLTAGE

logger = logging.getLogger("motor_control")

# -- STS3215 常量 --
POS_MIN = 0
POS_MAX = 4095
POS_MID = 2048  # 约 180 度位置

# 默认运动参数
DEFAULT_SPEED = 60    # 0.732 RPM/unit, 约 44 RPM
DEFAULT_ACC = 50      # 加速度 0~254
DEFAULT_TORQUE = 500  # 扭矩限制 0~1000


class MotorControl:
    """单电机控制接口

    特点：
    - WritePosEx 可同时控制位置+速度+加速度+扭矩
    - 支持正转/反转的 jog 模式
    - 所有位置用 0~4095 原始单位
    """

    def __init__(self, port: str = "COM5", baudrate: int = 1000000):
        self.port_name = port
        self.baudrate = baudrate
        self._ph: Optional[PortHandler] = None
        self._pk: Optional[sms_sts] = None
        self._connected = False

    # ── 连接管理 ────────────────────────────────────────────────────

    def connect(self) -> bool:
        """打开串口并初始化协议处理器"""
        if self._connected:
            logger.warning("Already connected")
            return True

        self._ph = PortHandler(self.port_name)
        self._ph.baudrate = self.baudrate

        if not self._ph.openPort():
            logger.error("Failed to open port %s", self.port_name)
            return False

        self._pk = sms_sts(self._ph)
        self._connected = True
        logger.info("Connected to %s at %d baud", self.port_name, self.baudrate)
        return True

    def disconnect(self):
        """关闭串口"""
        if self._connected and self._ph:
            self._ph.closePort()
            self._connected = False
            logger.info("Disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ── 错误处理 ────────────────────────────────────────────────────

    @staticmethod
    def _is_real_error(error_byte: int) -> bool:
        """判断 error byte 是否需要上报

        STS3215 在 12V 供电下 ERRBIT_VOLTAGE(bit0) 恒为 1，
        这是已知兼容问题，忽略。过热(bit2)、过流(bit3)、
        过载(bit5) 等真实异常照常上报。
        """
        if error_byte == 0:
            return False
        # 只保留非 ERRBIT_VOLTAGE 的 bit
        return (error_byte & ~ERRBIT_VOLTAGE) != 0

    # ── 状态检查 ────────────────────────────────────────────────────

    def _check(self):
        if not self._connected or not self._pk:
            raise RuntimeError("Not connected. Call connect() first.")

    # ── 扭矩控制 ────────────────────────────────────────────────────

    def set_torque(self, motor_id: int, enable: bool) -> Tuple[bool, str]:
        """启用/禁用扭矩

        Args:
            motor_id: 电机 ID (1~17)
            enable: True=启用, False=禁用

        Returns:
            (success, message)
        """
        self._check()
        result, error = self._pk.write1ByteTxRx(motor_id, SMS_STS_TORQUE_ENABLE, int(enable))
        if result != COMM_SUCCESS:
            return False, "comm_fail=%d" % result
        if self._is_real_error(error):
            return False, "err=0x%X" % error
        return True, "ok"

    # ── 位置控制 ────────────────────────────────────────────────────

    def set_position(self,
        motor_id: int,
        position: int,
        speed: Optional[int] = None,
        acc: Optional[int] = None,
        torque: Optional[int] = None,
        wait: bool = False,
        timeout: float = 3.0,
    ) -> Tuple[bool, str]:
        """控制单电机转动到指定位置

        Args:
            motor_id: 电机 ID (1~17)
            position: 目标位置 (0~4095)
            speed: 速度 (0.732 RPM/unit), 默认 60
            acc: 加速度 0~254, 默认 50
            torque: 扭矩限制 0~1000, 默认 500
            wait: 是否等待运动完成
            timeout: 等待超时 (秒)

        Returns:
            (success, message)
        """
        self._check()

        # 限制位置范围
        position = max(POS_MIN, min(POS_MAX, position))

        speed = speed if speed is not None else DEFAULT_SPEED
        acc = acc if acc is not None else DEFAULT_ACC
        torque = torque if torque is not None else DEFAULT_TORQUE

        result, error = self._pk.WritePosEx(motor_id, position, speed, acc, torque)
        if result != COMM_SUCCESS:
            return False, "write_fail=%d" % result
        if self._is_real_error(error):
            return False, "err=0x%X" % error

        if wait:
            return self._wait_for_stop(motor_id, timeout)

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
        """正转/反转相对步进

        Args:
            motor_id: 电机 ID
            delta: 步进量 (正=正转, 负=反转), 单位: 原始位置单位
            speed: 速度
            acc: 加速度
            torque: 扭矩限制
            wait: 是否等待运动完成
            timeout: 超时

        Returns:
            (success, message, new_position)
        """
        self._check()

        # 读取当前位置
        current, result, error = self._pk.ReadPos(motor_id)
        if result != COMM_SUCCESS:
            return False, "read_pos_fail=%d" % result, 0

        # 计算新位置并限制范围
        new_pos = max(POS_MIN, min(POS_MAX, current + delta))

        speed = speed if speed is not None else DEFAULT_SPEED
        acc = acc if acc is not None else DEFAULT_ACC
        torque = torque if torque is not None else DEFAULT_TORQUE

        result, error = self._pk.WritePosEx(motor_id, new_pos, speed, acc, torque)
        if result != COMM_SUCCESS:
            return False, "write_fail=%d" % result, current
        if self._is_real_error(error):
            return False, "err=0x%X" % error, current

        if wait:
            ok, msg = self._wait_for_stop(motor_id, timeout)
            if not ok:
                return False, msg, new_pos

        return True, "ok", new_pos

    # ── 读取 ────────────────────────────────────────────────────────

    def read_position(self, motor_id: int) -> Tuple[int, bool, str]:
        """读取当前位置

        Returns:
            (position_raw, success, message)
        """
        self._check()
        pos, result, error = self._pk.ReadPos(motor_id)
        if result != COMM_SUCCESS:
            return 0, False, "read_fail=%d" % result
        return pos, True, "ok"

    def read_voltage(self, motor_id: int) -> Tuple[float, bool, str]:
        """读取当前电压 (V)

        Note: error=1 (ERRBIT_VOLTAGE) 不影响数据读取，数据本身有效。
        只有非 ERRBIT_VOLTAGE 的异常才返回失败。

        Returns:
            (voltage_in_volts, success, message)
        """
        self._check()
        raw, result, error = self._pk.read1ByteTxRx(motor_id, SMS_STS_PRESENT_VOLTAGE)
        if result != COMM_SUCCESS:
            return 0.0, False, "comm_fail=%d" % result
        if self._is_real_error(error):
            return 0.0, False, "err=0x%X" % error
        return raw / 10.0, True, "ok"

    def read_temperature(self, motor_id: int) -> Tuple[int, bool, str]:
        """读取当前温度 (C)

        Note: error=1 (ERRBIT_VOLTAGE) 不影响数据读取，数据本身有效。

        Returns:
            (temperature_C, success, message)
        """
        self._check()
        temp, result, error = self._pk.read1ByteTxRx(motor_id, SMS_STS_PRESENT_TEMPERATURE)
        if result != COMM_SUCCESS:
            return 0, False, "comm_fail=%d" % result
        if self._is_real_error(error):
            return 0, False, "err=0x%X" % error
        return temp, True, "ok"

    # ── 内部辅助 ────────────────────────────────────────────────────

    def _wait_for_stop(self, motor_id: int, timeout: float = 3.0) -> Tuple[bool, str]:
        """等待电机停止运动（通过位置判稳，避免 Moving 标志受 error=1 干扰）"""
        start = time.time()
        stable_count = 0
        last_pos = -999

        while time.time() - start < timeout:
            pos, ok, _ = self.read_position(motor_id)
            if not ok:
                time.sleep(0.05)
                continue

            if pos == last_pos:
                stable_count += 1
            else:
                stable_count = 0
                last_pos = pos

            # 连续 5 次读取位置不变(约 250ms) = 停止
            if stable_count >= 5:
                return True, "done"

            time.sleep(0.05)

        return False, "timeout"

    def print_info(self, motor_id: int):
        """打印单个电机的完整状态"""
        pos, ok, msg = self.read_position(motor_id)
        pos_str = "%d" % pos if ok else msg

        volt, ok, msg = self.read_voltage(motor_id)
        volt_str = "%.1fV" % volt if ok else msg

        temp, ok, msg = self.read_temperature(motor_id)
        temp_str = "%dC" % temp if ok else msg

        print("  Motor %2d: pos=%s  volt=%s  temp=%s" % (motor_id, pos_str, volt_str, temp_str))


# ── 单元测试 / 演示 ──────────────────────────────────────────────────

def quick_test():
    """快速测试：遍历 1~17 号电机，每个正转 200 步再反转回来"""
    import time

    print("=" * 60)
    print("Quick Test: jog each motor +200, then -200")
    print("=" * 60)

    with MotorControl("COM5") as mc:
        for mid in range(1, 18):
            print("\n--- Motor %d ---" % mid)
            mc.print_info(mid)

            # 正转 200
            ok, msg, new_pos = mc.move_jog(mid, 200, speed=40, wait=True)
            print("  +200 -> %s [%s]" % (new_pos, "OK" if ok else msg))

            if not ok:
                continue

            time.sleep(0.3)

            # 反转 200 (回到原位附近)
            ok, msg, new_pos = mc.move_jog(mid, -200, speed=40, wait=True)
            print("  -200 -> %s [%s]" % (new_pos, "OK" if ok else msg))

            time.sleep(0.3)

    print("\nDone!")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    import argparse

    parser = argparse.ArgumentParser(description="Single motor control")
    parser.add_argument("action", nargs="?", default="test",
                        choices=["test", "pos", "jog", "info", "scan"],
                        help="Action to perform")
    parser.add_argument("--id", type=int, default=1, help="Motor ID (1~17)")
    parser.add_argument("--pos", type=int, default=2048, help="Target position (0~4095)")
    parser.add_argument("--delta", type=int, default=200, help="Jog step size")
    parser.add_argument("--speed", type=int, default=60, help="Speed")
    parser.add_argument("--acc", type=int, default=50, help="Acceleration")
    parser.add_argument("--torque", type=int, default=500, help="Torque limit")
    parser.add_argument("--port", type=str, default="COM5", help="Serial port")
    parser.add_argument("--wait", action="store_true", default=True,
                        help="Wait for motion to complete")

    args = parser.parse_args()

    mc = MotorControl(args.port)
    if not mc.connect():
        sys.exit(1)

    if args.action == "test":
        quick_test()
    elif args.action == "info":
        mc.print_info(args.id)
    elif args.action == "pos":
        ok, msg = mc.set_position(args.id, args.pos, args.speed, args.acc, args.torque, args.wait)
        print("[%s] Motor %d -> %d: %s" % ("OK" if ok else "FAIL", args.id, args.pos, msg))
        mc.print_info(args.id)
    elif args.action == "jog":
        ok, msg, new_pos = mc.move_jog(args.id, args.delta, args.speed, args.acc, args.torque, args.wait)
        print("[%s] Motor %d jog %d -> %d: %s" % ("OK" if ok else "FAIL", args.id, args.delta, new_pos, msg))
        mc.print_info(args.id)
    elif args.action == "scan":
        print("ID  | Position | Voltage | Temp")
        print("-" * 40)
        for mid in range(1, 18):
            pos, pok, pmsg = mc.read_position(mid)
            v, vok, vmsg = mc.read_voltage(mid)
            t, tok, tmsg = mc.read_temperature(mid)
            pos_s = "%d" % pos if pok else pmsg
            v_s = "%.1fV" % v if vok else vmsg
            t_s = "%dC" % t if tok else tmsg
            print("%3d | %8s | %6s | %s" % (mid, pos_s, v_s, t_s))

    mc.disconnect()
