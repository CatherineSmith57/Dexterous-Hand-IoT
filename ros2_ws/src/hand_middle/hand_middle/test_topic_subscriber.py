"""
test_topic_subscriber.py — hand_middle 话题订阅测试节点

功能：
1. 订阅 /hand_middle/status 话题 (HandStatus.msg, 10Hz)
2. 实时打印手部完整状态：
   - 连接/标定/使能状态
   - 10个关节的位置/温度/力矩
   - 故障码与故障描述
   - 当前执行动作与状态
3. 支持统计模式 (--ros-args -p stats_mode:=true)
   - 不逐条打印，每秒输出一次汇总统计
4. 自动检测连接断开、故障触发等异常状态并告警

使用方式:
    # 终端1: 启动服务端
    ros2 launch hand_middle hand_middle.launch.py

    # 终端2: 订阅状态话题（详细模式）
    ros2 run hand_middle test_topic_subscriber

    # 统计模式（每秒汇总，减少终端输出）
    ros2 run hand_middle test_topic_subscriber --ros-args -p stats_mode:=true

    # 仅打印故障信息（过滤正常状态）
    ros2 run hand_middle test_topic_subscriber --ros-args -p fault_only:=true
"""

import sys
import time
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ── 自定义Topic消息 ────────────────────────────────────────
try:
    from hand_middle_interfaces.msg import HandStatus
    _MSG_AVAILABLE = True
except ModuleNotFoundError:
    HandStatus = None  # type: ignore
    _MSG_AVAILABLE = False

logger = logging.getLogger(__name__)


class TestTopicSubscriber(Node):
    """
    hand_middle 话题订阅测试节点。

    订阅 /hand_middle/status，实时打印手部状态。
    支持详细模式、统计模式、故障过滤模式。
    """

    def __init__(self):
        super().__init__("test_topic_subscriber")

        # ── 声明参数 ─────────────────────────────────────────
        self.declare_parameter("stats_mode", False)
        self.declare_parameter("fault_only", False)
        self.declare_parameter("topic_name", "/hand_middle/status")
        self.declare_parameter("print_interval_sec", 1.0)

        self._stats_mode = (
            self.get_parameter("stats_mode")
            .get_parameter_value()
            .bool_value
        )
        self._fault_only = (
            self.get_parameter("fault_only")
            .get_parameter_value()
            .bool_value
        )
        self._topic_name = (
            self.get_parameter("topic_name")
            .get_parameter_value()
            .string_value
        )
        self._print_interval = (
            self.get_parameter("print_interval_sec")
            .get_parameter_value()
            .double_value
        )

        # ── 校验Message是否已编译 ────────────────────────────
        if not _MSG_AVAILABLE or HandStatus is None:
            self.get_logger().fatal(
                "HandStatus.msg NOT compiled! "
                "Run 'colcon build' before using this subscriber."
            )
            sys.exit(1)

        # ── 统计计数器 ────────────────────────────────────────
        self._msg_count = 0
        self._last_stats_time = time.time()
        self._fault_count = 0
        self._last_fault_code = 0

        # ── 创建订阅者 ────────────────────────────────────────
        # QoS: Best Effort 匹配发布端设置
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._subscription = self.create_subscription(
            HandStatus,
            self._topic_name,
            self._status_callback,
            qos,
        )

        mode_desc = (
            "fault-only" if self._fault_only
            else "stats (1Hz summary)" if self._stats_mode
            else "verbose (every message)"
        )
        self.get_logger().info(
            f"Subscribed to '{self._topic_name}' — mode: {mode_desc}"
        )

    # ── 状态回调 ─────────────────────────────────────────────

    def _status_callback(self, msg: HandStatus) -> None:
        """
        接收 HandStatus 消息并打印。

        根据模式选择输出方式：
        - fault_only: 仅在 fault_code != 0 时打印
        - stats_mode: 累积计数，每秒汇总
        - verbose: 每条消息都格式化打印
        """
        self._msg_count += 1

        # ── 故障检测与告警 ───────────────────────────────────
        if msg.fault_code != 0:
            self._fault_count += 1
            if msg.fault_code != self._last_fault_code:
                self.get_logger().warn(
                    f"⚠ FAULT DETECTED! code={msg.fault_code}, "
                    f"message='{msg.fault_message}'"
                )
                self._last_fault_code = msg.fault_code

        # ── 故障过滤模式 ─────────────────────────────────────
        if self._fault_only:
            if msg.fault_code != 0:
                # 故障时强制打印
                self._print_detailed_status(msg)
            return

        # ── 统计模式 ─────────────────────────────────────────
        if self._stats_mode:
            elapsed = time.time() - self._last_stats_time
            if elapsed >= self._print_interval:
                self._print_stats_summary(elapsed)
                self._last_stats_time = time.time()
                self._msg_count = 0
                self._fault_count = 0
            return

        # ── 详细模式 ─────────────────────────────────────────
        self._print_detailed_status(msg)

    # ── 打印方法 ─────────────────────────────────────────────

    def _print_detailed_status(self, msg: HandStatus):
        """
        格式化打印单条 HandStatus 消息的完整内容。

        Parameters
        ----------
        msg : HandStatus
            手部状态消息
        """
        # 从Header提取时间戳
        ts = msg.header.stamp
        ts_str = f"{ts.sec}.{ts.nanosec:09d}"

        # 状态图标
        conn_icon = "🟢" if msg.connected else "🔴"
        cal_icon = "✅" if msg.calibrated else "⬜"
        motor_icon = "⚡" if msg.motor_enabled else "🔒"
        fault_icon = "⚠" if msg.fault_code != 0 else "✓"

        lines = [
            f"\n{'─'*70}",
            f"  Hand Status @ {ts_str}",
            f"  {conn_icon} connected={msg.connected}  "
            f"{cal_icon} calibrated={msg.calibrated}  "
            f"{motor_icon} motor_enabled={msg.motor_enabled}",
            f"  {fault_icon} fault_code={msg.fault_code}  "
            f"action={msg.current_action}/{msg.execution_status}",
            f"  {'─'*70}",
        ]

        # ── 关节数据表格 ─────────────────────────────────────
        header = (
            f"  {'Joint':<16s} {'Pos(°)':>8s} {'Temp(°C)':>10s} {'Torque(N·m)':>12s}"
        )
        lines.append(header)
        lines.append(f"  {'─'*16} {'─'*8} {'─'*10} {'─'*12}")

        count = len(msg.joint_names)
        for i in range(count):
            name = msg.joint_names[i]
            pos = msg.joint_positions[i] if i < len(msg.joint_positions) else 0.0
            temp = msg.joint_temperatures[i] if i < len(msg.joint_temperatures) else 0.0
            torque = msg.joint_torques[i] if i < len(msg.joint_torques) else 0.0

            # 温度/力矩越限高亮
            temp_flag = " 🔥" if temp > 70.0 else ""
            torque_flag = " ⚡" if torque > 2.0 else ""

            lines.append(
                f"  {name:<16s} {pos:>+8.1f} {temp:>9.1f}{temp_flag} {torque:>11.3f}{torque_flag}"
            )

        # ── 故障信息 ─────────────────────────────────────────
        if msg.fault_message:
            lines.append(f"  {'─'*70}")
            lines.append(f"  ⚠ FAULT: [{msg.fault_code}] {msg.fault_message}")

        lines.append(f"{'─'*70}")

        print("\n".join(lines))

    def _print_stats_summary(self, elapsed: float):
        """
        打印统计汇总（统计模式用）。

        Parameters
        ----------
        elapsed : float
            距离上次打印的秒数
        """
        actual_hz = self._msg_count / elapsed if elapsed > 0 else 0.0

        status_line = (
            f"[STATS] {elapsed:.1f}s: "
            f"{self._msg_count} msgs received, "
            f"{actual_hz:.1f} Hz actual, "
        )

        if self._fault_count > 0:
            status_line += f"⚠ {self._fault_count} FAULT(S) detected!"
        else:
            status_line += "no faults"

        self.get_logger().info(status_line)


# ── 入口点 ──────────────────────────────────────────────────

def main(args=None):
    """ROS2话题订阅测试节点入口点 (console_scripts)。"""
    rclpy.init(args=args)

    node = None
    try:
        node = TestTopicSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("KeyboardInterrupt received — subscriber stopped")
    except Exception as e:
        logger.exception(f"TestTopicSubscriber crashed: {e}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
