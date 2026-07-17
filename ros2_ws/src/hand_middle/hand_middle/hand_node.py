"""
hand_node.py — ORCA灵巧手ROS2主控节点

功能：
1. 启动时初始化HandBridge（连接+标定+使能电机）
2. 暴露 /hand_middle/command 服务（HandCommand.srv）
   - 接收上层控制指令：关节控制、手势执行、硬件复位、电机使能/去使能
   - 封装调用orca_core串口硬件接口执行动作
   - 返回标准execution_result（success/status/error_code/error_message/timestamp）
3. 10Hz定时器定时读取硬件实时状态，通过 /hand_middle/status 话题向外发布
   - 包含：连接/标定/使能状态、各关节位置/温度/力矩、故障码/故障描述、当前动作
4. 分级异常容错：
   - DEBUG：内部状态细节（传感器模拟更新、插值计算）
   - INFO：关键流程里程碑（连接/标定/指令执行开始与完成）
   - WARN：可恢复异常（状态读取偶发失败、非关键资源释放失败）
   - ERROR：需人工介入（串口断开、参数越界、执行失败）
   - FATAL：致命错误（设备完全不可用）

完整业务闭环：
   上层下发控制指令 → /hand_middle/command 服务接收
   → HandBridge封装调用orca_core串口硬件接口执行动作
   → 10Hz定时器读取硬件实时状态 → /hand_middle/status 话题向外发布

启动方式:
    ros2 run hand_middle hand_node
    ros2 run hand_middle hand_node --ros-args -p port:=/dev/ttyUSB0
    ros2 launch hand_middle hand_middle.launch.py
"""

import sys
import logging
import traceback
from datetime import datetime, timezone, timedelta

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ── 自定义Service/Message ──────────────────────────────────
# 需要先 colcon build 编译后才会生成Python模块
# 此处使用 try/except 兼容未编译环境，给出清晰错误提示
try:
    from hand_middle_interfaces.srv import HandCommand
    _SRV_AVAILABLE = True
except ModuleNotFoundError:
    HandCommand = None  # type: ignore
    _SRV_AVAILABLE = False

try:
    from hand_middle_interfaces.msg import HandStatus
    _MSG_AVAILABLE = True
except ModuleNotFoundError:
    HandStatus = None  # type: ignore
    _MSG_AVAILABLE = False

from .hand_bridge import HandBridge

logger = logging.getLogger(__name__)

# ── 北京时间时区 ───────────────────────────────────────────
_CST = timezone(timedelta(hours=8))

# ── 固定关节顺序（与 HandStatus.msg / hand_bridge.JOINT_ORDER 一致）──
JOINT_ORDER = [
    "thumb_mcp", "thumb_pip",
    "index_mcp", "index_pip",
    "middle_mcp", "middle_pip",
    "ring_mcp", "ring_pip",
    "pinky_mcp", "pinky_pip",
]


class HandNode(Node):
    """
    ORCA灵巧手ROS2主控节点。

    对外Service:
        /hand_middle/command (HandCommand.srv)
            支持指令: joint_control | gesture | reset | enable | disable

    对外Topic:
        /hand_middle/status (HandStatus.msg, 10Hz)
            包含: 连接/标定/使能状态、关节位置/温度/力矩、故障标记

    ROS2参数:
        port                   — 串口号 (默认 /dev/ttyUSB0)
        baudrate               — 波特率 (默认 115200)
        status_publish_rate    — 状态发布频率Hz (默认 10.0)
        auto_initialize        — 启动时自动连接标定 (默认 true)
        temperature_limit      — 过温阈值°C (默认 75.0)
        torque_limit           — 过力矩阈值N·m (默认 2.5)
    """

    def __init__(self):
        super().__init__("hand_node")

        # ── 声明ROS2参数 ────────────────────────────────────
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("status_publish_rate", 10.0)
        self.declare_parameter("auto_initialize", True)
        self.declare_parameter("temperature_limit", 75.0)
        self.declare_parameter("torque_limit", 2.5)

        port = (
            self.get_parameter("port")
            .get_parameter_value()
            .string_value
        )
        baudrate = (
            self.get_parameter("baudrate")
            .get_parameter_value()
            .integer_value
        )
        publish_rate = (
            self.get_parameter("status_publish_rate")
            .get_parameter_value()
            .double_value
        )
        auto_init = (
            self.get_parameter("auto_initialize")
            .get_parameter_value()
            .bool_value
        )
        temp_limit = (
            self.get_parameter("temperature_limit")
            .get_parameter_value()
            .double_value
        )
        torque_limit = (
            self.get_parameter("torque_limit")
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(
            f"HandNode starting — "
            f"port={port}, baudrate={baudrate}, "
            f"publish_rate={publish_rate}Hz, "
            f"auto_init={auto_init}, "
            f"temp_limit={temp_limit}°C, torque_limit={torque_limit}N·m"
        )

        # ── 初始化中间适配层 ─────────────────────────────────
        self._bridge = HandBridge(
            port=port,
            baudrate=baudrate,
            temperature_limit=temp_limit,
            torque_limit=torque_limit,
        )

        if auto_init:
            try:
                init_ok = self._bridge.initialize()
                if init_ok:
                    self.get_logger().info(
                        "Device initialized successfully: connected + calibrated + motor enabled"
                    )
                else:
                    self.get_logger().error(
                        "Device initialization FAILED! "
                        "Check serial connection & calibration. "
                        "Service will return error until device is ready."
                    )
            except Exception as e:
                self.get_logger().error(
                    f"Device initialization raised exception: {e}"
                )
                self.get_logger().warn(
                    "Node will continue running without hardware. "
                    "Use 'reset' command to re-initialize when device is available."
                )

        # ── 创建状态发布者 (10Hz定时器) ───────────────────────
        self._status_publisher = None
        self._status_timer = None
        if _MSG_AVAILABLE and HandStatus is not None:
            # QoS: 传感器数据 — Reliable + 适量历史深度
            # 使用 RELIABLE 确保 ros2 topic echo 等调试工具能正常接收
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self._status_publisher = self.create_publisher(
                HandStatus,
                "/hand_middle/status",
                qos,
            )
            self._status_timer = self.create_timer(
                1.0 / publish_rate,
                self._publish_status_callback,
            )
            self.get_logger().info(
                f"Status publisher ready on /hand_middle/status @ {publish_rate}Hz"
            )
        else:
            self.get_logger().warn(
                "HandStatus.msg NOT compiled! "
                "Run 'colcon build' first. "
                "Status topic will NOT be published."
            )

        # ── 创建Service服务器 ────────────────────────────────
        if _SRV_AVAILABLE and HandCommand is not None:
            self._srv = self.create_service(
                HandCommand,
                "/hand_middle/command",
                self._handle_command,
            )
            self.get_logger().info(
                "Service /hand_middle/command ready "
                "(joint_control | gesture | reset | enable | disable)"
            )
        else:
            self.get_logger().warn(
                "HandCommand.srv NOT compiled! "
                "Run 'colcon build' first. "
                "Service will NOT be available."
            )
            self._srv = None

        self.get_logger().info("HandNode started successfully")

    # ── 10Hz状态发布回调 ────────────────────────────────────

    def _publish_status_callback(self) -> None:
        """
        定时器回调：读取设备状态并发布到 /hand_middle/status 话题。

        发布频率由 status_publish_rate 参数控制 (默认10Hz, 即100ms间隔)。

        异常处理：
        - 读取失败 → 发布上一次已知状态并标记 fault_code
        - 发布失败 → 记录ERROR日志但不中断定时器
        """
        if self._status_publisher is None or HandStatus is None:
            return

        try:
            # 从 HandBridge 获取设备扩展状态
            device_status = self._bridge.get_device_status()

            # 组装 HandStatus 消息
            msg = HandStatus()

            # ── 消息头 ─────────────────────────────────────────
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "hand_base_link"

            # ── 设备基础状态 ───────────────────────────────────
            msg.connected = device_status.get("connected", False)
            msg.calibrated = device_status.get("calibrated", False)
            msg.motor_enabled = device_status.get("motor_enabled", False)

            # ── 关节数据 ───────────────────────────────────────
            joint_names = device_status.get("joint_names", JOINT_ORDER)
            joint_positions = device_status.get("joint_positions", [0.0] * 10)
            joint_temperatures = device_status.get("joint_temperatures", [0.0] * 10)
            joint_torques = device_status.get("joint_torques", [0.0] * 10)

            # 动态数组用 append 填充
            for name in joint_names:
                msg.joint_names.append(str(name))
            for pos in joint_positions:
                msg.joint_positions.append(float(pos))
            for temp in joint_temperatures:
                msg.joint_temperatures.append(float(temp))
            for tor in joint_torques:
                msg.joint_torques.append(float(tor))

            # ── 故障标记 ───────────────────────────────────────
            msg.fault_code = device_status.get("fault_code", 0)
            msg.fault_message = device_status.get("fault_message", "")

            # ── 执行状态 ───────────────────────────────────────
            msg.current_action = device_status.get("current_action", "idle")
            msg.execution_status = device_status.get("execution_status", "idle")

            # ── 发布消息 ───────────────────────────────────────
            self._status_publisher.publish(msg)

            # DEBUG级别：每次发布都记录（高频，生产环境可关闭）
            self.get_logger().debug(
                f"Status published: connected={msg.connected}, "
                f"calibrated={msg.calibrated}, "
                f"motor_enabled={msg.motor_enabled}, "
                f"fault_code={msg.fault_code}, "
                f"action={msg.current_action}/{msg.execution_status}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to publish status on /hand_middle/status: {e}"
            )
            self.get_logger().debug(traceback.format_exc())

    # ── Service回调 ─────────────────────────────────────────

    def _handle_command(self, request, response):
        """
        处理 /hand_middle/command 服务请求。

        将ROS Service请求转换为HandBridge统一指令调用：
            request → HandBridge.execute_command(...)
            → orca_core.OrcaHand → 硬件执行

        Parameters
        ----------
        request : HandCommand.Request
            - command_type: str        指令类型
            - gesture_name: str        手势名（gesture模式）
            - amplitude: float32       开合幅度 0.0~1.0
            - joint_names: string[10]  目标关节名数组
            - joint_targets: float32[10] 目标角度数组
            - hold_time_sec: float32   保持时间
            - return_to_neutral: bool  是否回中
        response : HandCommand.Response
            - success: bool
            - execution_status: str
            - error_code: int32
            - error_message: str
            - timestamp: str

        Returns
        -------
        HandCommand.Response
        """
        self.get_logger().info(
            f"Received command: type={request.command_type}, "
            f"gesture={request.gesture_name}, "
            f"amplitude={request.amplitude:.2f}, "
            f"hold={request.hold_time_sec:.1f}s, "
            f"return_neutral={request.return_to_neutral}"
        )

        # ── 将ROS2固定数组转为Python列表 ──────────────────────
        joint_names_list = list(request.joint_names) if request.joint_names else []
        joint_targets_list = list(request.joint_targets) if request.joint_targets else []

        # 调试：打印非空关节目标
        non_empty = [
            (n, t) for n, t in zip(joint_names_list, joint_targets_list)
            if n.strip()
        ]
        if non_empty:
            self.get_logger().debug(f"Joint targets parsed: {non_empty}")

        # ── 调用中间层统一指令执行 ────────────────────────────
        try:
            result = self._bridge.execute_command(
                command_type=request.command_type,
                gesture_name=request.gesture_name,
                amplitude=request.amplitude,
                joint_names=joint_names_list,
                joint_targets=joint_targets_list,
                hold_time_sec=request.hold_time_sec,
                return_to_neutral=request.return_to_neutral,
            )
        except Exception as e:
            # 捕获 HandBridge 未预期的异常（防御性编程）
            self.get_logger().error(f"Unhandled exception in execute_command: {e}")
            self.get_logger().debug(traceback.format_exc())
            result = {
                "success": False,
                "execution_status": "failed",
                "error_code": 1006,
                "error_message": f"Internal bridge error: {e}",
                "timestamp": datetime.now(_CST).isoformat(),
            }

        # ── 填充ROS Service响应 ───────────────────────────────
        response.success = result.get("success", False)
        response.execution_status = result.get("execution_status", "failed")
        response.error_code = result.get("error_code", 1006)
        response.error_message = result.get("error_message", "Unknown error")
        response.timestamp = result.get("timestamp", datetime.now(_CST).isoformat())

        # ── 分级日志 ──────────────────────────────────────────
        if response.success:
            self.get_logger().info(
                f"Command '{request.command_type}' SUCCESS — "
                f"status={response.execution_status}"
            )
        else:
            log_level = "ERROR" if response.error_code >= 1000 else "WARN"
            self.get_logger().error(
                f"Command '{request.command_type}' FAILED — "
                f"error_code={response.error_code}, "
                f"message='{response.error_message}', "
                f"status={response.execution_status}"
            )

        return response

    # ── 生命周期 ────────────────────────────────────────────

    def destroy_node(self):
        """节点销毁时的清理：停止定时器 → 释放发布者 → 关闭设备。"""
        self.get_logger().info("Shutting down HandNode...")

        # 1. 停止状态发布定时器
        if self._status_timer is not None:
            self.destroy_timer(self._status_timer)
            self._status_timer = None
            self.get_logger().debug("Status timer destroyed")

        # 2. 释放发布者
        if self._status_publisher is not None:
            self.destroy_publisher(self._status_publisher)
            self._status_publisher = None
            self.get_logger().debug("Status publisher destroyed")

        # 3. 安全关闭设备
        if self._bridge is not None:
            self._bridge.shutdown()

        super().destroy_node()
        self.get_logger().info("HandNode shutdown complete")


# ── 入口点 ──────────────────────────────────────────────────

def main(args=None):
    """
    ROS2节点入口点 (console_scripts)。

    注册到 setup.py entry_points:
        hand_node = hand_middle.hand_node:main
    """
    rclpy.init(args=args)

    node = None
    try:
        node = HandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("KeyboardInterrupt received — shutting down")
    except Exception as e:
        logger.exception(f"HandNode crashed with unhandled exception: {e}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
