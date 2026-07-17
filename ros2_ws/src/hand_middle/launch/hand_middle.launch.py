"""
hand_middle.launch.py — hand_middle 一键启动文件

启动 hand_node（主控节点），可选配置串口与发布参数。

使用方式:
    # 默认参数（/dev/ttyUSB0, 115200, 10Hz）
    ros2 launch hand_middle hand_middle.launch.py

    # 自定义串口（Windows）
    ros2 launch hand_middle hand_middle.launch.py port:=COM3

    # 自定义串口（Linux）
    ros2 launch hand_middle hand_middle.launch.py port:=/dev/ttyUSB1 baudrate:=921600

    # 跳过自动初始化（仅启动节点，不连接设备）
    ros2 launch hand_middle hand_middle.launch.py auto_initialize:=false

    # 修改状态发布频率
    ros2 launch hand_middle hand_middle.launch.py status_publish_rate:=20.0

    # 调整保护阈值
    ros2 launch hand_middle hand_middle.launch.py temperature_limit:=80.0 torque_limit:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成LaunchDescription供 ros2 launch 使用。

    启动的节点:
        hand_node — 主控节点
            Service: /hand_middle/command
            Topic:   /hand_middle/status (10Hz)
    """

    # ── 声明启动参数 ────────────────────────────────────────

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description=(
            "Serial port for ORCA hand. "
            "Linux: /dev/ttyUSB0, Windows: COM3"
        ),
    )

    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Serial baudrate (standard: 115200, 921600)",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "status_publish_rate",
        default_value="10.0",
        description="Status publish rate in Hz (default: 10.0)",
    )

    auto_init_arg = DeclareLaunchArgument(
        "auto_initialize",
        default_value="true",
        description=(
            "Auto-connect and calibrate on startup. "
            "Set to 'false' if hardware is not connected."
        ),
    )

    temp_limit_arg = DeclareLaunchArgument(
        "temperature_limit",
        default_value="75.0",
        description="Temperature limit in °C — triggers fault_code=1 if exceeded",
    )

    torque_limit_arg = DeclareLaunchArgument(
        "torque_limit",
        default_value="2.5",
        description="Torque limit in N·m — triggers fault_code=2 if exceeded",
    )

    # ── hand_node（主控节点）─────────────────────────────────

    hand_node = Node(
        package="hand_middle",
        executable="hand_node",
        name="hand_node",
        output="screen",
        # 模拟时钟注释（真实硬件不需要）:
        # parameters=[{"use_sim_time": True}],
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "status_publish_rate": LaunchConfiguration("status_publish_rate"),
                "auto_initialize": LaunchConfiguration("auto_initialize"),
                "temperature_limit": LaunchConfiguration("temperature_limit"),
                "torque_limit": LaunchConfiguration("torque_limit"),
            }
        ],
        # 节点崩溃时自动重启（ROS2 Jazz 支持）
        # respawn=True,
        # respawn_delay=2.0,
    )

    # ── 启动日志 ────────────────────────────────────────────

    startup_log = LogInfo(
        msg=[
            "╔══════════════════════════════════════════════╗",
            "║   ORCA Dexterous Hand — Hand Middleware      ║",
            "╠══════════════════════════════════════════════╣",
            "║  Service: /hand_middle/command               ║",
            "║  Topic:   /hand_middle/status (10Hz)         ║",
            "╠══════════════════════════════════════════════╣",
            "║  port: ", LaunchConfiguration("port"),
            "║  baudrate: ", LaunchConfiguration("baudrate"),
            "║  publish_rate: ", LaunchConfiguration("status_publish_rate"),
            "║  auto_init: ", LaunchConfiguration("auto_initialize"),
            "╚══════════════════════════════════════════════╝",
        ]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        publish_rate_arg,
        auto_init_arg,
        temp_limit_arg,
        torque_limit_arg,
        startup_log,
        hand_node,
    ])
