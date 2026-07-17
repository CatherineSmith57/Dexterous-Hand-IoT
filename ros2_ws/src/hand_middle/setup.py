"""
setup.py — hand_middle ROS2功能包构建配置

关键职责：
1. 将 third_party/orca_core 添加到Python检索路径 (sys.path)，
   解决跨目录导入报 ModuleNotFoundError 的问题。
2. 注册自定义Service消息 HandCommand.srv。
3. 注册自定义Topic消息 HandStatus.msg。
4. 注册ROS2节点入口点 (console_scripts)。
"""

import os
import sys
from glob import glob
from setuptools import setup

# ── 注入 orca_core 路径 ────────────────────────────────────
# third_party/orca_core 位于 ros2_ws 同级目录下，
# 从 setup.py 所在位置 (ros2_ws/src/hand_middle/) 推算：
#   setup.py 目录 → ../../ → 仓库根目录 → third_party/orca_core/
_package_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.abspath(os.path.join(_package_dir, "..", "..", ".."))
_orca_core_path = os.path.join(_repo_root, "third_party", "orca_core")

if os.path.isdir(_orca_core_path):
    if _orca_core_path not in sys.path:
        sys.path.insert(0, _orca_core_path)
else:
    # 兼容：尝试从环境变量 ORCA_CORE_PATH 读取
    _env_path = os.environ.get("ORCA_CORE_PATH", "")
    if _env_path and os.path.isdir(_env_path) and _env_path not in sys.path:
        sys.path.insert(0, _env_path)

# ── ROS2 ament_python 配置 ─────────────────────────────────
package_name = "hand_middle"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        # 安装launch文件
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        # 安装配置文件
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hand Middleware Team",
    maintainer_email="dev@orca-hand.local",
    description="ORCA Dexterous Hand ROS2 Middleware — Unified Service & Topic Bridge",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 主控制节点 (Service + 10Hz状态发布)
            "hand_node = hand_middle.hand_node:main",
            # 服务测试客户端
            "test_service_client = hand_middle.test_service_client:main",
            # 话题订阅测试节点
            "test_topic_subscriber = hand_middle.test_topic_subscriber:main",
        ],
    },
)
