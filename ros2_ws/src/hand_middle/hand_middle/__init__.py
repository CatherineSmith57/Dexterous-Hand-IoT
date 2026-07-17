"""
hand_middle — ORCA灵巧手ROS2中间层功能包

职责：
1. 将上层ROS2 Service命令转换为orca_core底层库调用
2. 以10Hz频率发布手部实时状态 (关节位置/温度/力矩/故障)
3. 串联整套灵巧手系统：命令下发 → 执行 → 状态回传

对外接口:
    Service: /hand_middle/command   (HandCommand.srv)
    Topic:   /hand_middle/status    (HandStatus.msg, 10Hz)

路径: ros2_ws/src/hand_middle/hand_middle/
"""

# ── 注入 orca_core 路径 ────────────────────────────────────
# 确保运行时也能找到 third_party/orca_core
import os
import sys

_pkg_dir = os.path.dirname(os.path.abspath(__file__))
# hand_middle/hand_middle/ → ../../.. → 仓库根目录
_repo_root = os.path.abspath(os.path.join(_pkg_dir, "..", "..", "..", ".."))
_orca_core_path = os.path.join(_repo_root, "third_party", "orca_core")

if os.path.isdir(_orca_core_path) and _orca_core_path not in sys.path:
    sys.path.insert(0, _orca_core_path)  # add third_party/orca_core/

# 也尝试环境变量
_env_path = os.environ.get("ORCA_CORE_PATH", "")
if _env_path and os.path.isdir(_env_path) and _env_path not in sys.path:
    sys.path.insert(0, _env_path)
