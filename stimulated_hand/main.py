"""main.py — 主循环 + OpenCV 显示窗口 + 机器人桥接

将手部追踪、关节映射、可视化显示、机械手控制整合在一起。

用法:
    # 仅视觉追踪（不控制机械手）
    python stimulated_hand/main.py

    # 视觉追踪 + 机械手控制
    python stimulated_hand/main.py --robot

按键操作:
    q / ESC — 退出
    e       — 切换机械手执行开关
    s       — 打印机械手状态到控制台
    t       — 切换显示模式（简洁/详细）
"""

import sys
import time
import argparse
from pathlib import Path

import cv2
import numpy as np

# 确保能导入项目内的模块（stimulated_hand 和 scripts）
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_project_root_str = str(_PROJECT_ROOT)
if _project_root_str not in sys.path:
    sys.path.insert(0, _project_root_str)

from stimulated_hand.hand_tracker import HandTracker
from stimulated_hand.joint_mapper import JointMapper


# ============================================================
#  显示模块（内嵌在 main.py 中，方便初学者理解完整流程）
# ============================================================

def render_frame(
    frame: np.ndarray,
    hands: list[dict] | None,
    joint_angles: dict[str, float] | None,
    status: dict,
) -> np.ndarray:
    """在摄像头画面上叠加状态面板和关节角度信息。

    Args:
        frame: 原始 BGR 图像（已画好骨骼线）
        hands: 手部检测结果或 None
        joint_angles: 关节角度字典或 None
        status: 状态信息字典，包含 robot_connected, fps, mode 等

    Returns:
        带完整标注的图像
    """
    display = frame.copy()
    h, w = display.shape[:2]

    # === 右侧信息面板 ===
    panel_w = 280
    panel_x = w - panel_w

    # 半透明背景
    overlay = display.copy()
    cv2.rectangle(overlay, (panel_x, 0), (w, h), (40, 40, 45), -1)
    cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)

    y = 20  # 当前绘制行

    # 标题
    cv2.putText(display, "STIMULATED HAND", (panel_x + 15, y + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    y += 35
    cv2.line(display, (panel_x + 10, y), (w - 10, y), (80, 80, 80), 1)
    y += 10

    # 机械手状态
    robot_color = (0, 255, 0) if status["robot_connected"] else (0, 0, 255)
    robot_text = "CONNECTED" if status["robot_connected"] else "OFFLINE"
    cv2.putText(display, f"Robot:  {robot_text}",
                (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                robot_color, 1)
    y += 28

    # 控制模式
    mode_color = (0, 255, 255) if status["robot_enabled"] else (150, 150, 150)
    mode_text = "CONTROLLING" if status["robot_enabled"] else "TRACKING ONLY"
    cv2.putText(display, f"Mode:   {mode_text}",
                (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                mode_color, 1)
    y += 28

    # FPS
    cv2.putText(display, f"FPS:    {status['fps']:.1f}",
                (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 255), 1)
    y += 30
    cv2.line(display, (panel_x + 10, y), (w - 10, y), (80, 80, 80), 1)
    y += 10

    # 关节角度
    cv2.putText(display, "JOINT ANGLES (deg):",
                (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (180, 180, 180), 1)
    y += 25

    if joint_angles:
        # 只显示有变化的主要关节（mcp 和 pip）
        show_joints = [
            ("index_mcp", "i_mcp"), ("index_pip", "i_pip"),
            ("middle_mcp", "m_mcp"), ("middle_pip", "m_pip"),
            ("ring_mcp", "r_mcp"), ("ring_pip", "r_pip"),
            ("pinky_mcp", "p_mcp"), ("pinky_pip", "p_pip"),
            ("thumb_cmc", "t_cmc"), ("thumb_mcp", "t_mcp"),
        ]

        if status.get("show_detail", False):
            # 详细模式：显示全部 17 个关节
            show_joints = [(name, name) for name in sorted(joint_angles.keys())]

        for full_name, short_name in show_joints:
            angle = joint_angles.get(full_name, 0.0)
            bar_len = int(abs(angle) / 120 * 80)  # 小进度条
            bar = "#" * min(bar_len, 40) + " " * max(0, 40 - bar_len)

            cv2.putText(display, f"  {short_name:8s} {angle:6.1f}",
                        (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (255, 255, 255), 1)
            y += 18
    else:
        cv2.putText(display, "  ---  (no hand)",
                    (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (100, 100, 100), 1)
        y += 18

    y += 5
    cv2.line(display, (panel_x + 10, y), (w - 10, y), (80, 80, 80), 1)
    y += 10

    # 按键提示
    cv2.putText(display, "CONTROLS:",
                (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (150, 150, 150), 1)
    y += 22
    for key, desc in [("q/ESC", "Quit"), ("e", "Toggle Robot"),
                       ("s", "Status"), ("t", "Detail")]:
        cv2.putText(display, f"  [{key}]  {desc}",
                    (panel_x + 15, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (120, 120, 120), 1)
        y += 20

    # === 底部状态栏 ===
    y = h - 10
    hand_status = f"Hand: {'DETECTED' if hands else 'NOT DETECTED'}"
    status_color = (0, 255, 0) if hands else (0, 0, 255)
    cv2.putText(display, hand_status, (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)

    robot_info = f"Robot: {robot_text}"
    cv2.putText(display, robot_info, (w - panel_w - 200, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, robot_color, 1)

    return display


# ============================================================
#  机器人桥接模块（内嵌，简化调用）
# ============================================================

class RobotBridge:
    """对队友 scripts/hand_interface.py 的薄封装。

    处理连接、断开、发送关节角度和错误恢复。
    """

    def __init__(self, config_path: str | None = None):
        self.config_path = config_path
        self._connected = False

    def connect(self) -> bool:
        """尝试连接机械手。成功返回 True，失败返回 False。"""
        try:
            import hand_interface
            self.hi = hand_interface
            result = self.hi.init(self.config_path)
            self._connected = result
            if result:
                print("[RobotBridge] 机械手连接成功")
            else:
                print("[RobotBridge] 机械手连接失败（init 返回 False）")
            return result
        except ImportError:
            print("[RobotBridge] 无法导入 hand_interface（orca_core 可能未安装）")
            self._connected = False
            return False
        except Exception as e:
            print(f"[RobotBridge] 连接异常: {e}")
            self._connected = False
            return False

    def is_connected(self) -> bool:
        return self._connected

    def send_joint_angles(self, joint_angles: dict, hold_time: float = 0.05) -> dict:
        """发送关节角度到机械手。

        Args:
            joint_angles: {关节名: 角度}
            hold_time: 保持时间（秒），实时模式用很短的时间

        Returns:
            执行结果字典，包含 success, error_code 等
        """
        if not self._connected:
            return {"success": False, "status": "skipped"}

        try:
            return self.hi.do_joint_command(joint_angles, hold_time_sec=hold_time)
        except Exception as e:
            print(f"[RobotBridge] 发送失败: {e}")
            return {"success": False, "error_code": 9999, "error_message": str(e)}

    def get_status(self) -> dict:
        """获取机械手当前状态。"""
        if not self._connected:
            return {"connected": False}
        try:
            return self.hi.get_status()
        except Exception:
            return {"connected": self._connected}

    def disconnect(self):
        """断开机械手连接。"""
        if self._connected:
            try:
                self.hi.cleanup()
                print("[RobotBridge] 机械手已断开")
            except Exception as e:
                print(f"[RobotBridge] 断开异常: {e}")
        self._connected = False


# ============================================================
#  EMA 平滑滤波器
# ============================================================

class EMASmoother:
    """指数移动平均滤波器，用于平滑关节角度，减少抖动。

    公式: smoothed = alpha * current + (1-alpha) * previous
    alpha 越小，平滑效果越强（但响应越慢）。
    """

    def __init__(self, alpha: float = 0.3):
        self.alpha = alpha
        self._prev: dict[str, float] = {}

    def smooth(self, angles: dict[str, float]) -> dict[str, float]:
        """对输入的关节角度做平滑处理。

        Args:
            angles: 当前帧计算出的原始关节角度

        Returns:
            平滑后的关节角度
        """
        smoothed = {}
        for name, angle in angles.items():
            prev = self._prev.get(name, angle)
            smoothed[name] = self.alpha * angle + (1.0 - self.alpha) * prev
            self._prev[name] = smoothed[name]
        return smoothed


# ============================================================
#  主程序
# ============================================================

def main(
    connect_robot: bool = False,
    camera_id: int = 0,
    config_path: str | None = None,
):
    """运行手部动作模仿的主循环。

    Args:
        connect_robot: 是否在启动时连接机械手
        camera_id: 摄像头设备编号
        config_path: 机械手配置文件路径
    """
    print("=" * 50)
    print("  Stimulated Hand — 手部动作模仿")
    print("=" * 50)
    print()

    # ---- 初始化各模块 ----
    print("[Init] 打开摄像头...")
    try:
        tracker = HandTracker(camera_id=camera_id)
        print(f"[Init] 摄像头 (ID={camera_id}) 就绪")
    except RuntimeError as e:
        print(f"[Error] {e}")
        sys.exit(1)

    mapper = JointMapper()
    smoother = EMASmoother(alpha=0.3)
    bridge = RobotBridge(config_path=config_path)

    # 尝试连接机械手
    robot_available = False
    if connect_robot:
        print("[Init] 尝试连接机械手...")
        robot_available = bridge.connect()
    robot_enabled = robot_available  # 启动时如果连接上了就开启执行
    show_detail = False

    print()
    print("  按键说明:")
    print("    q / ESC — 退出")
    print("    e       — 切换机械手执行开关")
    print("    s       — 打印机械手状态")
    print("    t       — 切换显示模式")
    print()

    if not robot_available:
        print("[Info] 机械手未连接，仅运行视觉追踪模式")
        print("[Info] 按 E 键可尝试连接机械手")
    else:
        print("[Info] 机械手已连接，按 E 键可暂停/恢复控制")

    # ---- 主循环 ----
    fps_frame_count = 0
    fps_last_time = time.time()
    fps_value = 0.0
    frame_count = 0  # 用于跳帧

    try:
        while True:
            # 1. 读取摄像头帧
            success, frame = tracker.get_frame()
            if not success:
                print("[Warn] 读取摄像头帧失败，跳过")
                continue

            # 2. 检测手部关键点
            hands = tracker.detect(frame)

            # 3. 画骨骼线
            frame = tracker.draw_landmarks(frame, hands)

            # 4. 计算关节角度（仅当检测到手时）
            joint_angles = None
            if hands:
                # 使用第一只手
                landmarks = hands[0]["landmarks"]
                raw_angles = mapper.compute(landmarks)
                joint_angles = smoother.smooth(raw_angles)

                # 5. 发送到机械手（每 3 帧发一次，减少负担）
                if robot_enabled and bridge.is_connected():
                    if frame_count % 3 == 0:
                        bridge.send_joint_angles(joint_angles, hold_time=0.05)

            # 6. FPS 计算
            fps_frame_count += 1
            if fps_frame_count >= 30:
                now = time.time()
                fps_value = 30.0 / (now - fps_last_time + 1e-9)
                fps_last_time = now
                fps_frame_count = 0

            # 7. 渲染显示
            status = {
                "robot_connected": bridge.is_connected(),
                "robot_enabled": robot_enabled,
                "fps": fps_value,
                "show_detail": show_detail,
            }
            display = render_frame(frame, hands, joint_angles, status)

            # 8. 显示窗口
            cv2.imshow("Stimulated Hand — 手部动作模仿", display)

            # 9. 按键处理
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:  # q 或 ESC
                print("[Exit] 用户退出")
                break
            elif key == ord('e'):
                # 切换机械手执行
                if not bridge.is_connected():
                    print("[Action] 尝试连接机械手...")
                    if bridge.connect():
                        robot_available = True
                        robot_enabled = True
                    else:
                        print("[Action] 连接失败，继续追踪模式")
                else:
                    robot_enabled = not robot_enabled
                    status_text = "控制中" if robot_enabled else "追踪中"
                    print(f"[Action] 机械手执行: {'开启' if robot_enabled else '暂停'} ({status_text})")
            elif key == ord('s'):
                # 打印状态
                robot_status = bridge.get_status()
                print(f"[Status] 机械手: {robot_status}")
                if joint_angles:
                    print(f"[Status] 关节角度: {dict(list(joint_angles.items())[:8])}...")
            elif key == ord('t'):
                # 切换详细显示
                show_detail = not show_detail
                print(f"[Display] 模式: {'详细' if show_detail else '简洁'}")

            frame_count += 1

    except KeyboardInterrupt:
        print("\n[Exit] 用户中断")
    finally:
        # ---- 清理 ----
        print("[Cleanup] 正在清理...")
        tracker.release()
        bridge.disconnect()
        cv2.destroyAllWindows()
        print("[Cleanup] 完成")


# ============================================================
#  命令行入口
# ============================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Stimulated Hand — 手部动作模仿模块",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
    python stimulated_hand/main.py              仅视觉追踪
    python stimulated_hand/main.py --robot      视觉追踪+机械手控制
    python stimulated_hand/main.py --camera 1   使用第二个摄像头
        """,
    )
    parser.add_argument(
        "--robot", action="store_true",
        help="启动时自动连接机械手"
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="摄像头设备编号（默认 0）"
    )
    parser.add_argument(
        "--config", type=str, default=None,
        help="机械手配置文件路径"
    )
    args = parser.parse_args()

    main(
        connect_robot=args.robot,
        camera_id=args.camera,
        config_path=args.config,
    )
