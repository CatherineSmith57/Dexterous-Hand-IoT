r"""
main.py — 手部视觉追踪 + 17 关节实时机械手控制

保留旧版的全部界面和按键功能：
    q / ESC — 退出
    e       — 连接机械手或暂停/恢复实时控制
    s       — 打印机械手状态与当前关节角
    t       — 切换简洁/详细显示

仅视觉：
    python -m stimulated_hand.main

视觉 + 机械手：
    python -m stimulated_hand.main --robot \
        --calibration .\flexible_hand\hardware\calibration.yaml
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from stimulated_hand.hand_tracker import HandTracker
from stimulated_hand.joint_mapper import JointMapper


# ============================================================
# 显示模块：保留旧版状态面板、按键提示和底部状态栏
# ============================================================

def render_frame(
    frame: np.ndarray,
    hands: list[dict] | None,
    joint_angles: dict[str, float] | None,
    status: dict,
) -> np.ndarray:
    display = frame.copy()
    h, w = display.shape[:2]

    panel_w = 280
    panel_x = max(0, w - panel_w)

    overlay = display.copy()
    cv2.rectangle(
        overlay,
        (panel_x, 0),
        (w, h),
        (40, 40, 45),
        -1,
    )
    cv2.addWeighted(
        overlay,
        0.7,
        display,
        0.3,
        0,
        display,
    )

    y = 20

    cv2.putText(
        display,
        "STIMULATED HAND",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (200, 200, 200),
        1,
    )
    y += 35
    cv2.line(
        display,
        (panel_x + 10, y),
        (w - 10, y),
        (80, 80, 80),
        1,
    )
    y += 10

    connected = bool(status.get("robot_connected", False))
    robot_color = (0, 255, 0) if connected else (0, 0, 255)
    robot_text = "CONNECTED" if connected else "OFFLINE"
    cv2.putText(
        display,
        f"Robot:  {robot_text}",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        robot_color,
        1,
    )
    y += 28

    enabled = bool(status.get("robot_enabled", False))
    mode_color = (0, 255, 255) if enabled else (150, 150, 150)
    mode_text = "CONTROLLING" if enabled else "TRACKING ONLY"
    cv2.putText(
        display,
        f"Mode:   {mode_text}",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        mode_color,
        1,
    )
    y += 28

    cv2.putText(
        display,
        f"FPS:    {status.get('fps', 0.0):.1f}",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
    )
    y += 30
    cv2.line(
        display,
        (panel_x + 10, y),
        (w - 10, y),
        (80, 80, 80),
        1,
    )
    y += 10

    cv2.putText(
        display,
        "JOINT ANGLES (deg):",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (180, 180, 180),
        1,
    )
    y += 25

    if joint_angles:
        show_joints = [
            ("index_mcp", "i_mcp"),
            ("index_pip", "i_pip"),
            ("middle_mcp", "m_mcp"),
            ("middle_pip", "m_pip"),
            ("ring_mcp", "r_mcp"),
            ("ring_pip", "r_pip"),
            ("pinky_mcp", "p_mcp"),
            ("pinky_pip", "p_pip"),
            ("thumb_cmc", "t_cmc"),
            ("thumb_mcp", "t_mcp"),
        ]

        if status.get("show_detail", False):
            show_joints = [
                (name, name)
                for name in sorted(joint_angles)
            ]

        # 详细模式时 17 行可能超过窗口高度，给按键区预留空间。
        max_joint_y = max(245, h - 155)
        for full_name, short_name in show_joints:
            if y + 18 > max_joint_y:
                break

            angle = float(joint_angles.get(full_name, 0.0))
            cv2.putText(
                display,
                f"  {short_name:10s} {angle:6.1f}",
                (panel_x + 15, y + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
            )
            y += 18
    else:
        cv2.putText(
            display,
            "  ---  (no hand)",
            (panel_x + 15, y + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (100, 100, 100),
            1,
        )
        y += 18

    # 按键提示固定在面板底部，防止详细关节列表将其挤出窗口。
    controls_y = max(y + 15, h - 135)
    cv2.line(
        display,
        (panel_x + 10, controls_y),
        (w - 10, controls_y),
        (80, 80, 80),
        1,
    )
    controls_y += 10

    cv2.putText(
        display,
        "CONTROLS:",
        (panel_x + 15, controls_y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (150, 150, 150),
        1,
    )
    controls_y += 22

    controls = [
        ("q/ESC", "Quit"),
        ("e", "Connect / Toggle"),
        ("s", "Print Status"),
        ("t", "Detail"),
    ]
    for key, description in controls:
        if controls_y + 15 >= h - 8:
            break
        cv2.putText(
            display,
            f"  [{key}]  {description}",
            (panel_x + 15, controls_y + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            (130, 130, 130),
            1,
        )
        controls_y += 19

    hand_text = (
        "Hand: DETECTED"
        if hands
        else "Hand: NOT DETECTED"
    )
    hand_color = (
        (0, 255, 0)
        if hands
        else (0, 0, 255)
    )
    cv2.putText(
        display,
        hand_text,
        (10, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        hand_color,
        1,
    )

    robot_info = f"Robot: {robot_text}"
    robot_x = max(10, panel_x - 200)
    cv2.putText(
        display,
        robot_info,
        (robot_x, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        robot_color,
        1,
    )

    return display


# ============================================================
# 机器人桥接：只调用 flexible_hand.hand_interface
# ============================================================

class RobotBridge:
    def __init__(
        self,
        config_path: str | None = None,
        calibration_path: str | None = None,
    ):
        self.config_path = config_path
        self.calibration_path = calibration_path
        self._connected = False
        self._last_error = ""

    @staticmethod
    def _api():
        from flexible_hand import hand_interface
        return hand_interface

    def connect(self) -> bool:
        try:
            api = self._api()
            ok = bool(
                api.init(
                    config_path=self.config_path,
                    calibration_path=self.calibration_path,
                )
            )
            self._connected = ok

            if ok:
                self._last_error = ""
                print(
                    "[RobotBridge] 机械手连接、limits 验证、"
                    "回中和实时调度器启动成功"
                )
            else:
                status = api.get_status()
                self._last_error = str(
                    status.get(
                        "last_error",
                        "unknown initialization error",
                    )
                )
                print(
                    "[RobotBridge] 初始化失败: "
                    f"{self._last_error}"
                )

            return ok

        except Exception as exc:
            self._last_error = str(exc)
            self._connected = False
            print(
                f"[RobotBridge] 连接异常: {exc}"
            )
            return False

    def is_connected(self) -> bool:
        if not self._connected:
            return False

        try:
            connected = bool(
                self._api()
                .get_status()
                .get("connected", False)
            )
            if not connected:
                self._connected = False
            return connected
        except Exception as exc:
            self._last_error = str(exc)
            self._connected = False
            return False

    def send_joint_angles(
        self,
        joint_angles: dict[str, float],
        confidence: float,
    ) -> dict:
        if not self.is_connected():
            return {
                "status": "skipped",
                "error_code": 1001,
                "message": "Hand not connected",
            }

        result = self._api().update_from_vision(
            joint_angles,
            confidence=confidence,
            min_confidence=0.6,
        )

        if result.get("status") == "failed":
            self._last_error = str(
                result.get("message", "unknown error")
            )
            print(
                "[RobotBridge] 实时发送失败: "
                f"{result}"
            )

        return result

    def get_status(self) -> dict:
        try:
            status = self._api().get_status()
            if self._last_error and not status.get("last_error"):
                status["last_error"] = self._last_error
            return status
        except Exception as exc:
            return {
                "connected": False,
                "last_error": str(exc),
            }

    def pause_control(self) -> None:
        """暂停控制时关闭电机扭矩。"""
        if not self.is_connected():
            return

        result = self._api().emergency_stop()
        if not result.get("success", False):
            print(
                "[RobotBridge] 暂停控制时关闭扭矩失败: "
                f"{result}"
            )

    def disconnect(self) -> None:
        try:
            self._api().cleanup()
            print("[RobotBridge] 机械手已断开")
        except Exception as exc:
            print(
                f"[RobotBridge] 断开异常: {exc}"
            )
        finally:
            self._connected = False


# ============================================================
# EMA 平滑
# ============================================================

class EMASmoother:
    def __init__(self, alpha: float = 0.3):
        if not 0.0 < alpha <= 1.0:
            raise ValueError(
                "alpha must be in (0, 1]"
            )
        self.alpha = alpha
        self._prev: dict[str, float] = {}

    def smooth(
        self,
        angles: dict[str, float],
    ) -> dict[str, float]:
        smoothed = {}
        for name, angle in angles.items():
            previous = self._prev.get(name, angle)
            value = (
                self.alpha * angle
                + (1.0 - self.alpha) * previous
            )
            smoothed[name] = value
            self._prev[name] = value
        return smoothed

    def reset(self) -> None:
        self._prev.clear()


# ============================================================
# 主程序
# ============================================================

def main(
    connect_robot: bool = False,
    camera_id: int = 0,
    config_path: str | None = None,
    calibration_path: str | None = None,
) -> None:
    print("=" * 50)
    print("  Stimulated Hand — 手部动作模仿")
    print("=" * 50)
    print()

    print("[Init] 打开摄像头...")
    try:
        tracker = HandTracker(
            camera_id=camera_id
        )
        print(
            f"[Init] 摄像头 (ID={camera_id}) 就绪"
        )
    except RuntimeError as exc:
        print(f"[Error] {exc}")
        raise SystemExit(1)

    mapper = JointMapper()
    smoother = EMASmoother(alpha=0.3)
    bridge = RobotBridge(
        config_path=config_path,
        calibration_path=calibration_path,
    )

    robot_available = False
    if connect_robot:
        print("[Init] 尝试连接机械手...")
        robot_available = bridge.connect()

    robot_enabled = robot_available
    show_detail = False

    print()
    print("  按键说明:")
    print("    q / ESC — 退出")
    print("    e       — 连接或暂停/恢复机械手控制")
    print("    s       — 打印机械手状态和当前关节角")
    print("    t       — 切换简洁/详细显示")
    print()

    if not robot_available:
        print(
            "[Info] 机械手未连接，仅运行视觉追踪模式"
        )
        print(
            "[Info] 按 E 键可再次尝试连接机械手"
        )
    else:
        print(
            "[Info] 机械手已连接，按 E 可暂停/恢复控制"
        )

    fps_frame_count = 0
    fps_last_time = time.time()
    fps_value = 0.0

    last_hand_time = time.monotonic()
    no_hand_stopped = False
    no_hand_timeout = 0.5
    joint_angles: Optional[dict[str, float]] = None

    try:
        while True:
            success, frame = tracker.get_frame()
            if not success:
                print(
                    "[Warn] 读取摄像头帧失败，跳过"
                )
                continue

            hands = tracker.detect(frame)
            frame = tracker.draw_landmarks(
                frame,
                hands,
            )

            joint_angles = None
            if hands:
                primary_hand = hands[0]
                last_hand_time = time.monotonic()
                no_hand_stopped = False

                raw_angles = mapper.compute(
                    primary_hand["landmarks"]
                )
                joint_angles = smoother.smooth(
                    raw_angles
                )

                if (
                    robot_enabled
                    and bridge.is_connected()
                ):
                    bridge.send_joint_angles(
                        joint_angles,
                        confidence=float(
                            primary_hand.get(
                                "score",
                                1.0,
                            )
                        ),
                    )

            elif (
                robot_enabled
                and bridge.is_connected()
                and not no_hand_stopped
                and (
                    time.monotonic()
                    - last_hand_time
                    > no_hand_timeout
                )
            ):
                # 丢手后不继续保持可能错误的姿态。
                bridge.pause_control()
                robot_enabled = False
                no_hand_stopped = True
                print(
                    "[Safety] 超过 0.5 秒未检测到手，"
                    "已暂停控制并关闭扭矩"
                )

            fps_frame_count += 1
            if fps_frame_count >= 30:
                now = time.time()
                fps_value = (
                    30.0
                    / (now - fps_last_time + 1e-9)
                )
                fps_last_time = now
                fps_frame_count = 0

            status = {
                "robot_connected":
                    bridge.is_connected(),
                "robot_enabled": robot_enabled,
                "fps": fps_value,
                "show_detail": show_detail,
            }

            display = render_frame(
                frame,
                hands,
                joint_angles,
                status,
            )
            cv2.imshow(
                "Stimulated Hand — 手部动作模仿",
                display,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                print("[Exit] 用户退出")
                break

            elif key == ord("e"):
                if not bridge.is_connected():
                    print(
                        "[Action] 尝试连接机械手..."
                    )
                    if bridge.connect():
                        robot_available = True
                        robot_enabled = True
                        smoother.reset()
                        print(
                            "[Action] 连接成功，实时控制已开启"
                        )
                    else:
                        robot_available = False
                        robot_enabled = False
                        print(
                            "[Action] 连接失败，继续视觉追踪"
                        )
                else:
                    robot_enabled = not robot_enabled
                    if robot_enabled:
                        smoother.reset()
                        print(
                            "[Action] 机械手实时控制：开启"
                        )
                    else:
                        bridge.pause_control()
                        print(
                            "[Action] 机械手实时控制：暂停，"
                            "电机扭矩已关闭"
                        )

            elif key == ord("s"):
                robot_status = bridge.get_status()
                print(
                    f"[Status] 机械手: {robot_status}"
                )

                if joint_angles:
                    print(
                        "[Status] 当前 17 关节角:"
                    )
                    for name in sorted(joint_angles):
                        print(
                            f"  {name:12s}: "
                            f"{joint_angles[name]:7.2f}°"
                        )
                else:
                    print(
                        "[Status] 当前未检测到手"
                    )

            elif key == ord("t"):
                show_detail = not show_detail
                print(
                    "[Display] 模式: "
                    f"{'详细' if show_detail else '简洁'}"
                )

    except KeyboardInterrupt:
        print("\n[Exit] 用户中断")

    finally:
        print("[Cleanup] 正在清理...")
        try:
            tracker.release()
        finally:
            bridge.disconnect()
            cv2.destroyAllWindows()
        print("[Cleanup] 完成")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Stimulated Hand — 手部动作模仿模块"
        ),
        formatter_class=(
            argparse.RawDescriptionHelpFormatter
        ),
        epilog=r"""
示例:
  python -m stimulated_hand.main
  python -m stimulated_hand.main --robot
  python -m stimulated_hand.main --camera 1
  python -m stimulated_hand.main --robot ^
      --calibration .\flexible_hand\hardware\calibration.yaml
""",
    )
    parser.add_argument(
        "--robot",
        action="store_true",
        help="启动时自动连接机械手",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="摄像头设备编号（默认 0）",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="config_safe.yaml 路径",
    )
    parser.add_argument(
        "--calibration",
        type=str,
        default=None,
        help="数字 Motor ID limits YAML 路径",
    )
    args = parser.parse_args()

    main(
        connect_robot=args.robot,
        camera_id=args.camera,
        config_path=args.config,
        calibration_path=args.calibration,
    )
