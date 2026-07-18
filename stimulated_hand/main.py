r"""
main.py — 手部视觉追踪 + 17 关节实时机械手控制

按键：
    q / ESC     退出
    e           暂停/恢复实时跟随；暂停时保持最后目标
    p / Space   Snapshot：截取当前视觉姿态，只下发一次并固定
    d           打印角度 → Motor ID → target raw → actual raw
    x           急停并关闭全部电机扭矩
    s           打印机械手状态和当前关节角
    t           切换简洁/详细显示
    [ / ]       运行中降低/提高 speed

示例：
    python -m stimulated_hand.main --robot \
        --calibration .\flexible_hand\hardware\calibration.yaml \
        --speed 40 --ema-alpha 0.45
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


def render_frame(
    frame: np.ndarray,
    hands: list[dict] | None,
    joint_angles: dict[str, float] | None,
    status: dict,
) -> np.ndarray:
    display = frame.copy()
    h, w = display.shape[:2]

    panel_w = 300
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

    connected = bool(
        status.get("robot_connected", False)
    )
    robot_color = (
        (0, 255, 0)
        if connected
        else (0, 0, 255)
    )
    robot_text = (
        "CONNECTED"
        if connected
        else "OFFLINE"
    )
    cv2.putText(
        display,
        f"Robot: {robot_text}",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        robot_color,
        1,
    )
    y += 28

    enabled = bool(
        status.get("robot_enabled", False)
    )
    snapshot_hold = bool(
        status.get("snapshot_hold", False)
    )

    if snapshot_hold:
        mode_text = "SNAPSHOT HOLD"
        mode_color = (255, 200, 0)
    elif enabled:
        mode_text = "CONTROLLING"
        mode_color = (0, 255, 255)
    elif connected:
        mode_text = "PAUSED / HOLD"
        mode_color = (180, 180, 180)
    else:
        mode_text = "TRACKING ONLY"
        mode_color = (150, 150, 150)

    cv2.putText(
        display,
        f"Mode:  {mode_text}",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        mode_color,
        1,
    )
    y += 28

    cv2.putText(
        display,
        f"FPS:   {status.get('fps', 0.0):.1f}",
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
    )
    y += 23

    cv2.putText(
        display,
        (
            f"Speed:{status.get('speed', 0)}  "
            f"EMA:{status.get('ema_alpha', 0.0):.2f}"
        ),
        (panel_x + 15, y + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (200, 200, 120),
        1,
    )
    y += 27

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
        0.43,
        (180, 180, 180),
        1,
    )
    y += 24

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

        max_joint_y = max(245, h - 205)
        for full_name, short_name in show_joints:
            if y + 18 > max_joint_y:
                break

            angle = float(
                joint_angles.get(
                    full_name,
                    0.0,
                )
            )
            cv2.putText(
                display,
                f"  {short_name:10s} {angle:6.1f}",
                (panel_x + 15, y + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.37,
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

    controls_y = max(y + 15, h - 190)
    cv2.line(
        display,
        (panel_x + 10, controls_y),
        (w - 10, controls_y),
        (80, 80, 80),
        1,
    )
    controls_y += 8

    cv2.putText(
        display,
        "CONTROLS:",
        (panel_x + 15, controls_y + 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (150, 150, 150),
        1,
    )
    controls_y += 20

    controls = [
        ("q", "Quit"),
        ("e", "Live / Pause"),
        ("p", "Snapshot Hold"),
        ("d", "Mapping Debug"),
        ("x", "Emergency Stop"),
        ("[ ]", "Speed - / +"),
        ("s/t", "Status / Detail"),
    ]
    for key, description in controls:
        if controls_y + 14 >= h - 8:
            break
        cv2.putText(
            display,
            f" [{key:4s}] {description}",
            (panel_x + 15, controls_y + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (135, 135, 135),
            1,
        )
        controls_y += 18

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
    cv2.putText(
        display,
        robot_info,
        (max(10, panel_x - 205), h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        robot_color,
        1,
    )

    return display


class RobotBridge:
    def __init__(
        self,
        config_path: str | None,
        calibration_path: str | None,
        speed: int,
        acc: int,
        torque: int,
        control_hz: float,
    ):
        self.config_path = config_path
        self.calibration_path = calibration_path

        self.speed = int(speed)
        self.acc = int(acc)
        self.torque = int(torque)
        self.control_hz = float(control_hz)

        self._connected = False
        self._last_error = ""

    @staticmethod
    def _api():
        from flexible_hand import hand_interface
        return hand_interface

    def connect(self) -> bool:
        try:
            api = self._api()
            api.configure_motion(
                speed=self.speed,
                acc=self.acc,
                torque=self.torque,
                control_hz=self.control_hz,
            )
            ok = bool(
                api.init(
                    config_path=self.config_path,
                    calibration_path=self.calibration_path,
                )
            )
            self._connected = ok

            if ok:
                print(
                    "[RobotBridge] 连接、limits 验证、"
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
        except Exception:
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
            print(
                "[RobotBridge] 实时发送失败: "
                f"{result}"
            )
        return result

    def pause_hold(self) -> dict:
        """清空视觉队列，但让电机保持最后目标。"""
        if not self.is_connected():
            return {
                "success": False,
                "message": "Hand not connected",
            }
        return self._api().pause_realtime()

    def snapshot_pose(
        self,
        joint_angles: dict[str, float],
    ) -> dict:
        if not self.is_connected():
            return {
                "success": False,
                "message": "Hand not connected",
            }
        return self._api().hold_snapshot(
            joint_angles
        )

    def debug_mapping(
        self,
        joint_angles: dict[str, float],
    ) -> dict:
        return self._api().preview_joint_targets(
            joint_angles,
            read_actual=True,
        )

    def set_speed(self, speed: int) -> dict:
        self.speed = max(1, int(speed))
        return self._api().configure_motion(
            speed=self.speed
        )

    def emergency_stop(self) -> dict:
        if not self.is_connected():
            return {
                "success": False,
                "message": "Hand not connected",
            }
        return self._api().emergency_stop()

    def get_status(self) -> dict:
        try:
            return self._api().get_status()
        except Exception as exc:
            return {
                "connected": False,
                "last_error": str(exc),
            }

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
        result = {}
        for name, angle in angles.items():
            previous = self._prev.get(
                name,
                angle,
            )
            value = (
                self.alpha * angle
                + (1.0 - self.alpha) * previous
            )
            result[name] = value
            self._prev[name] = value
        return result

    def reset(self) -> None:
        self._prev.clear()


def print_mapping_debug(debug: dict) -> None:
    if not debug.get("success", False):
        print(f"[Debug] 失败: {debug}")
        return

    print()
    print(
        "Joint          ID  Angle   Target  Actual   Delta  Reverse"
    )
    print("-" * 66)

    joints = debug["joints"]
    for joint_name in sorted(joints):
        item = joints[joint_name]
        actual = item["actual_raw"]
        delta = item["delta_raw"]

        actual_text = (
            "N/A"
            if actual is None
            else str(actual)
        )
        delta_text = (
            "N/A"
            if delta is None
            else f"{delta:+d}"
        )

        print(
            f"{joint_name:13s} "
            f"{item['motor_id']:2d} "
            f"{item['angle_deg']:7.1f} "
            f"{item['target_raw']:7d} "
            f"{actual_text:>7s} "
            f"{delta_text:>7s} "
            f"{str(item['reversed']):>7s}"
        )

    print(
        "[Debug] motion=",
        debug.get("motion", {}),
    )
    print()


def main(
    connect_robot: bool,
    camera_id: int,
    config_path: str | None,
    calibration_path: str | None,
    speed: int,
    acc: int,
    torque: int,
    control_hz: float,
    ema_alpha: float,
) -> None:
    print("=" * 52)
    print(" Stimulated Hand — 手部动作模仿")
    print("=" * 52)
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
    smoother = EMASmoother(
        alpha=ema_alpha
    )
    bridge = RobotBridge(
        config_path=config_path,
        calibration_path=calibration_path,
        speed=speed,
        acc=acc,
        torque=torque,
        control_hz=control_hz,
    )

    robot_available = (
        bridge.connect()
        if connect_robot
        else False
    )
    robot_enabled = robot_available
    snapshot_hold = False
    snapshot_angles: Optional[dict[str, float]] = None
    show_detail = False

    print()
    print("按键:")
    print("  E       实时跟随 / 暂停保持")
    print("  P/Space 截取当前姿态并固定")
    print("  D       打印方向和 raw 映射")
    print("  X       急停并关闭扭矩")
    print("  [ / ]   降低 / 提高 speed")
    print("  S       打印状态")
    print("  T       简洁 / 详细显示")
    print("  Q/ESC   退出")
    print()

    fps_count = 0
    fps_last_time = time.time()
    fps_value = 0.0
    joint_angles: Optional[
        dict[str, float]
    ] = None

    last_hand_time = time.monotonic()
    no_hand_timeout = 0.5
    no_hand_stopped = False

    try:
        while True:
            success, frame = tracker.get_frame()
            if not success:
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
                    and not snapshot_hold
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
                result = bridge.emergency_stop()
                robot_enabled = False
                snapshot_hold = False
                snapshot_angles = None
                no_hand_stopped = True
                print(
                    "[Safety] 丢手超过 0.5 秒，"
                    f"已急停: {result}"
                )

            fps_count += 1
            if fps_count >= 30:
                now = time.time()
                fps_value = (
                    30.0
                    / (now - fps_last_time + 1e-9)
                )
                fps_last_time = now
                fps_count = 0

            status = {
                "robot_connected":
                    bridge.is_connected(),
                "robot_enabled":
                    robot_enabled,
                "snapshot_hold":
                    snapshot_hold,
                "fps": fps_value,
                "show_detail": show_detail,
                "speed": bridge.speed,
                "ema_alpha": smoother.alpha,
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
                        robot_enabled = True
                        snapshot_hold = False
                        snapshot_angles = None
                        smoother.reset()
                        print(
                            "[Action] 实时跟随已开启"
                        )
                elif robot_enabled:
                    result = bridge.pause_hold()
                    robot_enabled = False
                    snapshot_hold = False
                    snapshot_angles = None
                    print(
                        "[Action] 实时跟随已暂停；"
                        f"保持最后目标: {result}"
                    )
                else:
                    robot_enabled = True
                    snapshot_hold = False
                    snapshot_angles = None
                    smoother.reset()
                    print(
                        "[Action] 实时跟随已恢复"
                    )

            elif key in (ord("p"), ord(" ")):
                if not bridge.is_connected():
                    print(
                        "[Snapshot] 机械手未连接"
                    )
                elif not joint_angles:
                    print(
                        "[Snapshot] 当前未检测到手"
                    )
                else:
                    snapshot_angles = dict(joint_angles)
                    result = bridge.snapshot_pose(
                        snapshot_angles
                    )
                    if result.get("success", False):
                        robot_enabled = False
                        snapshot_hold = True
                        print(
                            "[Snapshot] 已固定当前姿态"
                        )
                        print(
                            "[Snapshot] target_raw=",
                            result.get("target_raw"),
                        )
                    else:
                        print(
                            f"[Snapshot] 失败: {result}"
                        )

            elif key == ord("d"):
                debug_angles = (
                    snapshot_angles
                    if snapshot_hold
                    else joint_angles
                )

                if not debug_angles:
                    print(
                        "[Debug] 当前没有可分析的姿态"
                    )
                else:
                    source = (
                        "snapshot"
                        if snapshot_hold
                        else "live"
                    )
                    print(
                        f"[Debug] source={source}"
                    )
                    debug = bridge.debug_mapping(
                        dict(debug_angles)
                    )
                    print_mapping_debug(debug)

            elif key == ord("x"):
                result = bridge.emergency_stop()
                robot_enabled = False
                snapshot_hold = False
                snapshot_angles = None
                print(
                    f"[Emergency Stop] {result}"
                )

            elif key == ord("["):
                config = bridge.set_speed(
                    bridge.speed - 10
                )
                print(
                    "[Motion] speed ↓",
                    config,
                )

            elif key == ord("]"):
                config = bridge.set_speed(
                    bridge.speed + 10
                )
                print(
                    "[Motion] speed ↑",
                    config,
                )

            elif key == ord("s"):
                print(
                    "[Status]",
                    bridge.get_status(),
                )
                if joint_angles:
                    print(
                        "[Status] 当前视觉关节角:"
                    )
                    for name in sorted(
                        joint_angles
                    ):
                        print(
                            f"  {name:13s}: "
                            f"{joint_angles[name]:7.2f}°"
                        )

            elif key == ord("t"):
                show_detail = not show_detail
                print(
                    "[Display]",
                    "详细"
                    if show_detail
                    else "简洁",
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
        description="Stimulated Hand"
    )
    parser.add_argument(
        "--robot",
        action="store_true",
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
    )
    parser.add_argument(
        "--calibration",
        type=str,
        default=None,
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=20,
        help="电机速度字段，建议从 20、30、40 逐步测试",
    )
    parser.add_argument(
        "--acc",
        type=int,
        default=10,
        help="加速度字段 0~254",
    )
    parser.add_argument(
        "--torque",
        type=int,
        default=100,
        help="扭矩限制字段 0~1000",
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=20.0,
        help="视觉目标下发频率，不等于电机物理速度",
    )
    parser.add_argument(
        "--ema-alpha",
        type=float,
        default=0.3,
        help="视觉平滑系数；越大响应越快、抖动越明显",
    )
    args = parser.parse_args()

    main(
        connect_robot=args.robot,
        camera_id=args.camera,
        config_path=args.config,
        calibration_path=args.calibration,
        speed=args.speed,
        acc=args.acc,
        torque=args.torque,
        control_hz=args.control_hz,
        ema_alpha=args.ema_alpha,
    )
