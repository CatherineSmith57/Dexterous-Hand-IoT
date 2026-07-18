r"""固定手势视频展示程序。

交互模式：
    python -m flexible_hand.fixed_gesture_demo ^
        --calibration .\flexible_hand\hardware\calibration.yaml

自动展示：
    python -m flexible_hand.fixed_gesture_demo ^
        --calibration .\flexible_hand\hardware\calibration.yaml ^
        --sequence

单个手势：
    python -m flexible_hand.fixed_gesture_demo ^
        --calibration .\flexible_hand\hardware\calibration.yaml ^
        --gesture yeah
"""

from __future__ import annotations

import argparse
import time

try:
    from . import hand_interface
    from .fixed_gestures import (
        GESTURE_LABELS,
        FIXED_GESTURES,
        list_fixed_gestures,
    )
    from .gesture_api import execute_fixed_gesture
except ImportError:
    import hand_interface
    from fixed_gestures import (
        GESTURE_LABELS,
        FIXED_GESTURES,
        list_fixed_gestures,
    )
    from gesture_api import execute_fixed_gesture


DEFAULT_SEQUENCE = (
    "hi",
    "yeah",
    "six",
    "point",
    "fist",
    "hi",
)


def _configure_motion(
    speed: int,
    acc: int,
    torque: int,
    control_hz: float,
) -> None:
    configure = getattr(
        hand_interface,
        "configure_motion",
        None,
    )
    if callable(configure):
        result = configure(
            speed=speed,
            acc=acc,
            torque=torque,
            control_hz=control_hz,
        )
        print("[Motion]", result)
    else:
        print(
            "[Motion] 当前 hand_interface 不支持 "
            "configure_motion()，使用其内部默认参数"
        )


def _print_menu() -> None:
    print()
    print("固定手势：")
    for index, (name, label) in enumerate(
        list_fixed_gestures().items(),
        start=1,
    ):
        print(f"  {index} — {label} ({name})")
    print("  a — 自动播放全部手势")
    print("  x — 急停并关闭扭矩")
    print("  q — 退出")
    print()


def _show(
    gesture_name: str,
    hold_seconds: float,
) -> bool:
    label = GESTURE_LABELS[gesture_name]
    print(f"\n[Gesture] {label}")

    result = execute_fixed_gesture(
        gesture_name
    )
    print("[Result]", result)

    success = (
        result.get("success") is True
        or result.get("status")
        in {"completed", "snapshot_hold"}
    )
    if not success:
        print(
            "[Gesture] 执行失败，停止当前流程"
        )
        return False

    if hold_seconds > 0:
        time.sleep(hold_seconds)
    return True


def _run_sequence(
    hold_seconds: float,
    transition_seconds: float,
) -> None:
    print(
        "[Sequence] 开始自动展示：",
        " → ".join(DEFAULT_SEQUENCE),
    )

    for index, gesture_name in enumerate(
        DEFAULT_SEQUENCE
    ):
        if not _show(
            gesture_name,
            hold_seconds,
        ):
            return

        # 非 Hi 姿态之间插入短暂 Hi 过渡，
        # 让视频动作更整齐，也减少突然跳变。
        next_name = (
            DEFAULT_SEQUENCE[index + 1]
            if index + 1 < len(DEFAULT_SEQUENCE)
            else None
        )
        if (
            next_name is not None
            and gesture_name != "hi"
            and next_name != "hi"
        ):
            if not _show(
                "hi",
                transition_seconds,
            ):
                return

    print("[Sequence] 完成")


def _interactive(
    hold_seconds: float,
    transition_seconds: float,
) -> None:
    key_to_name = {
        str(index): name
        for index, name in enumerate(
            FIXED_GESTURES,
            start=1,
        )
    }

    _print_menu()

    while True:
        try:
            command = input(
                "输入手势编号/命令 > "
            ).strip().lower()
        except EOFError:
            command = "q"

        if command in {"q", "quit", "exit"}:
            return

        if command in {"a", "all", "sequence"}:
            _run_sequence(
                hold_seconds=hold_seconds,
                transition_seconds=transition_seconds,
            )
            continue

        if command in {"x", "stop"}:
            emergency_stop = getattr(
                hand_interface,
                "emergency_stop",
                None,
            )
            if callable(emergency_stop):
                print(
                    "[Emergency Stop]",
                    emergency_stop(),
                )
            else:
                print(
                    "[Emergency Stop] 当前接口无 "
                    "emergency_stop()"
                )
            continue

        gesture_name = key_to_name.get(
            command,
            command,
        )
        try:
            _show(
                gesture_name,
                hold_seconds=0.0,
            )
            print(
                "[Hold] 当前姿态保持中；"
                "输入下一个手势即可切换"
            )
        except KeyError as exc:
            print(f"[Input] {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="机械手固定手势视频展示"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
    )
    parser.add_argument(
        "--calibration",
        type=str,
        required=True,
        help="人工 motor_limits YAML 路径",
    )
    parser.add_argument(
        "--gesture",
        choices=tuple(FIXED_GESTURES),
        default=None,
        help="只展示一个固定手势",
    )
    parser.add_argument(
        "--sequence",
        action="store_true",
        help="自动播放五个固定手势",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=2.5,
        help="自动模式下每个手势保持秒数",
    )
    parser.add_argument(
        "--transition-hold",
        type=float,
        default=1.0,
        help="过渡 Hi 姿态保持秒数",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=40,
    )
    parser.add_argument(
        "--acc",
        type=int,
        default=15,
    )
    parser.add_argument(
        "--torque",
        type=int,
        default=100,
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=20.0,
    )
    args = parser.parse_args()

    print("=" * 54)
    print("  Fixed Gesture Showcase")
    print("=" * 54)

    _configure_motion(
        speed=args.speed,
        acc=args.acc,
        torque=args.torque,
        control_hz=args.control_hz,
    )

    ok = hand_interface.init(
        config_path=args.config,
        calibration_path=args.calibration,
    )
    if not ok:
        status = hand_interface.get_status()
        print(
            "[Init] 失败:",
            status.get(
                "last_error",
                "unknown error",
            ),
        )
        return 1

    print("[Init] 机械手已连接并回中")

    try:
        # 先张开，防止从未知姿态直接进入复杂手势。
        if not _show(
            "hi",
            hold_seconds=1.0,
        ):
            return 1

        if args.gesture is not None:
            if args.gesture != "hi":
                if not _show(
                    args.gesture,
                    hold_seconds=args.hold,
                ):
                    return 1
            print(
                "[Hold] 手势保持完成，按 Enter 退出"
            )
            input()
            return 0

        if args.sequence:
            _run_sequence(
                hold_seconds=args.hold,
                transition_seconds=(
                    args.transition_hold
                ),
            )
            print(
                "[Hold] 自动展示完成，按 Enter 退出"
            )
            input()
            return 0

        _interactive(
            hold_seconds=args.hold,
            transition_seconds=args.transition_hold,
        )
        return 0

    except KeyboardInterrupt:
        print("\n[Exit] 用户中断")
        return 130

    finally:
        print("[Cleanup] 关闭扭矩并释放串口")
        hand_interface.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
