"""只检查 Python 接口形状，不连接真实机械手。"""

from __future__ import annotations

import inspect

import hand_interface


EXPECTED = {
    "init": ["config_path", "calibration_path"],
    "do_gesture": [
        "gesture_name",
        "hold_time_sec",
        "return_to_neutral",
    ],
    "get_status": [],
    "cleanup": [],
    "do_joint_command": [
        "joint_positions",
        "hold_time_sec",
    ],
}


def main() -> None:
    for name, expected_params in EXPECTED.items():
        function = getattr(hand_interface, name)
        actual = list(
            inspect.signature(function).parameters
        )
        assert actual == expected_params, (
            f"{name}: expected {expected_params}, "
            f"got {actual}"
        )
        print(f"[OK] {name}{inspect.signature(function)}")

    print("Interface contract check passed.")


if __name__ == "__main__":
    main()
