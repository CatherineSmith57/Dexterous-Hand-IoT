"""
vision_demo.py --- 上层视觉识别接入示例

这里不包含具体视觉模型，只展示输出格式和调用位置。
真正使用时，把 fake_vision_output() 替换成 MediaPipe、
神经网络或你们自己的手势/关键点识别模块。
"""

from __future__ import annotations

import math
import time
from typing import Dict, Tuple

from hand_interface import (
    cleanup,
    get_status,
    init,
    update_from_vision,
)


def fake_vision_output(t: float) -> Tuple[Dict[str, float], float]:
    """构造一帧示例关节角；真实项目中由视觉模型替换。"""
    bend = 45.0 + 30.0 * math.sin(t)

    angles = {
        "wrist": 0.0,
        "index_pip": bend,
        "index_mcp": bend,
        "index_abd": 0.0,
        "middle_abd": 0.0,
        "ring_abd": 0.0,
        "ring_mcp": bend,
        "ring_pip": bend,
        "middle_pip": bend,
        "middle_mcp": bend,
        "pinky_pip": bend,
        "pinky_mcp": bend,
        "pinky_abd": 0.0,
        "thumb_abd": 20.0,
        "thumb_mcp": bend * 0.7,
        "thumb_dip": bend * 0.7,
        "thumb_cmc": 20.0,
    }
    return angles, 0.95


def main() -> None:
    init()

    try:
        start = time.monotonic()

        while True:
            elapsed = time.monotonic() - start
            joint_angles, confidence = (
                fake_vision_output(elapsed)
            )

            sequence = update_from_vision(
                joint_angles,
                confidence=confidence,
                min_confidence=0.6,
                require_all=True,
            )

            if sequence is not None and sequence % 20 == 0:
                status = get_status(
                    refresh_hardware=False
                )
                print(
                    "submitted=",
                    status["statistics"][
                        "submitted_frames"
                    ],
                    "sent=",
                    status["statistics"]["sent_frames"],
                    "dropped=",
                    status["statistics"][
                        "dropped_frames"
                    ],
                )

            # 模拟 30 FPS 视觉输出；调度器按 CONTROL_HZ 下发。
            time.sleep(1.0 / 30.0)

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        cleanup()


if __name__ == "__main__":
    main()
