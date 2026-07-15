"""hand_tracker.py — MediaPipe 封装（新版 tasks API）

用摄像头捕获画面，使用 MediaPipe Hands 检测手部的 21 个关键点。

依赖：需要 hand_landmarker.task 模型文件（已在 models/ 目录下）。
      如缺失，运行时会自动提示下载地址。

用法:
    from stimulated_hand.hand_tracker import HandTracker

    tracker = HandTracker()
    frame = tracker.get_frame()              # 读取一帧
    landmarks = tracker.detect(frame)         # 检测手部关键点
    annotated = tracker.draw_landmarks(frame, hands)  # 画骨骼线
"""

import os
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.core import base_options as mp_base_options
from typing import Optional

# 模型文件路径（相对于本文件所在目录）
_MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
_MODEL_PATH = os.path.join(_MODEL_DIR, "hand_landmarker.task")


class HandTracker:
    """MediaPipe 手部追踪器（基于新版 tasks API）。

    打开默认摄像头，实时检测手部的 21 个关键点。

    21 个关键点编号：
        0=手腕, 1-4=拇指(CMC,MCP,IP,TIP),
        5-8=食指(MCP,PIP,DIP,TIP),
        9-12=中指(MCP,PIP,DIP,TIP),
        13-16=无名指(MCP,PIP,DIP,TIP),
        17-20=小指(MCP,PIP,DIP,TIP)
    """

    # 骨骼连接线定义（MediaPipe 标准连接）
    CONNECTIONS = [
        # 拇指
        (0, 1), (1, 2), (2, 3), (3, 4),
        # 食指
        (0, 5), (5, 6), (6, 7), (7, 8),
        # 中指
        (0, 9), (9, 10), (10, 11), (11, 12),
        # 无名指
        (0, 13), (13, 14), (14, 15), (15, 16),
        # 小指
        (0, 17), (17, 18), (18, 19), (19, 20),
        # 手掌根部横线
        (5, 9), (9, 13), (13, 17),
    ]

    def __init__(
        self,
        camera_id: int = 0,
        max_hands: int = 1,
        min_detection_confidence: float = 0.7,
        min_tracking_confidence: float = 0.5,
    ):
        """初始化摄像头和 MediaPipe HandLandmarker。

        Args:
            camera_id: 摄像头设备编号，默认 0 是系统默认摄像头
            max_hands: 最多检测几只手，默认 1
            min_detection_confidence: 检测置信度阈值（0~1）
            min_tracking_confidence: 追踪置信度阈值（0~1）
        """
        # 打开摄像头
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"无法打开摄像头 (ID={camera_id})。"
                f"请检查摄像头是否被其他程序占用，或尝试 camera_id=1。"
            )

        # 检查模型文件
        if not os.path.exists(_MODEL_PATH):
            raise RuntimeError(
                f"MediaPipe 手部模型文件未找到：{_MODEL_PATH}\n"
                f"请下载模型：\n"
                f"  curl -L https://storage.googleapis.com/mediapipe-models/"
                f"hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
                f" -o {_MODEL_PATH}"
            )

        # 初始化 HandLandmarker（新版 tasks API）
        options = vision.HandLandmarkerOptions(
            base_options=mp_base_options.BaseOptions(
                model_asset_path=_MODEL_PATH
            ),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
            min_hand_presence_confidence=0.5,
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self._timestamp_ms = 0  # 用于 VIDEO 模式的递增时间戳

    def get_frame(self) -> tuple[bool, np.ndarray]:
        """从摄像头读取一帧。

        Returns:
            (success, frame): success 表示是否成功读取，
            frame 是 BGR 图像（已水平翻转，呈镜像效果）
        """
        success, frame = self.cap.read()
        if success:
            # 水平翻转（镜像效果，让用户感觉像照镜子）
            frame = cv2.flip(frame, 1)
        return success, frame

    def detect(self, frame: np.ndarray) -> Optional[list[dict]]:
        """检测画面中的手部关键点。

        Args:
            frame: BGR 图像（来自 get_frame()）

        Returns:
            检测到的手部列表。每只手是一个 dict：
                {
                    "landmarks": [{"x": 0.5, "y": 0.3, "z": 0.01}, ...],  # 21 个
                    "handedness": "Left" | "Right",
                    "score": 0.95
                }
            没检测到手则返回 None。
        """
        # BGR → RGB → MediaPipe Image
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # 检测（VIDEO 模式需要时间戳）
        self._timestamp_ms += 33  # 约 30fps，每帧 ~33ms
        result = self.landmarker.detect_for_video(mp_image, self._timestamp_ms)

        if not result.hand_landmarks or len(result.hand_landmarks) == 0:
            return None

        hands = []
        for i, landmarks_list in enumerate(result.hand_landmarks):
            # 提取 21 个关键点的 xyz 坐标
            landmarks = []
            for lm in landmarks_list:
                landmarks.append({
                    "x": lm.x if lm.x is not None else 0.0,
                    "y": lm.y if lm.y is not None else 0.0,
                    "z": lm.z if lm.z is not None else 0.0,
                })

            # 左右手信息
            handedness = "Unknown"
            score = 1.0
            if result.handedness and i < len(result.handedness):
                cats = result.handedness[i]
                if cats:
                    handedness = cats[0].category_name or "Unknown"
                    score = cats[0].score

            hands.append({
                "landmarks": landmarks,
                "handedness": handedness,
                "score": score,
            })

        return hands

    def draw_landmarks(
        self,
        frame: np.ndarray,
        hands: Optional[list[dict]],
    ) -> np.ndarray:
        """在图像上画手部骨骼线和关键点。

        Args:
            frame: 原始 BGR 图像
            hands: detect() 返回的手部列表，或 None

        Returns:
            带有骨骼线标注的图像（新副本，不修改原图）
        """
        annotated = frame.copy()

        if not hands:
            return annotated

        h, w = annotated.shape[:2]

        for hand in hands:
            landmarks = hand["landmarks"]

            # 转为像素坐标
            points = {}
            for i, lm in enumerate(landmarks):
                px = int(lm["x"] * w)
                py = int(lm["y"] * h)
                points[i] = (px, py)

            # 画骨骼连线（绿色）
            for start, end in self.CONNECTIONS:
                if start in points and end in points:
                    cv2.line(annotated, points[start], points[end],
                             (0, 255, 0), 2, cv2.LINE_AA)

            # 画关键点（青色小圆）
            for i, (px, py) in points.items():
                radius = 5 if i == 0 else 3  # 手腕稍大
                cv2.circle(annotated, (px, py), radius,
                           (255, 200, 0), -1, cv2.LINE_AA)

            # 显示左右手标签和置信度
            wrist = points.get(0)
            if wrist:
                label = f"{hand['handedness']} ({hand['score']:.0%})"
                cv2.putText(annotated, label,
                            (wrist[0] - 30, wrist[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return annotated

    def release(self):
        """释放摄像头和 MediaPipe 资源。"""
        self.cap.release()
        self.landmarker.close()
