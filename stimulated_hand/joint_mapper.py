"""joint_mapper.py — 关节角度映射算法

将 MediaPipe 的 21 个手部关键点映射为机械手的 17 个关节角度。

核心思路：
    1. 对每根手指，计算"弯曲程度"（0=伸直，1=完全弯曲）
    2. 将弯曲程度映射到机械手对应关节的 ROM（运动范围）
    3. 所有输出角度都限制在安全 ROM 范围内

机械手 17 个关节（来自 hand_interface.py）：
    wrist, thumb_cmc, thumb_abd, thumb_mcp, thumb_dip,
    index_abd, index_mcp, index_pip,
    middle_abd, middle_mcp, middle_pip,
    ring_abd, ring_mcp, ring_pip,
    pinky_abd, pinky_mcp, pinky_pip

MediaPipe 21 个关键点：
    0=手腕, 1-4=拇指(CMC,MCP,IP,TIP), 5-8=食指(MCP,PIP,DIP,TIP),
    9-12=中指, 13-16=无名指, 17-20=小指
"""

import numpy as np
from typing import Optional


def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """计算两个向量之间的夹角（度）。

    Args:
        v1, v2: 三维向量（numpy array）

    Returns:
        夹角，范围 [0, 180]。180=方向相同，0=方向相反。
    """
    dot = np.dot(v1, v2)
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    if norm < 1e-9:
        return 180.0  # 向量长度接近 0，视为伸直
    cos_angle = np.clip(dot / norm, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def _three_point_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """计算三个点形成的角度 ∠ABC（B 是顶点）。

    例如：计算 PIP 关节弯曲角度时：
        a=MCP, b=PIP, c=DIP
        返回值 180° = 完全伸直，~90° = 完全弯曲

    Args:
        a, b, c: 三个三维坐标点（numpy array）

    Returns:
        ∠ABC 的角度（度），范围 [0, 180]
    """
    ba = a - b  # 向量 B→A（指向手掌方向）
    bc = c - b  # 向量 B→C（指向指尖方向）
    return _angle_between(ba, bc)


def _normalize_flexion(raw_angle: float) -> float:
    """将原始关节角度归一化为弯曲程度 [0, 1]。

    180°（完全伸直）→ 0.0
    90°（弯曲 90 度）→ 1.0

    Args:
        raw_angle: 三点角度（度），范围约 60~180

    Returns:
        弯曲程度，0.0=伸直，1.0=完全弯曲，超出则截断
    """
    flexion = (180.0 - raw_angle) / 90.0
    return float(np.clip(flexion, 0.0, 1.0))


def _to_xyz(lm: dict) -> np.ndarray:
    """将关键点字典转为 numpy 三维向量。

    Args:
        lm: {"x": 0.5, "y": 0.3, "z": 0.01}

    Returns:
        np.array([x, y, z])
    """
    return np.array([lm["x"], lm["y"], lm["z"]], dtype=np.float64)


class JointMapper:
    """手部关键点 → 机械手关节角度映射器。

    使用示例:
        mapper = JointMapper()
        angles = mapper.compute(landmarks)
        # angles = {"thumb_mcp": 45.0, "index_mcp": 60.0, ...}
    """

    # 各关节 ROM（运动范围），来自 hand_interface.py
    # 格式：{关节名: (最小值, 最大值)}，单位：度
    ROM_LIMITS = {
        "wrist":        (-65, 35),
        "thumb_cmc":    (-45, 33),
        "thumb_abd":    (-18, 55),
        "thumb_mcp":    (-60, 90),
        "thumb_dip":    (-55, 107),
        "index_abd":    (-30, 25),
        "index_mcp":    (-60, 100),
        "index_pip":    (-15, 107),
        "middle_abd":   (-27, 27),
        "middle_mcp":   (-60, 100),
        "middle_pip":   (-15, 107),
        "ring_abd":     (-27, 27),
        "ring_mcp":     (-60, 100),
        "ring_pip":     (-15, 107),
        "pinky_abd":    (-30, 30),
        "pinky_mcp":    (-60, 100),
        "pinky_pip":    (-15, 107),
    }

    # Neutral 位置（来自 config.yaml）
    NEUTRAL = {
        "wrist": 0.0,
        "thumb_cmc": 0.0, "thumb_abd": 50.0, "thumb_mcp": 33.0, "thumb_dip": 18.0,
        "index_abd": 0.0, "index_mcp": 2.0, "index_pip": 6.0,
        "middle_abd": 0.0, "middle_mcp": 2.0, "middle_pip": 4.0,
        "ring_abd": 0.0, "ring_mcp": -2.0, "ring_pip": 8.0,
        "pinky_abd": 0.0, "pinky_mcp": -4.0, "pinky_pip": -2.0,
    }

    # 四根手指的定义：{前缀: (mcp_idx, pip_idx, dip_idx, tip_idx)}
    FINGERS = {
        "index":  (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring":   (13, 14, 15, 16),
        "pinky":  (17, 18, 19, 20),
    }

    # 拇指定义
    THUMB = {"cmc": 1, "mcp": 2, "ip": 3, "tip": 4}

    def __init__(self, rom_limits: Optional[dict] = None):
        """初始化映射器。

        Args:
            rom_limits: 自定义 ROM 范围。None 则使用默认值。
        """
        self.rom = rom_limits if rom_limits else self.ROM_LIMITS

    def compute(self, landmarks: list[dict]) -> dict[str, float]:
        """主入口：将 21 个关键点映射为 17 个关节角度。

        Args:
            landmarks: 21 个关键点的列表，每个是 {"x", "y", "z"}

        Returns:
            关节角度字典，键=关节名，值=角度（度）
        """
        result = {}

        # 1. 四根手指的 MCP 和 PIP 弯曲
        for prefix, (mcp, pip, dip, tip) in self.FINGERS.items():
            # MCP 弯曲：手腕→MCP→PIP 的角度
            raw_mcp = _three_point_angle(
                _to_xyz(landmarks[0]),   # 手腕
                _to_xyz(landmarks[mcp]), # MCP
                _to_xyz(landmarks[pip]), # PIP
            )
            flex_mcp = _normalize_flexion(raw_mcp)

            # PIP 弯曲：MCP→PIP→DIP 的角度
            raw_pip = _three_point_angle(
                _to_xyz(landmarks[mcp]),
                _to_xyz(landmarks[pip]),
                _to_xyz(landmarks[dip]),
            )
            flex_pip = _normalize_flexion(raw_pip)

            # 映射到 ROM
            result[f"{prefix}_mcp"] = self._map_to_rom(f"{prefix}_mcp", flex_mcp)
            result[f"{prefix}_pip"] = self._map_to_rom(f"{prefix}_pip", flex_pip)

        # 2. 拇指
        thumb = self._compute_thumb(landmarks)
        result.update(thumb)

        # 3. 外展（手指展开/并拢程度）
        abd = self._compute_abduction(landmarks)
        result.update(abd)

        # 4. 手腕（第一版保持不动）
        result["wrist"] = self.NEUTRAL["wrist"]

        # 5. 安全截断（确保不超出 ROM）
        return self._clamp_all(result)

    def _compute_thumb(self, landmarks: list[dict]) -> dict:
        """计算拇指的 4 个关节角度。

        拇指结构与人不同，需要特殊处理：
        - thumb_cmc：手腕→CMC→MCP
        - thumb_mcp：CMC→MCP→IP
        - thumb_dip：MCP→IP→TIP
        - thumb_abd：拇指尖偏离手掌平面的角度
        """
        wrist = _to_xyz(landmarks[0])
        cmc = _to_xyz(landmarks[1])
        mcp = _to_xyz(landmarks[2])
        ip = _to_xyz(landmarks[3])
        tip = _to_xyz(landmarks[4])

        # CMC 弯曲
        raw_cmc = _three_point_angle(wrist, cmc, mcp)
        flex_cmc = _normalize_flexion(raw_cmc)

        # MCP 弯曲
        raw_mcp = _three_point_angle(cmc, mcp, ip)
        flex_mcp = _normalize_flexion(raw_mcp)

        # IP 弯曲（对应机械手的 thumb_dip）
        raw_dip = _three_point_angle(mcp, ip, tip)
        flex_dip = _normalize_flexion(raw_dip)

        # 外展：拇指尖相对于手掌平面的偏离
        # 用手掌平面法向量（食指MCP、小指MCP、手腕构成的平面）
        index_mcp = _to_xyz(landmarks[5])
        pinky_mcp = _to_xyz(landmarks[17])
        palm_v1 = index_mcp - wrist
        palm_v2 = pinky_mcp - wrist
        palm_normal = np.cross(palm_v1, palm_v2)
        thumb_vec = tip - wrist

        # 拇指向量与手掌法向量的夹角
        if np.linalg.norm(palm_normal) > 1e-9 and np.linalg.norm(thumb_vec) > 1e-9:
            abd_angle = _angle_between(thumb_vec, palm_normal)
            # 归一化到 0~1（90 度最张，0 度贴掌心）
            flex_abd = float(np.clip(abd_angle / 90.0, 0.0, 1.0))
        else:
            flex_abd = 0.5

        return {
            "thumb_cmc": self._map_to_rom("thumb_cmc", flex_cmc),
            "thumb_mcp": self._map_to_rom("thumb_mcp", flex_mcp),
            "thumb_dip": self._map_to_rom("thumb_dip", flex_dip),
            "thumb_abd": self._map_to_rom("thumb_abd", flex_abd),
        }

    def _compute_abduction(self, landmarks: list[dict]) -> dict:
        """计算手指外展角度（手指张开/并拢程度）。

        以中指为参考（abduction=0），计算其他手指根部相对于中指的角度。
        """
        wrist = _to_xyz(landmarks[0])

        # 各手指 MCP 相对于手腕的方向（只取 x,y 平面）
        def mcp_dir(idx):
            v = _to_xyz(landmarks[idx]) - wrist
            return np.array([v[0], v[1]], dtype=np.float64)

        dir_index = mcp_dir(5)
        dir_middle = mcp_dir(9)
        dir_ring = mcp_dir(13)
        dir_pinky = mcp_dir(17)

        # 计算各手指相对于中指的角度（度）
        ref_angle = np.arctan2(dir_middle[1], dir_middle[0])

        def relative_angle(d):
            a = np.arctan2(d[1], d[0])
            return float(np.degrees(a - ref_angle))

        # 原始相对角度（正值=在参考方向的一侧）
        raw_index = relative_angle(dir_index)
        raw_ring = relative_angle(dir_ring)
        raw_pinky = relative_angle(dir_pinky)

        # 归一化：约为 ±25 度的范围
        # 手指自然并拢时接近 0，张开时绝对值变大
        abd_range = 30.0  # 假设最大展开 ~30 度

        flex_index = float(np.clip(raw_index / abd_range, -1.0, 1.0))
        flex_ring = float(np.clip(raw_ring / abd_range, -1.0, 1.0))
        flex_pinky = float(np.clip(raw_pinky / abd_range, -1.0, 1.0))

        return {
            "index_abd": self._map_to_rom("index_abd", flex_index),
            "middle_abd": 0.0,  # 参考手指，不偏移
            "ring_abd": self._map_to_rom("ring_abd", flex_ring),
            "pinky_abd": self._map_to_rom("pinky_abd", flex_pinky),
        }

    def _map_to_rom(self, joint_name: str, flexion: float) -> float:
        """将弯曲程度 [0,1] 映射到关节 ROM 范围内的角度。

        flexion=0 → ROM 最小值（手指伸直）
        flexion=1 → ROM 最大值（手指完全弯曲）

        Args:
            joint_name: 关节名（如 "index_mcp"）
            flexion: 弯曲程度，0=伸直，1=弯曲

        Returns:
            关节角度（度）
        """
        lo, hi = self.rom.get(joint_name, (0, 100))
        return lo + flexion * (hi - lo)

    def _clamp_all(self, angles: dict) -> dict:
        """将所有角度截断到对应的 ROM 范围内。"""
        clamped = {}
        for name, angle in angles.items():
            lo, hi = self.rom.get(name, (-999, 999))
            clamped[name] = float(np.clip(angle, lo, hi))
        return clamped
