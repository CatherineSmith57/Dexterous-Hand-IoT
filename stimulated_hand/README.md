# Stimulated Hand — 手部动作模仿模块

## 功能

通过摄像头实时捕捉人手动作，映射为灵巧手（ORCA Hand）关节角度，驱动机械手跟随你做动作。

```
摄像头 → MediaPipe手部检测 → 关节角度映射 → OpenCV显示窗口
                                    ↓
                          scripts/hand_interface.py → 机械手
```

## 快速开始

### 1. 安装依赖

```bash
pip install -r stimulated_hand/requirements.txt
```

需要三个包：`mediapipe`（手部识别）、`opencv-python`（摄像头+显示）、`numpy`（数学计算）。

### 2. 运行

```bash
# 仅视觉追踪（不控制机械手）—— 推荐先这样跑，看效果
python stimulated_hand/main.py

# 视觉追踪 + 机械手控制
python stimulated_hand/main.py --robot

# 使用第二个摄像头
python stimulated_hand/main.py --camera 1
```

### 3. 按键操作

| 按键 | 功能 |
|------|------|
| **q** 或 **ESC** | 退出程序 |
| **e** | 切换机械手执行开关（连接/断开/暂停） |
| **s** | 打印机械手状态到控制台 |
| **t** | 切换显示模式（简洁 / 详细） |

## 文件说明

| 文件 | 职责 |
|------|------|
| `hand_tracker.py` | MediaPipe 封装：打开摄像头、检测手部21个关键点、画骨骼线 |
| `joint_mapper.py` | 核心映射算法：21个关键点 → 机械手17个关节角度 |
| `main.py` | 主循环 + OpenCV 显示窗口 + 机器人桥接 + EMA 平滑滤波 |
| `requirements.txt` | pip 依赖清单 |

## 工作原理

### 手部检测

MediaPipe 识别出手部的 21 个关键点：
```
0=手腕, 1-4=拇指, 5-8=食指, 9-12=中指, 13-16=无名指, 17-20=小指
```

### 关节映射

每根手指计算"弯曲程度"（0=完全伸直，1=完全弯曲），然后映射到机械手对应关节的 ROM（运动范围）。

例如：食指伸直 → `index_mcp = -60°`，食指完全弯曲 → `index_mcp = 100°`。

### 安全限制

- 所有角度限制在机械手 ROM 安全范围内
- 默认**不发送**到机械手，需按 **E 键**开启
- 动作经过 EMA 平滑滤波，减少抖动
- 每 3 帧发送一次命令，避免过度频繁

## 对队友代码的依赖

本模块只调用队友的 `scripts/hand_interface.py`：
- `init()` — 连接机械手
- `do_joint_command()` — 发送关节角度
- `get_status()` — 获取状态
- `cleanup()` — 断开连接

不直接调用 `third_party/orca_core` 的任何代码。

## 常见问题

**摄像头打不开？**
尝试 `--camera 1`，或检查是否有其他程序占用了摄像头。

**机械手连接失败？**
1. 确认机械手已上电
2. 确认 USB 串口已连接
3. 确认 `config_safe.yaml` 中的串口配置正确

**画面卡顿？**
1. 关掉其他占用 CPU 的程序
2. 确保光线充足（MediaPipe 在暗光下检测更慢）

**手指检测不准？**
1. 手掌正对摄像头，手指向上
2. 保证光线均匀，不要逆光
3. 背景尽量简单，避免其他肤色物体
