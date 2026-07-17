# ORCA 灵巧手 ROS2 中间层 — hand_middle 开发手册

> **版本**: 0.1.0  
> **ROS2发行版**: Jazzy (ament_python)  
> **维护团队**: Hand Middleware Team  
> **最后更新**: 2026-07-16  
> **仓库地址**: https://github.com/CatherineSmith57/Dexterous-Hand-IoT

---

## 目录

1. [项目背景与目标](#1-项目背景与目标)
2. [架构总览](#2-架构总览)
3. [文件清单与路径](#3-文件清单与路径)
4. [通信接口规范](#4-通信接口规范)
5. [关键设计决策](#5-关键设计决策)
6. [开发环境搭建](#6-开发环境搭建)
7. [编译与运行](#7-编译与运行)
8. [完整终端操作命令](#8-完整终端操作命令)
9. [测试指南](#9-测试指南)
10. [异常处理与错误码](#10-异常处理与错误码)
11. [扩展指南 (给后续开发者)](#11-扩展指南-给后续开发者)
12. [常见问题 (FAQ)](#12-常见问题-faq)
13. [小组协作约定](#13-小组协作约定)
14. [附录: 完整源码索引](#14-附录-完整源码索引)

---

## 1. 项目背景与目标

### 1.1 项目定位

`hand_middle` 是 ORCA 灵巧手项目的 **ROS2 中间适配层**。它位于上层控制应用与底层硬件驱动库之间，负责：

- **指令翻译**: 将 ROS2 Service 调用转换为 `orca_core` 硬件库的 Python 方法调用
- **状态回传**: 以 10Hz 固定频率从硬件读取实时状态，封装为 ROS2 Topic 向外发布
- **异常容错**: 统一捕获串口断开、参数越界、读取超时等异常，分级记录日志

### 1.2 核心约束

> ⚠️ **铁律：绝不修改 `third_party/orca_core/` 下的任何文件。**

`orca_core` 由底层驱动团队维护，我们仅通过其公开的 Python API（`OrcaHand`、`OrcaStatus`、异常类）进行封装调用。

### 1.3 与其他包的关系

```
third_party/orca_core/        ← 底层驱动 (不可修改)
        ▲
        │ 封装调用
        │
ros2_ws/src/hand_middle/      ← 本项目 (中间层)
        ▲
        │ Service/Topic
        │
上层控制应用 (行为树/状态机/GUI)
```

> **注意**: 仓库中已存在 `ros2_ws/src/orca_hand_ctrl/` 包，那是早期原型（仅支持 `CloseAllFingers` 单一服务）。`hand_middle` 是其替代升级版，覆盖完整指令集。

---

## 2. 架构总览

```
                         ┌─────────────────────────────────────┐
                         │         上层控制应用                  │
                         │   (行为树 / 状态机 / GUI / CLI)       │
                         └──────┬──────────────┬───────────────┘
                                │ Service      │ Topic
                                ▼              ▼
              ┌─────────────────────┐   ┌─────────────────────┐
              │ /hand_middle/command│   │ /hand_middle/status  │
              │  HandCommand.srv    │   │  HandStatus.msg      │
              │  (单次指令下发)      │   │  (10Hz 持续发布)     │
              └────────┬────────────┘   └──────────┬──────────┘
                       │                           │
                       ▼                           ▼
              ┌─────────────────────────────────────────────┐
              │              hand_node.py                    │
              │  ┌──────────────┐  ┌────────────────────┐   │
              │  │ Service 回调  │  │ 10Hz 状态发布定时器 │   │
              │  │ _handle_cmd() │  │ _publish_status()  │   │
              │  └──────┬───────┘  └─────────┬──────────┘   │
              └─────────┼────────────────────┼──────────────┘
                        │                    │
                        ▼                    ▼
              ┌─────────────────────────────────────────────┐
              │              hand_bridge.py                  │
              │  ┌──────────────────────────────────────┐    │
              │  │ execute_command()  统一指令分发        │    │
              │  │  ├── joint_control  (关节精细控制)    │    │
              │  │  ├── gesture        (手势+幅度插值)   │    │
              │  │  ├── reset          (硬件复位)        │    │
              │  │  ├── enable         (电机使能)        │    │
              │  │  └── disable        (电机去使能)      │    │
              │  ├──────────────────────────────────────┤    │
              │  │ get_device_status() 扩展状态读取      │    │
              │  │  ├── orca_core 真实数据               │    │
              │  │  ├── 模拟温度/力矩 (待硬件替换)        │    │
              │  │  └── _detect_fault() 故障综合检测     │    │
              │  └──────────────────────────────────────┘    │
              └────────────────────┬────────────────────────┘
                                   │
                                   ▼
              ┌─────────────────────────────────────────────┐
              │          third_party/orca_core/              │
              │  ┌──────────┐ ┌──────────────┐ ┌─────────┐  │
              │  │OrcaHand  │ │ OrcaStatus   │ │ OrcaErr │  │
              │  │connect() │ │connected     │ │ 1001..  │  │
              │  │calibrate│ │calibrated    │ │or       │  │
              │  │execute_* │ │joint_positions│ │...     │  │
              │  │get_status│ │...           │ │         │  │
              │  └──────────┘ └──────────────┘ └─────────┘  │
              └─────────────────────────────────────────────┘
```

### 业务闭环

```
上层下发控制指令
    → /hand_middle/command 服务接收
    → HandBridge 封装调用 orca_core 串口硬件接口执行动作
    → 10Hz 定时器读取硬件实时状态
    → /hand_middle/status 话题向外发布
```

---

## 3. 文件清单与路径

```
ros2_ws/src/hand_middle/
├── package.xml                              # 功能包清单 (依赖声明)
├── setup.py                                 # ament_python 构建配置
├── setup.cfg                                # Python 安装路径配置
├── config/
│   └── params.yaml                          # 节点参数默认值
├── srv/
│   └── HandCommand.srv                      # [自定义Service] 统一指令下发
├── msg/
│   └── HandStatus.msg                       # [自定义Message] 10Hz 状态发布
├── launch/
│   └── hand_middle.launch.py                # 一键启动文件
├── hand_middle/
│   ├── __init__.py                          # 包初始化 + orca_core 路径注入
│   ├── hand_bridge.py                       # 中间适配层 (~460行)
│   ├── hand_node.py                         # ROS2 主控节点 (~300行)
│   ├── test_service_client.py               # Service 全指令测试 (~500行)
│   └── test_topic_subscriber.py             # Topic 订阅测试 (~280行)
└── HAND_MIDDLE_开发手册.md                   # ← 本文件
```

**总代码量**: ~2,500 行 Python + 87 行消息定义

---

## 4. 通信接口规范

### 4.1 Service: `/hand_middle/command`

**文件**: `srv/HandCommand.srv`

#### Request (指令下发)

| 字段 | 类型 | 说明 |
|------|------|------|
| `command_type` | `string` | 指令类型 (见下表) |
| `gesture_name` | `string` | 手势名 (仅 gesture 模式使用) |
| `amplitude` | `float32` | 开合幅度 0.0(全开) ~ 1.0(全闭) |
| `joint_names[10]` | `string[10]` | 目标关节名数组 (空串 = 跳过) |
| `joint_targets[10]` | `float32[10]` | 目标角度数组 (度) |
| `hold_time_sec` | `float32` | 动作保持时间 (秒) |
| `return_to_neutral` | `bool` | 动作完成后是否回中 |

**支持的 `command_type`**:

| 指令 | 说明 | 必须参数 |
|------|------|---------|
| `joint_control` | 精细关节角度控制 | `joint_names`, `joint_targets` |
| `gesture` | 预定义手势执行 | `gesture_name`, `amplitude` (可选) |
| `reset` | 硬件复位 (回零+重标定) | 无 |
| `enable` | 使能电机 (上电) | 无 |
| `disable` | 去使能电机 (安全锁定) | 无 |

**预定义手势 (`gesture_name`)**:

| 手势标签 (上层可用) | orca_core 内部名 | 说明 |
|---------------------|------------------|------|
| `open_palm` / `hand_open` | `hand_open` | 五指张开 |
| `fist` / `hand_close` | `hand_close` | 握拳 |
| `pinch` / `pinch_grasp` | `pinch_grasp` | 指尖捏合 |
| `two_finger` / `two_finger_pose` | `two_finger_pose` | 两指 |
| `point` / `point_pose` | `point_pose` | 食指指向 |

> 手势标签映射在 `hand_bridge.py` 的 `GESTURE_MAPPING` 字典中维护。

**10个关节 (固定顺序)**:

| 索引 | 关节名 | ROM (最小°/最大°) |
|------|--------|-------------------|
| 0 | `thumb_mcp` | -10 ~ 60 |
| 1 | `thumb_pip` | -10 ~ 90 |
| 2 | `index_mcp` | -5 ~ 90 |
| 3 | `index_pip` | -5 ~ 100 |
| 4 | `middle_mcp` | -5 ~ 90 |
| 5 | `middle_pip` | -5 ~ 100 |
| 6 | `ring_mcp` | -5 ~ 90 |
| 7 | `ring_pip` | -5 ~ 100 |
| 8 | `pinky_mcp` | -5 ~ 90 |
| 9 | `pinky_pip` | -5 ~ 100 |

#### Response (执行结果)

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 是否执行成功 |
| `execution_status` | `string` | `idle` / `received` / `running` / `completed` / `failed` / `aborted` |
| `error_code` | `int32` | 错误码 (0=成功, 其他见[错误码表](#10-异常处理与错误码)) |
| `error_message` | `string` | 错误描述 (成功时为空) |
| `timestamp` | `string` | ISO 8601 时间戳 (北京时间) |

### 4.2 Topic: `/hand_middle/status`

**文件**: `msg/HandStatus.msg`  
**发布频率**: 10Hz (可配置 `status_publish_rate`)  
**QoS**: Best Effort, Volatile, KeepLast(10)

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 标准 ROS2 消息头 (时间戳) |
| `connected` | `bool` | 串口是否已连接 |
| `calibrated` | `bool` | 是否已完成标定 |
| `motor_enabled` | `bool` | 电机是否使能 |
| `joint_names[10]` | `string[10]` | 关节名称数组 |
| `joint_positions[10]` | `float32[10]` | 关节角度 (°) |
| `joint_temperatures[10]` | `float32[10]` | 关节温度 (°C) *(当前为模拟值)* |
| `joint_torques[10]` | `float32[10]` | 关节力矩 (N·m) *(当前为模拟值)* |
| `fault_code` | `int32` | 故障码 (0=正常) |
| `fault_message` | `string` | 故障描述 |
| `current_action` | `string` | 当前执行动作名 |
| `execution_status` | `string` | 执行状态枚举 |

> ⚠️ **关于温度和力矩**: `orca_core` 当前版本未暴露温度/力矩传感器接口。`hand_bridge.py` 中使用模拟数据 + 随机波动填充这些字段。接入真实硬件后，替换 `_update_sim_sensors()` 方法中的实现即可，详见 [扩展指南](#11-扩展指南-给后续开发者)。

---

## 5. 关键设计决策

| 决策 | 说明 | 原因 |
|------|------|------|
| **固定 10 关节数组** | `string[10]` + `float32[10]` | 匹配 orca_core 固定关节数，避免动态数组在 ROS2 IDL 中的复杂性 |
| **温度/力矩占位** | 模拟数据 + 随机波动 | 消息接口已就绪，硬件就位后仅需替换一个方法 |
| **故障码共用体系** | `1-2` 传感器故障 + `1001-1008` 协议错误 | 统一 `fault_code` 字段，上游只需检查一个字段 |
| **开合幅度线性插值** | `angle = open + amplitude × (target - open)` | 每个关节独立插值，保证运动平滑自然 |
| **QoS Best Effort** | 状态话题不保证送达 | 10Hz 传感器流丢帧可接受，避免重传导致延迟累积 |
| **Service 同步等待** | 调用 `rclpy.spin_until_future_complete` | 指令执行需要确认结果，同步等待符合控制场景 |
| **不修改 orca_core** | 所有扩展在 bridge 层完成 | 底层驱动独立演进，中间层负责适配和增强 |

---

## 6. 开发环境搭建

### 6.1 前置条件

- Ubuntu 22.04 / Windows 11 WSL2
- ROS2 Jazzy (完整安装)
- Python 3.10+
- Git

### 6.2 从零搭建步骤

```bash
# 1. 克隆仓库
cd ~
git clone https://github.com/CatherineSmith57/Dexterous-Hand-IoT.git
cd Dexterous-Hand-IoT

# 2. 安装 ROS2 依赖
cd ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. 验证 orca_core 可导入
python3 -c "
import sys
sys.path.insert(0, 'third_party/orca_core')
from orca_core import OrcaHand
print('orca_core OK')
"

# 4. 首次编译 (全部包)
colcon build --symlink-install

# 5. 刷新环境
source install/setup.bash

# 6. 验证 hand_middle 可执行文件
ros2 pkg executables hand_middle
# 预期输出:
#   hand_node
#   test_service_client
#   test_topic_subscriber
```

### 6.3 WSL 串口配置 (Linux 侧)

```bash
# 检查 USB 串口设备
ls /dev/ttyUSB* /dev/ttyACM*

# 如果找不到设备，可能需要挂载 Windows COM 口
# 在 WSL 中: COM3 → /dev/ttyS3
# 参考: https://learn.microsoft.com/en-us/windows/wsl/connect-usb

# 添加当前用户到 dialout 组 (避免 sudo)
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

---

## 7. 编译与运行

### 7.1 编译

```bash
cd ~/Dexterous-Hand-IoT/ros2_ws

# 仅编译 hand_middle
colcon build --packages-select hand_middle --symlink-install

# 完整编译
colcon build --symlink-install

# 清理后重新编译
rm -rf build/ install/ log/
colcon build --symlink-install
```

> `--symlink-install`: Python 文件以符号链接安装，修改源码无需重新编译，直接重启节点即可生效。**开发阶段强烈建议使用。**

### 7.2 刷新环境

```bash
# 每次打开新终端都需要执行
source ~/Dexterous-Hand-IoT/ros2_ws/install/setup.bash

# 或者写入 ~/.bashrc (开发阶段推荐)
echo "source ~/Dexterous-Hand-IoT/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 7.3 启动主节点

```bash
# 方式 A: ros2 run (默认参数)
ros2 run hand_middle hand_node

# 方式 B: 自定义串口
ros2 run hand_middle hand_node --ros-args -p port:=/dev/ttyUSB0

# 方式 C: launch 一键启动
ros2 launch hand_middle hand_middle.launch.py

# 方式 D: launch + 全参数自定义
ros2 launch hand_middle hand_middle.launch.py \
    port:=/dev/ttyUSB1 \
    baudrate:=921600 \
    status_publish_rate:=20.0 \
    temperature_limit:=80.0 \
    torque_limit:=3.0 \
    auto_initialize:=true
```

---

## 8. 完整终端操作命令

### 8.1 服务调用

```bash
# ── 使能电机 ─────────────────────────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'enable', gesture_name: '', amplitude: 0.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 0.0, return_to_neutral: true}"

# ── 手势: 握拳 (全闭, amplitude=1.0) ─────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'gesture', gesture_name: 'hand_close', amplitude: 1.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 2.0, return_to_neutral: true}"

# ── 手势: 半握 (amplitude=0.5) ───────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'gesture', gesture_name: 'hand_close', amplitude: 0.5, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.5, return_to_neutral: true}"

# ── 手势: 五指张开 ────────────────────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'gesture', gesture_name: 'hand_open', amplitude: 1.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.5, return_to_neutral: true}"

# ── 手势: 指尖捏合 ────────────────────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'gesture', gesture_name: 'pinch_grasp', amplitude: 1.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.5, return_to_neutral: true}"

# ── 精细关节控制: 仅食指 ──────────────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'joint_control', gesture_name: '', amplitude: 0.0, \
    joint_names: ['index_mcp','index_pip','','','','','','','',''], \
    joint_targets: [45.0,60.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.5, return_to_neutral: true}"

# ── 精细关节控制: 全部手指 ────────────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'joint_control', gesture_name: '', amplitude: 0.0, \
    joint_names: ['thumb_mcp','thumb_pip','index_mcp','index_pip','middle_mcp','middle_pip','ring_mcp','ring_pip','pinky_mcp','pinky_pip'], \
    joint_targets: [30.0,50.0,45.0,60.0,45.0,60.0,45.0,60.0,45.0,60.0], \
    hold_time_sec: 1.5, return_to_neutral: true}"

# ── 硬件复位 ───────────────────────────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'reset', gesture_name: '', amplitude: 0.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 0.0, return_to_neutral: true}"

# ── 去使能电机 ─────────────────────────────────────────
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'disable', gesture_name: '', amplitude: 0.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 0.0, return_to_neutral: true}"
```

### 8.2 话题订阅

```bash
# 查看话题原始数据 (一次)
ros2 topic echo --once /hand_middle/status

# 持续监听
ros2 topic echo /hand_middle/status

# 查看发布频率
ros2 topic hz /hand_middle/status

# 查看话题信息
ros2 topic info /hand_middle/status

# 使用测试脚本 (详细模式)
ros2 run hand_middle test_topic_subscriber

# 使用测试脚本 (统计模式)
ros2 run hand_middle test_topic_subscriber --ros-args -p stats_mode:=true

# 使用测试脚本 (仅故障)
ros2 run hand_middle test_topic_subscriber --ros-args -p fault_only:=true
```

### 8.3 异常测试

```bash
# ROM 越界: thumb_mcp 最大 60°, 请求 999° → 预期 error_code=1005
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'joint_control', gesture_name: '', amplitude: 0.0, \
    joint_names: ['thumb_mcp','','','','','','','','',''], \
    joint_targets: [999.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.0, return_to_neutral: true}"

# 非法关节名 → 预期 error_code=1004
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'joint_control', gesture_name: '', amplitude: 0.0, \
    joint_names: ['elbow_joint','','','','','','','','',''], \
    joint_targets: [45.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.0, return_to_neutral: true}"

# 非法手势名 → 预期 error_code=1003
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'gesture', gesture_name: 'jazz_hands', amplitude: 1.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.0, return_to_neutral: true}"

# 幅度越界 → 预期 error_code=1005
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'gesture', gesture_name: 'hand_close', amplitude: 1.5, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 1.0, return_to_neutral: true}"

# 未知指令类型 → 预期 error_code=1003
ros2 service call /hand_middle/command hand_middle/srv/HandCommand \
  "{command_type: 'do_backflip', gesture_name: '', amplitude: 0.0, \
    joint_names: ['','','','','','','','','',''], \
    joint_targets: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], \
    hold_time_sec: 0.0, return_to_neutral: true}"
```

### 8.4 仿真模式启动 (无硬件)

当没有真实灵巧手硬件时，`orca_core` 使用模拟串口逻辑：

```bash
# 跳过自动初始化，节点启动但不连接设备
ros2 launch hand_middle hand_middle.launch.py auto_initialize:=false

# 启动后手动尝试初始化 (会失败，但节点保持运行)
# 此时可以测试异常处理路径和服务响应格式
```

---

## 9. 测试指南

### 9.1 自动化测试脚本

```bash
# ── 终端1: 启动服务端 ─────────────────────────────────
ros2 launch hand_middle hand_middle.launch.py

# ── 终端2: 全量测试 (所有场景: 正常流+手势+关节+复位+异常) ──
ros2 run hand_middle test_service_client

# ── 终端2: 仅测试正常流程 ──────────────────────────────
ros2 run hand_middle test_service_client --ros-args -p test_scenario:=normal

# ── 终端2: 仅测试手势枚举 + 幅度插值 ───────────────────
ros2 run hand_middle test_service_client --ros-args -p test_scenario:=gesture_only

# ── 终端2: 仅测试精细关节控制 ──────────────────────────
ros2 run hand_middle test_service_client --ros-args -p test_scenario:=joint_only

# ── 终端2: 仅测试异常场景 (预期全部返回失败) ────────────
ros2 run hand_middle test_service_client --ros-args -p test_scenario:=error_cases

# ── 终端2: 仅测试复位流程 ──────────────────────────────
ros2 run hand_middle test_service_client --ros-args -p test_scenario:=reset_flow
```

### 9.2 测试覆盖矩阵

| 测试场景 | `test_scenario` 参数 | 用例数 | 说明 |
|---------|---------------------|--------|------|
| 全量 | `all` (默认) | ~30 | 所有场景合并 |
| 正常流程 | `normal` | 5 | enable → gesture×2 → joint → disable |
| 手势枚举 | `gesture_only` | 11 | 5种手势 + 幅度插值 + 标签映射 |
| 关节控制 | `joint_only` | 4 | 单关节 + 全关节 |
| 异常场景 | `error_cases` | 5 | 非法指令/手势/关节/ROM/幅度 |
| 复位流程 | `reset_flow` | 4 | disable → reset → enable → gesture |

### 9.3 离线自测 (无需 ROS2)

```bash
# 纯 Python 自测，验证 orca_core stub 逻辑
cd ~/Dexterous-Hand-IoT
python3 tests/offline_self_test.py
```

---

## 10. 异常处理与错误码

### 10.1 错误码表

| 错误码 | 异常类 | 触发条件 | 日志级别 |
|--------|--------|---------|---------|
| `0` | — | 正常 | — |
| `1` | (传感器) | 任一关节温度 > `temperature_limit` | WARN |
| `2` | (传感器) | 任一关节力矩 > `torque_limit` | WARN |
| `1001` | `OrcaNotConnectedError` | 设备未连接 / 串口断开 | ERROR |
| `1002` | `OrcaNotCalibratedError` | 设备未标定 | ERROR |
| `1003` | `OrcaInvalidGestureError` | 非法手势名 / 非法 command_type | ERROR |
| `1004` | `OrcaInvalidJointError` | 非法关节名 | ERROR |
| `1005` | `OrcaJointROMError` | 关节角度超出 ROM / 幅度越界 | ERROR |
| `1006` | `OrcaTimeoutError` | 执行超时 / 状态读取超时 / 未知异常 | ERROR |
| `1007` | `OrcaSerialError` | 串口通信失败 (读写异常) | ERROR |
| `1008` | `OrcaSafetyError` | 安全策略阻止 / 紧急停止 / 电机未使能 | WARN |

### 10.2 日志分级策略

| 级别 | 使用场景 | 示例 |
|------|---------|------|
| `DEBUG` | 内部状态细节 | 传感器模拟更新值、插值计算结果、定时器发布详情 |
| `INFO` | 关键流程里程碑 | 连接成功、标定完成、指令开始/完成、节点启动/关闭 |
| `WARN` | 可恢复异常 | 非关键资源释放失败、状态读取偶发失败、故障触发 |
| `ERROR` | 需人工介入 | 串口断开、参数越界、执行失败、设备初始化失败 |
| `FATAL` | 致命错误 (节点不可用) | orca_core 完全不可用 |

### 10.3 异常处理链

```
ROS Service Request
    │
    ▼
hand_node._handle_command()
    ├── try: bridge.execute_command()
    │     ├── 参数校验 (ROM / 关节名 / 手势名 / 幅度)
    │     │     └── 越界 → 立即返回 error_code, 不调用硬件
    │     ├── 设备就绪检查 (connected / calibrated / motor_enabled)
    │     │     └── 不满足 → 立即返回 error_code
    │     └── orca_core API 调用
    │           ├── OrcaNotConnectedError  → error_code=1001
    │           ├── OrcaNotCalibratedError → error_code=1002
    │           ├── OrcaInvalidGestureError → error_code=1003
    │           ├── OrcaInvalidJointError  → error_code=1004
    │           ├── OrcaJointROMError      → error_code=1005
    │           ├── OrcaTimeoutError       → error_code=1006
    │           ├── OrcaSerialError        → error_code=1007
    │           ├── OrcaSafetyError        → error_code=1008
    │           └── Exception (未预期)      → error_code=1006
    │
    └── except Exception → error_code=1006 (兜底防御)
```

---

## 11. 扩展指南 (给后续开发者)

### 11.1 接入真实温度/力矩传感器

当硬件团队完成温度/力矩传感器驱动后，仅需修改 `hand_bridge.py` 中的一个方法：

```python
# 文件: hand_middle/hand_bridge.py
# 当前实现 (模拟数据):

def _update_sim_sensors(self) -> None:
    """更新模拟温度/力矩传感器数据。"""
    for joint in JOINT_ORDER:
        self._sim_temperatures[joint] += random.uniform(-0.3, 0.3)
        self._sim_temperatures[joint] = max(20.0, min(90.0, self._sim_temperatures[joint]))
        self._sim_torques[joint] += random.uniform(-0.02, 0.02)
        self._sim_torques[joint] = max(0.0, min(5.0, self._sim_torques[joint]))

# ── 替换为真实硬件读取 (示例) ─────────────────────────

def _update_sim_sensors(self) -> None:
    """从硬件编码器读取温度/电流，估算力矩。"""
    if self._hand is None:
        return
    for joint in JOINT_ORDER:
        # 示例: 通过 orca_core 新增的 API 读取
        # self._sim_temperatures[joint] = self._hand.get_joint_temperature(joint)
        # self._sim_torques[joint] = self._hand.get_joint_current(joint) * TORQUE_CONSTANT
        pass  # 替换为实际读取逻辑
```

> 注意: 需要硬件团队在 `orca_core` 中暴露 `get_joint_temperature()` 和 `get_joint_current()` 方法。在此之前，中间层接口已就绪，不影响上层开发。

### 11.2 添加新指令类型

在 `hand_bridge.py` 的 `execute_command()` 方法中添加新的 `elif` 分支：

```python
def execute_command(self, command_type: str, ...) -> Dict:
    if command_type == "joint_control":
        ...
    elif command_type == "gesture":
        ...
    elif command_type == "your_new_command":    # ← 新增
        return self._execute_your_new_command(...)
    ...
```

### 11.3 添加新关节

1. 在 `hand_bridge.py` 的 `JOINT_ORDER` 列表中添加新关节名
2. 修改 `HandStatus.msg` 和 `HandCommand.srv` 中的数组长度 `[10]` → `[N]`
3. 在 `hand_node.py` 中更新数组填充循环的 range
4. 在 `orca_core/orca_hand.py` 的 `VALID_JOINTS` 和 `JOINT_ROMS` 中添加关节定义

> ⚠️ 数组长度变更属于**破坏性修改**，需要同步更新消息定义、bridge、node 和所有上游消费者。

### 11.4 调整发布频率

```bash
# 运行时修改 (无需重新编译)
ros2 launch hand_middle hand_middle.launch.py status_publish_rate:=50.0
```

频率上限取决于串口波特率和硬件响应速度。当前 orca_core stub 没有实际串口延迟，最高可到 ~100Hz。

---

## 12. 常见问题 (FAQ)

### Q1: `colcon build` 报 `ModuleNotFoundError: No module named 'orca_core'`

**A**: 检查 `third_party/orca_core/` 目录是否存在，确认 `setup.py` 中的路径计算正确：

```bash
ls third_party/orca_core/
# 应该看到: __init__.py  orca_hand.py  orca_status.py  orca_exceptions.py
```

### Q2: `ros2 run hand_middle hand_node` 报 HandCommand.srv 找不到

**A**: 需要先 `colcon build` 编译消息定义。编译后检查：

```bash
# 确认消息已生成
ls install/hand_middle/lib/python3.*/site-packages/hand_middle/srv/
# 应该看到: _hand_command.py (或 HandCommand.py)

# 确认环境已刷新
source install/setup.bash
```

### Q3: 节点启动后立刻报 `Device initialization FAILED`

**A**: 这是预期行为 — 当前没有真实硬件连接。可以：

```bash
# 跳过自动初始化
ros2 launch hand_middle hand_middle.launch.py auto_initialize:=false
```

节点会在没有硬件的情况下正常运行，Service 调用会返回 `error_code=1001`（未连接）。

### Q4: 状态话题没有数据

**A**: 检查：

```bash
# 1. 确认话题存在
ros2 topic list | grep hand_middle

# 2. 确认 HandStatus.msg 已编译
ros2 interface show hand_middle/msg/HandStatus

# 3. 确认节点日志没有报错
# 节点终端应显示 "Status publisher ready on /hand_middle/status @ 10.0Hz"
```

### Q5: Windows 下串口号怎么写

**A**: 使用 Windows COM 口命名：

```bash
ros2 launch hand_middle hand_middle.launch.py port:=COM3
# 或
ros2 run hand_middle hand_node --ros-args -p port:=COM3
```

在 WSL 中访问 Windows COM 口需要额外配置，参考 [WSL USB 设备连接文档](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)。

### Q6: 如何在不重新编译的情况下修改参数

**A**: 修改 `config/params.yaml` 后在 launch 时指定：

```bash
ros2 launch hand_middle hand_middle.launch.py \
    --params-file src/hand_middle/config/params.yaml
```

`--symlink-install` 编译模式下，Python 代码修改无需重新编译，重启节点即可。

---

## 13. 小组协作约定

### 13.1 分支策略

```
main              ← 稳定版本 (仅通过完整测试后合并)
  └── develop     ← 日常开发分支
        ├── feature/xxx   ← 新功能开发
        ├── bugfix/xxx    ← Bug 修复
        └── hotfix/xxx    ← 紧急修复
```

### 13.2 代码规范

- **命名**: 遵循 ROS2 Python 规范
  - 文件名: `snake_case.py`
  - 类名: `PascalCase`
  - 函数/变量: `snake_case`
  - 常量: `UPPER_SNAKE_CASE`
  - 私有成员: `_leading_underscore`
- **注释**: 每个公开方法必须有 docstring (Google 风格)
- **类型标注**: 推荐使用 `typing` 模块 (`Dict`, `List`, `Optional` 等)
- **日志**: 使用 `self.get_logger().info()` (ROS2 节点内) 或 `logging.getLogger(__name__)` (纯 Python 模块)

### 13.3 提交前检查清单

- [ ] `colcon build --packages-select hand_middle` 编译通过
- [ ] `ros2 pkg executables hand_middle` 可执行文件已注册
- [ ] `ros2 launch hand_middle hand_middle.launch.py` 正常启动
- [ ] `ros2 run hand_middle test_service_client` 测试通过 (有硬件) 或异常路径返回正确错误码 (无硬件)
- [ ] 未修改 `third_party/orca_core/` 下的任何文件
- [ ] 新增功能有对应的 docstring 和测试用例
- [ ] 破坏性变更已在 commit message 中明确标注 `BREAKING:`

### 13.4 禁止事项

| ❌ 禁止 | ✅ 正确做法 |
|---------|------------|
| 修改 `third_party/` 下任何文件 | 在 `hand_bridge.py` 中封装和扩展 |
| 在 Service 回调中做耗时操作 (>100ms) | 使用异步 Future 或独立线程 |
| 吞掉异常不记录日志 | 分级日志 + 返回明确 error_code |
| 硬编码参数值 | 使用 ROS2 Parameter 系统 |
| 跳过参数校验直接调用硬件 | 先校验后执行，fail-fast |

### 13.5 沟通渠道

- **技术讨论**: 项目 GitHub Issues
- **API 变更**: 在 PR 中 @ 所有上游消费者
- **硬件问题**: 联系底层驱动团队 (orca_core 维护者)
- **紧急问题**: 项目群 / 实时通讯工具

---

## 14. 附录: 完整源码索引

### 14.1 `package.xml` — 功能包清单

**路径**: `ros2_ws/src/hand_middle/package.xml`

声明 ROS2 依赖: `rclpy`, `std_msgs`, `rosidl_default_generators`, `rosidl_default_runtime`。

### 14.2 `setup.py` — 构建配置

**路径**: `ros2_ws/src/hand_middle/setup.py`

- 注入 `third_party/orca_core` 到 `sys.path`
- 注册 3 个 `console_scripts` 入口点
- 安装 `.srv` / `.msg` / `.launch.py` / `.yaml` 到 share 目录

### 14.3 `HandCommand.srv` — 指令下发服务定义

**路径**: `ros2_ws/src/hand_middle/srv/HandCommand.srv`

```
string command_type             # joint_control | gesture | reset | enable | disable
string gesture_name             # 预定义手势名
float32 amplitude               # 开合幅度 0.0 ~ 1.0
string[10] joint_names          # 目标关节名数组
float32[10] joint_targets       # 目标角度数组 (°)
float32 hold_time_sec           # 保持时间 (秒)
bool return_to_neutral          # 是否回中
---
bool success                    # 执行成功?
string execution_status         # idle|received|running|completed|failed|aborted
int32 error_code                # 错误码
string error_message            # 错误说明
string timestamp                # ISO 8601 时间戳
```

### 14.4 `HandStatus.msg` — 状态发布消息定义

**路径**: `ros2_ws/src/hand_middle/msg/HandStatus.msg`

```
std_msgs/Header header          # 时间戳
bool connected                  # 串口已连接?
bool calibrated                 # 已标定?
bool motor_enabled              # 电机使能?
string[10] joint_names          # 关节名
float32[10] joint_positions     # 角度 (°)
float32[10] joint_temperatures  # 温度 (°C)
float32[10] joint_torques       # 力矩 (N·m)
int32 fault_code                # 故障码
string fault_message            # 故障描述
string current_action           # 当前动作
string execution_status         # 执行状态
```

### 14.5 `hand_bridge.py` — 中间适配层

**路径**: `ros2_ws/src/hand_middle/hand_middle/hand_bridge.py`

核心类: `HandBridge`

| 方法 | 功能 |
|------|------|
| `initialize()` | 连接 + 标定 + 使能电机 |
| `shutdown()` | 安全关闭 |
| `execute_command()` | 统一指令分发入口 |
| `get_device_status()` | 扩展状态读取 (含温度/力矩/故障) |
| `reset_hardware()` | 硬件复位 (stop → disconnect → reconnect → calibrate → enable) |
| `enable_motor()` | 电机使能 |
| `disable_motor()` | 电机去使能 + 紧急停止 |
| `emergency_stop()` | 紧急停止 |
| `_detect_fault()` | 综合故障检测 (过温/过力矩/连接异常) |
| `_execute_gesture_with_amplitude()` | 手势 + 开合幅度插值 |
| `_execute_joint_control()` | 精细关节控制 + ROM 校验 |

### 14.6 `hand_node.py` — ROS2 主控节点

**路径**: `ros2_ws/src/hand_middle/hand_middle/hand_node.py`

核心类: `HandNode` (继承 `rclpy.node.Node`)

| 成员 | 类型 | 功能 |
|------|------|------|
| `_bridge` | `HandBridge` | 中间适配层实例 |
| `_srv` | `Service` | `/hand_middle/command` 服务 |
| `_status_publisher` | `Publisher` | `/hand_middle/status` 发布者 |
| `_status_timer` | `Timer` | 10Hz 状态发布定时器 |
| `_handle_command()` | 回调 | Service 请求处理 |
| `_publish_status_callback()` | 回调 | 定时器触发 → 发布 HandStatus |

### 14.7 `test_service_client.py` — Service 测试

**路径**: `ros2_ws/src/hand_middle/hand_middle/test_service_client.py`

核心类: `TestServiceClient`

支持参数: `test_scenario` (`all` / `normal` / `gesture_only` / `joint_only` / `error_cases` / `reset_flow`)

### 14.8 `test_topic_subscriber.py` — Topic 测试

**路径**: `ros2_ws/src/hand_middle/hand_middle/test_topic_subscriber.py`

核心类: `TestTopicSubscriber`

支持参数: `stats_mode` (true/false), `fault_only` (true/false)

三种显示模式:
- **详细模式**: 每条消息打印完整的关节数据表格
- **统计模式**: 每秒汇总 Hz + 故障计数
- **故障过滤**: 仅在有故障时打印

### 14.9 `hand_middle.launch.py` — 启动文件

**路径**: `ros2_ws/src/hand_middle/launch/hand_middle.launch.py`

可配置参数: `port`, `baudrate`, `status_publish_rate`, `auto_initialize`, `temperature_limit`, `torque_limit`

### 14.10 `params.yaml` — 参数默认值

**路径**: `ros2_ws/src/hand_middle/config/params.yaml`

```yaml
hand_node:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 115200
    auto_initialize: true
    status_publish_rate: 10.0
    temperature_limit: 75.0
    torque_limit: 2.5
```

---

> **文档维护**: 本文件随 `hand_middle` 包一起版本管理。任何接口变更必须同步更新本文档。
>
> **反馈**: 发现文档错误或疏漏，请在对应 PR 中修正或联系 Hand Middleware Team。
