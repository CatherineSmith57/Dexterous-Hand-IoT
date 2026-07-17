# 机械手上层接口说明（17 电机实时版）

底层结构：

```text
视觉识别
    ↓ 17 个关节角 / 归一化弯曲度
HandRuntime：限位、限速、低置信度丢帧、latest-wins
    ↓
MultiMotorControl：GroupSyncWrite
    ↓ 一个 COM5 串口包携带多个电机目标
1~17 号电机
```

## 1. 初始化

```python
from hand_interface import init, cleanup

status = init()
print(status)
```

`init()` 只连接和启动调度器，不会自动移动，也不会运行旧 ORCA 的撞限位校准。

## 2. 标准手势

先在 `hand_config.py` 的 `GESTURES_RAW` 中填写实测位置：

```python
from hand_interface import do_gesture

do_gesture("hand_open")
do_gesture("hand_close")
do_gesture(
    "pinch_grasp",
    hold_time_sec=3.0,
    return_to_neutral=True,
)
```

## 3. 视觉实时模仿：角度输入

先为 17 个关节填写：

- `raw_min`
- `raw_max`
- `neutral_raw`
- `angle_min_deg`
- `angle_max_deg`
- `inverted`

然后视觉模块每帧调用：

```python
from hand_interface import update_from_vision

sequence = update_from_vision(
    {
        "wrist": 5.0,
        "index_pip": 52.0,
        "index_mcp": 47.0,
        # ...共 17 个关节
    },
    confidence=0.92,
    min_confidence=0.6,
    require_all=True,
)
```

该函数立即返回。视觉产生新帧过快时，旧帧会被最新帧覆盖，避免机械手追赶过时动作。

## 4. 直接下发 17 个电机 raw 位置

```python
from hand_interface import set_motor_positions

set_motor_positions(
    {
        1: 2048,
        2: 1900,
        # ...
        17: 2200,
    },
    realtime=True,
    require_all=True,
)
```

高层仍会检查 `hand_config.py` 中的每个关节软限位。

## 5. 状态与急停

```python
from hand_interface import (
    emergency_stop,
    get_status,
)

status = get_status(refresh_hardware=True)
print(status["motor_positions"])
print(status["motor_currents_raw"])

emergency_stop()
```

`motor_currents_raw` 是寄存器原始值，不冒充 mA。

## 6. 关闭

```python
cleanup()
```

`cleanup()` 会停止实时线程、关闭 1~17 扭矩并释放 COM5。
