# 上层兼容版机械手底层

## 替换位置

将这些文件放到：

```text
D:\IoT\team_project\flexible_hand\
```

核心文件：

- `hand_interface.py`
- `runtime_config.py`
- `realtime_calibration.yaml`
- `multi_motor_control.py`
- `motor_control.py`

## 与上层兼容的调用

```python
from hand_interface import init, do_gesture, get_status, cleanup

ok = init()
do_gesture("hand_open")
status = get_status()
cleanup()
```

同时兼容原上层代码里的：

```python
from hand_interface import do_joint_command

result = do_joint_command({
    "index_mcp": 30.0,
    "index_pip": 45.0,
})
```

## init 的行为

1. 自动读取原项目中的 `config_safe.yaml`；
2. 连接其中指定的 COM 口；
3. 验证 `realtime_calibration.yaml`；
4. 标定数据完整时同步回到 `neutral_position`；
5. 启动实时 latest-wins 调度器。

不会运行旧 `OrcaHand` 的撞限位自动校准。

## 重要

需要先逐关节填写 `realtime_calibration.yaml` 的 raw_min/raw_max。

在填写完整前，`init()` 会返回 False，不会回中或执行动作。
