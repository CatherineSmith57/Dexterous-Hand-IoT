# ORCA Hand `config.yaml` 说明书

## 1. 文件位置

当前项目主要使用的配置文件在：

`D:\IoT\orca_core\orca_core\orca_core\models\v2\orcahand_right\config.yaml`

这个文件的作用是定义：

1. 机械手和电脑如何通信
2. 机械手有哪些关节
3. 每个关节由哪个电机驱动
4. 每个关节允许的运动范围
5. 标定时采用什么参数

## 2. 这个文件本质上是什么

可以把 `config.yaml` 理解成四张表合在一起：

1. 通信参数表
2. 关节和电机对应表
3. 关节活动范围表
4. 标定流程参数表

它不是程序代码，但程序运行时会直接读取它。

## 3. 顶部通信参数

示例字段：

```yaml
port: /dev/ttyACM0
baudrate: 1000000
motor_type: waveshare
max_current: 300
type: right
control_mode: position
```

字段说明：

1. `port`
   作用：串口设备名，也就是机械手接到电脑后对应的串口。
   例子：Linux 下常见是 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。

2. `baudrate`
   作用：串口通信波特率。
   例子：`1000000` 表示 1 Mbps。

3. `motor_type`
   作用：底层电机驱动类型。
   当前值：`waveshare`
   说明：程序会根据这个值选择对应的底层通信客户端。

4. `max_current`
   作用：运行时允许的最大电流上限。
   说明：过大有损坏风险，过小可能带不动。

5. `type`
   作用：区分左手还是右手。
   当前值：`right`

6. `control_mode`
   作用：默认控制模式。
   当前值：`position`
   说明：表示主要按位置控制。

## 4. 电机编号表 `motor_ids`

示例字段：

```yaml
motor_ids:
- 1
- 2
- 3
...
- 17
```

说明：

1. 这里列出整只手实际使用的电机 ID。
2. 当前配置总共有 `17` 个电机。
3. 这些 ID 是底层总线上的设备编号。

## 5. 关节名称表 `joint_ids`

示例字段：

```yaml
joint_ids:
- wrist
- thumb_cmc
- thumb_abd
- thumb_mcp
- thumb_dip
- index_abd
- index_mcp
- index_pip
...
```

说明：

1. 这里定义了程序对外使用的关节名字。
2. 以后写 Python 脚本时，控制的是这些名字，不是直接写电机号。

常见缩写含义：

1. `wrist`
   手腕
2. `cmc`
   拇指根部关节
3. `mcp`
   掌指关节
4. `pip`
   近端指间关节
5. `dip`
   远端指间关节
6. `abd`
   外展/内收

## 6. 关节和电机映射 `joint_to_motor_map`

示例字段：

```yaml
joint_to_motor_map:
  thumb_cmc: 17
  thumb_abd: 14
  thumb_mcp: 15
  thumb_dip: 16
  index_abd: 4
  index_mcp: 3
  index_pip: 2
  wrist: 1
```

说明：

1. 这是最重要的字段之一。
2. 它定义：`某个关节由哪个电机驱动`。

例子：

1. `wrist -> 1`
2. `index_mcp -> 3`
3. `index_pip -> 2`
4. `thumb_mcp -> 15`

学习时要重点掌握：

1. 你写脚本控制的是 `joint`
2. 底层真正动作的是 `motor`
3. 这个字段负责把两者关联起来

## 7. 反向关节 `reverse_joints`

示例字段：

```yaml
reverse_joints:
- ring_mcp
- pinky_abd
- pinky_pip
- wrist
```

说明：

1. 这些关节的运动方向和普通关节相反。
2. 也就是说，同样是“角度增大”，在某些关节上实际电机可能要反方向转。
3. 这个字段是为了让上层仍然能统一按“关节角度”来思考。

## 8. 单位转换 `unit_conversion`

示例字段：

```yaml
unit_conversion:
  position: degree
  position_scale: 57.2958
```

说明：

1. 这里告诉你配置文件中的关节位置主要按什么单位理解。
2. 当前值表明这里主要按 `degree` 理解。
3. `57.2958` 接近弧度和角度的换算系数。

## 9. 关节运动范围 `joint_roms`

示例字段：

```yaml
joint_roms:
  wrist:
  - -65
  - 35
  index_mcp:
  - -60
  - 100
  index_pip:
  - -15
  - 107
```

说明：

1. `ROM` = `Range of Motion`
2. 它表示每个关节允许的最小值和最大值
3. 程序在发送关节命令前，会参考这里做安全限制

例子：

1. `wrist: [-65, 35]`
   表示手腕允许在 `-65` 到 `35` 之间运动
2. `index_mcp: [-60, 100]`
   表示食指 MCP 的允许范围

用途：

1. 防止命令超出安全范围
2. 标定时把电机极限映射成关节角度
3. 写手势时判断动作是否合理

## 10. 中性位置 `neutral_position`

示例字段：

```yaml
neutral_position:
  thumb_cmc: 0
  thumb_abd: 50
  index_mcp: 2
  index_pip: 6
  wrist: -8
```

说明：

1. 这是机械手的默认“回中姿态”。
2. 初始化完成后，程序通常会把手移动到这里。
3. 以后写动作脚本时，建议从这个姿态开始。

## 11. 标定参数

示例字段：

```yaml
calibration_current: 300
calibration_step_size: 0.15
calibration_step_period: 0.0001
calibration_num_stable: 10
calibration_threshold: 0.01
```

字段说明：

1. `calibration_current`
   标定时使用的电流大小

2. `calibration_step_size`
   标定时每次推进的步长

3. `calibration_step_period`
   标定时两次推进之间的时间间隔

4. `calibration_num_stable`
   连续多少次“基本不动”才认为已经碰到机械极限

5. `calibration_threshold`
   多小的变化量算“稳定”

注意：

1. 这些参数过激进，可能损伤机械手
2. 实机使用前，必须同时参考 `SAFE_CALIBRATION_GUIDE.md`

## 12. 标定顺序 `calibration_sequence`

示例字段：

```yaml
calibration_sequence:
- step: 1
  joints:
    thumb_cmc: flex
- step: 2
  joints:
    thumb_cmc: extend
```

说明：

1. 这个字段定义自动标定时的动作顺序。
2. 每一步指定：
   1. 标哪个关节
   2. 朝哪个方向运动

常见方向：

1. `flex`
   弯曲
2. `extend`
   伸展

作用：

1. 让程序知道先找哪个极限
2. 最终得到每个电机的上下限
3. 再根据上下限算出关节和电机之间的比例关系

## 13. 学习时最应该盯住的字段

如果你的目标是：

1. 会读 `config.yaml`
2. 会写简单手势脚本

那你优先掌握这 `6` 组字段：

1. `motor_ids`
2. `joint_ids`
3. `joint_to_motor_map`
4. `joint_roms`
5. `neutral_position`
6. `calibration_sequence`

## 14. 建议的学习方法

建议自己做一张表，格式如下：

| joint name | motor id | ROM | neutral |
| --- | --- | --- | --- |
| wrist | 1 | [-65, 35] | -8 |
| index_mcp | 3 | [-60, 100] | 2 |
| index_pip | 2 | [-15, 107] | 6 |

这样你很快就能建立：

1. 关节名字
2. 电机编号
3. 安全范围
4. 常用初始姿态

## 15. 一个最小理解例子

比如：

```yaml
index_mcp:
  ROM: [-60, 100]
  neutral: 2
  motor: 3
```

你应该能直接读出：

1. `index_mcp` 是食指掌指关节
2. 它由 `3` 号电机驱动
3. 它的安全范围是 `-60` 到 `100`
4. 默认中性位置是 `2`

## 16. 后续怎么接到 Python 控制

当你已经能读懂这个配置文件后，下一步就是写类似这样的代码：

```python
from orca_core import OrcaHand

hand = OrcaHand("orca_core/models/v2/orcahand_right/config.yaml")
hand.connect()
hand.enable_torque()
hand.set_joint_positions({
    "index_mcp": 30,
    "index_pip": 40,
})
hand.disable_torque()
hand.disconnect()
```

这里你会发现：

1. 脚本里写的是 `index_mcp`
2. 代码并没有直接写电机 `3`
3. 因为程序已经通过 `config.yaml` 知道它们之间的映射关系

## 17. 当前项目里和它相关的文件

建议同时配合阅读这些文件：

1. [config.yaml](/D:/IoT/orca_core/orca_core/orca_core/models/v2/orcahand_right/config.yaml)
2. [hand_config.py](/D:/IoT/orca_core/orca_core/orca_core/hand_config.py)
3. [hardware_hand.py](/D:/IoT/orca_core/orca_core/orca_core/hardware_hand.py)
4. [SAFE_CALIBRATION_GUIDE.md](/D:/IoT/orca_core/orca_core/SAFE_CALIBRATION_GUIDE.md)

## 18. 一句话总结

`config.yaml` 决定了：

1. 这只手怎么连接
2. 有哪些关节
3. 每个关节对应哪个电机
4. 每个关节能动到哪里
5. 标定时应该怎么做

如果你把这个文件读懂了，后面写手势脚本会顺很多。
