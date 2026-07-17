# ORCA Core 项目说明书

## 1. 项目定位

`orca_core` 是 ORCA Hand 灵巧手的底层控制项目。

它的核心职责不是做视觉、不是做 ROS 2 全套应用、也不是直接完成工业任务，而是负责：

1. 读取灵巧手模型配置
2. 连接真实电机总线
3. 管理扭矩、电流、控制模式
4. 完成标定与张力调整
5. 将“关节动作命令”转换为“电机位置命令”
6. 执行基础手势、测试动作和回中动作

一句话概括：

`orca_core = 灵巧手执行层 / hand execution layer`

---

## 2. 这个项目已经实现了什么功能

### 2.1 配置加载

项目可以读取 `config.yaml`，建立整只手的控制模型，包括：

1. 有哪些关节 `joint_ids`
2. 有哪些电机 `motor_ids`
3. 关节和电机如何对应 `joint_to_motor_map`
4. 每个关节允许的运动范围 `joint_roms`
5. 中立位姿 `neutral_position`
6. 标定顺序 `calibration_sequence`

相关文件：

1. [config.yaml](/D:/IoT/orca_core/orca_core/orca_core/models/v2/orcahand_right/config.yaml)
2. [hand_config.py](/D:/IoT/orca_core/orca_core/orca_core/hand_config.py)

### 2.2 硬件连接与总线通信

项目可以通过串口连接灵巧手电机总线，支持：

1. 打开/关闭串口
2. 自动尝试识别端口
3. 设置波特率
4. 统一管理多电机读写

当前代码支持的底层电机后端：

1. `Dynamixel`
2. `Feetech`
3. `WaveShare`

相关文件：

1. [hardware_hand.py](/D:/IoT/orca_core/orca_core/orca_core/hardware_hand.py)
2. [motor_client.py](/D:/IoT/orca_core/orca_core/orca_core/hardware/motor_client.py)
3. [waveshare_client.py](/D:/IoT/orca_core/orca_core/orca_core/hardware/waveshare_client.py)

### 2.3 关节级控制

项目对外提供的是“关节控制”，而不是只给你电机原始角度。

这意味着你可以直接写：

```python
hand.set_joint_positions({
    "index_mcp": 30,
    "thumb_mcp": 20,
})
```

然后项目会自动完成：

1. 对目标关节做合法性检查
2. 限制到 ROM 范围内
3. 根据标定结果换算为电机角度
4. 下发到对应的多颗电机

相关文件：

1. [base_hand.py](/D:/IoT/orca_core/orca_core/orca_core/base_hand.py)
2. [hardware_hand.py](/D:/IoT/orca_core/orca_core/orca_core/hardware_hand.py)

### 2.4 标定 calibration

项目已经实现自动标定流程，用于：

1. 找到每个关节的机械极限
2. 建立关节空间和电机空间之间的比例关系
3. 生成并更新 `calibration.yaml`

标定后，系统才知道：

1. 电机当前角度对应哪个关节角度
2. 关节最小值和最大值对应的电机边界

相关文件：

1. [safe_calibrate.py](/D:/IoT/orca_core/orca_core/scripts/safe_calibrate.py)
2. [SAFE_CALIBRATION_GUIDE.md](/D:/IoT/orca_core/orca_core/SAFE_CALIBRATION_GUIDE.md)
3. [calibration.py](/D:/IoT/orca_core/orca_core/orca_core/calibration.py)

### 2.5 张力调整 tension

由于 ORCA Hand 是肌腱驱动结构，项目提供张力调整流程，用于：

1. 给每根肌腱建立初始拉力
2. 消除线缆松弛
3. 为后续标定和抓取动作建立稳定基础

### 2.6 测试动作与示教脚本

项目已经提供多个脚本，可用于：

1. 回中 `neutral.py`
2. 基础测试 `simple_test.py`
3. 主流程测试 `setup.py`
4. 握住/松开 `grip_release.py`
5. 轻握 `slight_grip.py`
6. 回放动作 `replay_angles.py`
7. 记录动作 `record_angles.py`

相关目录：

1. [scripts](/D:/IoT/orca_core/orca_core/scripts)

---

## 3. 它实现了“手能怎么动”

这里不讨论具体工业场景，只讨论 `orca_core` 让灵巧手本体具备哪些运动能力。

### 3.1 单关节运动

可以单独控制某个关节运动，例如：

1. 食指 MCP 弯曲
2. 食指 PIP 弯曲
3. 拇指 MCP 弯曲
4. 手腕 flex / extend

### 3.2 多关节协同运动

可以同时控制多个关节，组成一个手势或抓取姿态，例如：

1. 多指同时弯曲
2. 拇指对掌
3. 四指张开/并拢
4. 手腕配合指尖动作

### 3.3 插值平滑运动

不是只能“瞬移”到目标位姿，还可以分步插值过去，实现更平滑的动作：

1. 从当前姿态平滑过渡到目标姿态
2. 降低突变动作带来的冲击
3. 便于做 demo 手势和抓取前接近动作

### 3.4 中立位姿控制

可以随时回到预设的 `neutral_position`，用于：

1. 上电后复位
2. 标定后回中
3. 抓取失败后的安全恢复

### 3.5 基础抓取动作

通过组合关节动作，已经具备构造以下基础动作的能力：

1. 张开 hand open
2. 握拳 hand close
3. 两指 pinch
4. 三指抓取 tripod grasp
5. 包络式抓取 power grasp

说明：

这些动作不是项目内置“高级语义技能”，而是建立在关节控制能力之上的“可组合手势能力”。

### 3.6 状态读取

项目不只会发命令，也能读状态，包括：

1. 电机位置
2. 电机电流
3. 电机温度
4. 当前关节位置

这使它具备基本闭环控制基础，但它目前还不是完整的高层智能抓取闭环系统。

---

## 4. 它没有实现什么

为了避免范围混乱，这里明确边界。

`orca_core` 当前没有直接实现：

1. 视觉识别
2. 相机标定
3. 目标检测
4. 抓取点规划
5. ROS 2 完整任务编排
6. 工业产线节拍逻辑
7. 云端 IoT 平台业务逻辑
8. 完整人机交互界面

所以它适合当作：

`可运行的灵巧手底层 baseline`

不适合直接当作：

`完整比赛成品`

---

## 5. 项目上下游技术位置

建议把整套系统分成三层理解。

### 5.1 下游：硬件执行层

`orca_core` 的下游是它直接控制的对象：

1. 总线舵机 / 电机
2. USB 转串口设备
3. 灵巧手本体机构
4. 肌腱传动结构
5. 电源与驱动链路

下游关心的是：

1. 能不能连上
2. 电机会不会动
3. 标定安不安全
4. 电流和温度是否正常

### 5.2 中游：`orca_core`

`orca_core` 在系统里承担“动作翻译器”的角色：

1. 接收上层发来的目标关节姿态
2. 结合配置和标定信息做换算
3. 向多颗电机下发控制命令
4. 回读硬件状态

### 5.3 上游：应用与任务层

`orca_core` 的上游应该是你们后续比赛要做的内容，例如：

1. ROS 2 节点
2. 视觉感知模块
3. 目标识别与分类
4. 抓取策略规划
5. 工业任务状态机
6. 上位机界面
7. IoT 数据展示系统

---

## 6. 上下游技术约定

这一节很重要，建议你们后续开发都遵守。

### 6.1 `orca_core` 对上游的输入约定

上游给 `orca_core` 的命令，建议统一成“关节空间命令”，不要直接发电机命令。

推荐输入形式：

1. `dict[str, float]`
2. 键名使用 `config.yaml` 中定义的关节名
3. 数值单位与配置保持一致

例子：

```python
{
    "thumb_mcp": 20,
    "index_mcp": 35,
    "index_pip": 50
}
```

约定：

1. 上游不直接操作 `motor_id`
2. 上游不直接写串口协议
3. 上游只描述“我想让手变成什么姿态”

### 6.2 `orca_core` 对上游的输出约定

`orca_core` 返回给上游的，建议统一为：

1. 连接状态
2. 标定状态
3. 当前关节位置
4. 当前电机电流
5. 当前电机温度
6. 错误信息

这意味着上游模块应该消费“状态信息”，而不是去读底层寄存器。

### 6.3 `orca_core` 与 ROS 2 的约定

如果后续接 ROS 2，建议保持如下边界：

1. `ROS 2` 负责消息通信、节点管理、任务编排
2. `orca_core` 负责手部硬件控制

推荐模式：

1. ROS 2 节点订阅上层抓取指令
2. ROS 2 节点调用 `orca_core` Python API
3. ROS 2 节点发布手部状态

不推荐模式：

1. 在 ROS 2 节点里重写电机换算逻辑
2. 在 ROS 2 节点里直接操作串口协议

### 6.4 `orca_core` 与视觉模块的约定

视觉模块不应该直接控制电机。

视觉模块的职责应是：

1. 给出目标类别
2. 给出目标位置或抓取建议
3. 给出抓取模式选择

然后由任务层决定调用哪一个 hand pose / grasp policy，再交给 `orca_core` 执行。

### 6.5 `orca_core` 与 IoT 模块的约定

IoT 模块建议只做状态展示与日志上报，不直接下发危险控制。

推荐上报内容：

1. 当前任务状态
2. 当前动作名称
3. 连接状态
4. 标定状态
5. 电流/温度告警

如果一定要允许远程下发命令，建议只允许：

1. 回中
2. 打开/关闭 demo
3. 启停任务

不建议直接开放：

1. 任意电机角度
2. 任意电流值
3. 任意标定操作

---

## 7. 复现阶段建议遵守的工程边界

老师说“先复现，再微调”，这里给出具体约定。

### 7.1 复现阶段不要先改的内容

1. 不先改底层串口协议
2. 不先改关节-电机换算逻辑
3. 不先改大量配置参数
4. 不先改标定主流程

### 7.2 复现阶段优先确认的内容

1. 项目能否正常加载 `config.yaml`
2. 是否能连接到真实硬件
3. 是否能读取电机状态
4. 是否能完成安全标定
5. 是否能回中
6. 是否能执行一个简单手势

### 7.3 微调阶段优先改的内容

1. `config.yaml` 中的安全参数
2. 手势脚本
3. 上层接口封装
4. 与 ROS 2 或视觉模块的连接

---

## 8. 你们比赛中可以如何使用它

建议使用方式：

1. 把 `orca_core` 当作灵巧手控制底座
2. 在它上面接 ROS 2 节点
3. 再接视觉识别和工业任务逻辑
4. 最终形成“识别 -> 决策 -> 抓取 -> 反馈”的比赛系统

推荐分工：

1. 一人负责 `orca_core` 复现与硬件联调
2. 一人负责 ROS 2 与任务流程
3. 一人负责视觉 / IoT / 演示系统

---

## 9. 当前项目的关键风险

在使用 `orca_core` 时，最需要注意的不是“代码能不能跑”，而是“实机会不会损坏”。

关键风险包括：

1. 标定电流过大
2. 标定步长过大
3. 肌腱张力过紧
4. 端口配置错误
5. 未标定就直接执行大动作

建议始终优先阅读：

1. [SAFE_CALIBRATION_GUIDE.md](/D:/IoT/orca_core/orca_core/SAFE_CALIBRATION_GUIDE.md)

---

## 10. 总结

`orca_core` 已经完成了灵巧手控制中最底层、最关键的一部分：

1. 配置建模
2. 串口连接
3. 电机控制
4. 标定与张力调整
5. 关节动作执行
6. 基础手势与测试脚本

它适合作为你们比赛项目的：

`基础执行控制平台`

你们后续真正需要补上的，是它上层的：

1. ROS 2 封装
2. 视觉感知
3. 任务编排
4. 工业场景逻辑
5. IoT 展示与评审可视化

---

## 11. 推荐阅读顺序

1. [config_yaml_guide.md](/D:/IoT/team_project/docs/config_yaml_guide.md)
2. [config.yaml](/D:/IoT/orca_core/orca_core/orca_core/models/v2/orcahand_right/config.yaml)
3. [base_hand.py](/D:/IoT/orca_core/orca_core/orca_core/base_hand.py)
4. [hardware_hand.py](/D:/IoT/orca_core/orca_core/orca_core/hardware_hand.py)
5. [safe_calibrate.py](/D:/IoT/orca_core/orca_core/scripts/safe_calibrate.py)
6. [learning_path_team_roles_first_two_weeks.md](/D:/IoT/team_project/docs/learning_path_team_roles_first_two_weeks.md)
