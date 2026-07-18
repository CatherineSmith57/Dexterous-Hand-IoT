# ORCA 项目接口协议

## 1. 文档目的

本协议用于统一当前比赛项目中各模块之间的输入输出格式。

当前项目目标为：

1. 识别图片中的手势
2. 将识别结果转换为灵巧手动作命令
3. 调用 `orca_core` 控制 ORCA Hand 执行动作
4. 回传执行状态给上层或展示端

本协议适用于以下模块之间的对接：

1. 算法识别模块
2. 上层控制模块
3. ROS 2 集成模块
4. `orca_core` 执行模块
5. 展示与平台模块

---

## 2. 总体链路

当前系统链路定义为：

1. `image_input`
2. `gesture_recognition`
3. `gesture_mapping`
4. `hand_command`
5. `orca_core_execution`
6. `execution_status`

对应中文含义：

1. 输入图片
2. 识别手势类别
3. 将手势类别映射为灵巧手动作
4. 生成机械手命令
5. 调用 `orca_core` 执行动作
6. 返回执行结果

---

## 3. 模块边界

### 3.1 算法识别模块

职责：

1. 接收图片或视频帧
2. 输出手势识别结果
3. 不直接控制机械手

### 3.2 上层控制模块

职责：

1. 接收算法输出
2. 将识别结果映射为手部动作
3. 调用 `orca_core` 或通过 ROS 2 发送命令
4. 管理动作流程和简单状态机

### 3.3 `orca_core` 执行模块

职责：

1. 接收手势命令或关节目标
2. 控制灵巧手真实动作
3. 回传执行状态

### 3.4 展示模块

职责：

1. 展示输入目标
2. 展示识别结果
3. 展示手部执行状态
4. 不直接访问底层串口或电机寄存器

---

## 4. 数据命名约定

统一命名规则如下：

1. 全部字段使用 `snake_case`
2. 动作名、手势名统一使用英文小写
3. 状态值统一使用固定枚举
4. 时间统一使用 ISO 8601 字符串或 Unix 时间戳

示例：

1. `gesture_label`
2. `gesture_name`
3. `joint_targets`
4. `execution_status`
5. `error_message`

不允许：

1. 中文字段名
2. 拼音字段名
3. 混用 `camelCase`
4. 临时缩写不统一

---

## 5. 算法模块输出协议

算法模块输出统一命名为：`gesture_recognition_result`

最小字段定义如下：

```json
{
  "request_id": "req_0001",
  "source_type": "image",
  "image_id": "img_0001",
  "gesture_label": "fist",
  "confidence": 0.92,
  "timestamp": "2026-07-10T20:00:00+08:00"
}
```

字段说明：

1. `request_id`  
   本次请求唯一标识
2. `source_type`  
   数据源类型，当前取值建议为 `image` 或 `camera`
3. `image_id`  
   当前图片编号
4. `gesture_label`  
   识别出的手势类别
5. `confidence`  
   识别置信度，范围 `0.0 ~ 1.0`
6. `timestamp`  
   识别完成时间

当前建议的最小手势集合：

1. `open_palm`
2. `fist`
3. `pinch`

后续可扩展：

1. `two_finger`
2. `point`
3. `ok_sign`

---

## 6. 手势映射协议

上层控制模块负责把 `gesture_label` 转成 `gesture_name` 或 `joint_targets`。

推荐优先使用 `gesture_name`，因为更利于比赛阶段快速联调。

### 6.1 手势名命令格式

```json
{
  "request_id": "req_0001",
  "gesture_label": "fist",
  "gesture_name": "hand_close",
  "hold_time_sec": 2.0,
  "return_to_neutral": true
}
```

字段说明：

1. `request_id`  
   透传算法侧请求 ID
2. `gesture_label`  
   原始识别结果
3. `gesture_name`  
   映射后的灵巧手动作名
4. `hold_time_sec`  
   动作保持时间，单位秒
5. `return_to_neutral`  
   动作完成后是否回中

### 6.2 推荐映射表

当前建议最小映射如下：

1. `open_palm -> hand_open`
2. `fist -> hand_close`
3. `pinch -> pinch_grasp`

如需扩展，可新增：

1. `two_finger -> two_finger_pose`
2. `point -> point_pose`

---

## 7. `orca_core` 输入协议

`orca_core` 执行层接收两类命令：

1. `gesture_command`
2. `joint_command`

### 7.1 `gesture_command`

适合比赛早期与 demo 阶段。

```json
{
  "command_type": "gesture_command",
  "request_id": "req_0001",
  "gesture_name": "hand_close",
  "hold_time_sec": 2.0,
  "return_to_neutral": true
}
```

字段说明：

1. `command_type`  
   固定值：`gesture_command`
2. `request_id`  
   请求唯一标识
3. `gesture_name`  
   预定义动作名
4. `hold_time_sec`  
   动作保持时间
5. `return_to_neutral`  
   动作后是否回中

### 7.2 `joint_command`

适合后续精细化控制。

```json
{
  "command_type": "joint_command",
  "request_id": "req_0002",
  "joint_targets": {
    "thumb_mcp": 20,
    "index_mcp": 35,
    "index_pip": 50
  },
  "num_steps": 25,
  "step_size": 0.01,
  "hold_time_sec": 1.5,
  "return_to_neutral": true
}
```

字段说明：

1. `command_type`  
   固定值：`joint_command`
2. `request_id`  
   请求唯一标识
3. `joint_targets`  
   目标关节角度字典
4. `num_steps`  
   插值步数
5. `step_size`  
   相邻步之间时间间隔
6. `hold_time_sec`  
   动作保持时间
7. `return_to_neutral`  
   动作后是否回中

约束要求：

1. `joint_targets` 中的键必须来自 `config.yaml` 的 `joint_ids`
2. 角度范围必须遵守 `joint_roms`
3. 未标定状态下不得执行大幅度动作

---

## 8. `orca_core` 输出协议

`orca_core` 执行后统一返回 `execution_result`。

```json
{
  "request_id": "req_0001",
  "success": true,
  "execution_status": "completed",
  "current_action": "hand_close",
  "connected": true,
  "calibrated": true,
  "error_code": 0,
  "error_message": "",
  "timestamp": "2026-07-10T20:00:03+08:00"
}
```

字段说明：

1. `request_id`  
   请求唯一标识
2. `success`  
   是否执行成功
3. `execution_status`  
   当前执行状态
4. `current_action`  
   当前动作名
5. `connected`  
   当前是否连接设备
6. `calibrated`  
   当前是否已标定
7. `error_code`  
   错误码
8. `error_message`  
   错误说明
9. `timestamp`  
   返回时间

---

## 9. 状态枚举约定

### 9.1 `execution_status`

允许取值：

1. `received`
2. `running`
3. `completed`
4. `failed`
5. `aborted`

说明：

1. `received`  
   已接收命令，尚未开始执行
2. `running`  
   正在执行动作
3. `completed`  
   已完成动作
4. `failed`  
   执行失败
5. `aborted`  
   人工中断或安全停止

### 9.2 `source_type`

允许取值：

1. `image`
2. `camera`
3. `manual`

---

## 10. 错误码约定

建议统一如下错误码：

1. `0`  
   成功
2. `1001`  
   未连接设备
3. `1002`  
   未完成标定
4. `1003`  
   非法手势名
5. `1004`  
   非法关节名
6. `1005`  
   关节目标超出 ROM
7. `1006`  
   执行动作超时
8. `1007`  
   串口通信失败
9. `1008`  
   安全策略阻止执行

错误返回示例：

```json
{
  "request_id": "req_0003",
  "success": false,
  "execution_status": "failed",
  "connected": false,
  "calibrated": false,
  "error_code": 1001,
  "error_message": "hand is not connected"
}
```

---

## 11. ROS 2 对接建议

如果后续使用 ROS 2，建议按下列方式对接：

### 11.1 推荐节点划分

1. `gesture_recognition_node`
2. `gesture_mapping_node`
3. `hand_control_node`
4. `status_publish_node`

### 11.2 推荐 topic / service

推荐 topic：

1. `/gesture_recognition/result`
2. `/hand_control/status`

推荐 service：

1. `/hand_control/execute_gesture`
2. `/hand_control/execute_joint_targets`

### 11.3 消息职责

1. 算法节点发布识别结果
2. 控制节点负责映射与下发
3. `orca_core` 封装在 hand control 节点内
4. 状态统一从 hand control 节点对外发布

---

## 12. 展示层接口协议

展示层只读取统一状态，不直接访问底层驱动。

展示层推荐读取字段：

1. `image_id`
2. `gesture_label`
3. `gesture_name`
4. `execution_status`
5. `success`
6. `current_action`
7. `error_message`

展示页最小展示建议：

1. 当前输入图片
2. 当前识别手势
3. 当前映射动作
4. 当前执行状态
5. 最近一次执行结果

---

## 13. 最小验收闭环

当前项目最小验收流程定义如下：

1. 输入 `open_palm` 图片
2. 算法输出 `gesture_label = open_palm`
3. 上层映射为 `gesture_name = hand_open`
4. `orca_core` 执行动作
5. 返回 `execution_status = completed`

同理应支持：

1. `fist -> hand_close`
2. `pinch -> pinch_grasp`

---

## 14. 版本约定

本接口协议当前版本：

1. `version: 0.1.0`

版本升级原则：

1. 新增字段尽量保持向后兼容
2. 删除字段必须升级次版本或主版本
3. 枚举值变更必须更新文档

---

## 15. 总结

本协议核心目标只有一个：

1. 让算法模块、控制模块、`orca_core` 和展示模块能稳定对接

当前统一约定如下：

1. 算法输出 `gesture_label`
2. 上层映射成 `gesture_name`
3. `orca_core` 接收 `gesture_command` 或 `joint_command`
4. 统一返回 `execution_result`
5. 展示层只消费统一状态，不碰底层驱动

