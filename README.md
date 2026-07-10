# Team Project

## 1. 仓库定位

这个仓库是你们团队自己的比赛总仓库。

它和上游 `orca_core` 的关系是：

1. `orca_core` 负责灵巧手底层执行控制
2. 这个仓库负责你们自己的识别、ROS 2、接口、演示和比赛交付

也就是说：

1. `上游仓库` 不再承担你们整个项目的主仓库角色
2. `这个仓库` 才是你们团队真正协作和提交的主仓库

---

## 2. 当前目标

当前最小目标定义为：

1. 输入一张图片
2. 识别图片中的手势
3. 将识别结果映射为灵巧手动作
4. 调用 `orca_core` 控制灵巧手执行
5. 返回执行状态并用于展示

---

## 3. 目录结构

```text
team_project/
├─ third_party/
│  └─ orca_core/
├─ gesture_recognition/
├─ ros2_ws/
│  └─ src/
├─ interfaces/
├─ docs/
├─ demos/
└─ scripts/
```

各目录职责：

1. `third_party/orca_core`  
   放上游 `orca_core`，只在确实需要时修改
2. `gesture_recognition`  
   放手势识别代码
3. `ros2_ws/src`  
   放 ROS 2 包
4. `interfaces`  
   放接口协议、消息格式、桥接层定义
5. `docs`  
   放你们自己的设计文档、说明书、里程碑
6. `demos`  
   放 demo 脚本、演示样例、输入输出示例
7. `scripts`  
   放辅助脚本

---

## 4. 推荐工作边界

### 4.1 在这个仓库里做什么

1. 你们自己的识别代码
2. 你们自己的 ROS 2 节点
3. 你们自己的动作映射逻辑
4. 你们自己的接口协议
5. 你们自己的展示和 demo

### 4.2 不要在上游仓库里乱堆什么

1. 不要把比赛文档全放回 `orca_core`
2. 不要把识别代码直接塞进上游源码
3. 不要把 ROS 2 总工程直接混进上游目录

---

## 5. 当前本地上游位置

你当前已经有一份本地 `orca_core`：

1. `D:\IoT\orca_core\orca_core`

这份可以先继续作为参考和调试用的上游工作副本。

后续如果你们要把它正式纳入团队仓库，有两种方式：

1. 直接复制一份到 `third_party/orca_core`
2. 后续改成 `git submodule` 或 `git subtree`

当前阶段建议：

1. 先按目录边界开发
2. 等你们主链路跑通后，再决定是否做正式上游纳管

---

## 6. 当前开发顺序

建议顺序：

1. 在 `gesture_recognition` 跑通最小手势识别
2. 在 `interfaces` 固定识别结果和手部命令格式
3. 在 `ros2_ws/src` 或 Python 脚本里写桥接层
4. 用 `orca_core` 执行动作
5. 在 `demos` 放可重复演示样例

---

## 7. 当前建议

当前不要急着做的事：

1. 不要先下 ROS 源码
2. 不要先重写 `orca_core`
3. 不要先堆太多类别的手势

当前最应该做的事：

1. 固定目录边界
2. 固定接口格式
3. 跑通最小闭环

---

## 8. 当前分工摘要

当前建议的三人分工如下：

1. `Role A: Hand Hardware and orca_core`
   负责灵巧手硬件、`orca_core`、标定、安全和基础动作
2. `Role B: ROS 2 and System Integration`
   负责 ROS 2 节点、桥接层、消息流和系统联调
3. `Role C: Gesture Recognition and Demo Application`
   负责图片手势识别、动作映射、展示和比赛 demo

详细分工请看：

1. [docs/三人分层分工表_本周任务_接口约定.md](/D:/IoT/team_project/docs/三人分层分工表_本周任务_接口约定.md)
