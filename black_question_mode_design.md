# black 智力题模式边界与设计记录

本文档记录当前关于 `black` 任务赛智力题模式的讨论结论。当前阶段只确定 `rl_sar` 侧边界与设计方向，不涉及代码实现。

## 背景

根据比赛规则，任务赛中机器人需要自主识别智力题目并计算答案。答案对 4 取模后得到本轮高分归位区编号：

- `answer % 4 = 0/1/2/3`
- 对应编号的归位区为本轮高分区
- 若机器人未能识别智力题并确定高分区，则本轮没有高分归位区

`blackW` 用于障碍赛，当前模式体系已经基本完成。`black` 用于任务赛，会外接机械臂与相机。机械臂和相机代码位于：

```text
/home/windnotebook/PROJECT/RoboCon/Dog/arm
```

该项目由其他负责人维护，`rl_sar` 只读取接口信息，不直接修改 arm 项目代码。

## 已确定边界

`rl_sar` 只负责两类事情：

1. 机器人本体姿态控制。
2. 答题结果在当前 UI 界面中的展示。

`rl_sar` 不负责以下内容：

- 不识别智力题图像。
- 不解析或计算四则运算。
- 不决定搬箱顺序。
- 不决定高分区利用策略。
- 不触发机械臂。
- 不调用 arm/camera 项目的服务。
- 不直接参与上层导航任务编排。

机械臂控制保持独立，由上层导航或任务规划触发。答题逻辑由外部视觉/答题节点实现。

## 模式定位

建议在 `black` 的 FSM 中新增一个本体姿态模式，名称可以考虑：

- `RLFSMStateQuestionPose`
- `RLFSMStateQuestionStand`

UI 中可显示为：

```text
Question
```

这个模式不是完整意义上的“答题模式”，而是“稳定拍题/等待答题结果的本体姿态模式”。

其核心目标是：

- 让机器人站在适合相机识别题目的稳定姿态。
- 在该状态下不接受普通速度控制。
- 避免 RL 残留输出影响本体稳定。
- 为外部视觉节点识别智力题提供稳定条件。

## 与 blackW retry 模式的区别

`blackW` 的 retry 模式用于障碍赛失败后的人工搬运，语义是“命令锁定 + 人工搬运允许”。

`black` 的 question 模式用于任务赛自主流程，语义是“本体稳定 + 等待外部答题结果”。

两者技术上都可以复用以下思路：

- 固定姿态。
- 清零速度指令。
- 停止 RL 输出。
- 屏蔽策略切换。
- 使用 YAML 配置姿态和切换周期。

但 UI 和状态提示不应混淆。retry 更偏危险/人工干预态，可以用红色警示；question 是正常任务态，应醒目但不应表现为故障或危险。

## black 本体姿态行为

`black` 当前是 12 个腿部 DOF，没有轮子，因此 question 模式比 `blackW` 的 retry 模式简单。

进入 question 模式后，建议行为如下：

1. 停止 RL 初始化或运行标志。
2. 清空 RL 输出队列。
3. 记录进入模式时的当前关节位置。
4. 按配置周期平滑过渡到 question 默认姿态。
5. 姿态到位后持续保持该姿态。
6. 每个控制周期清零 `x/y/yaw`。
7. 屏蔽 policy switch 请求。
8. 不响应普通 `/cmd_vel` 导致的运动。

是否强制关闭 `navigation_mode` 需要后续决定：

- 若 `navigation_mode=false` 表示完全不接受导航速度，则 question 模式中强制关闭更安全。
- 若上层需要通过导航状态判断流程，可能只需要在 FSM 内部忽略速度指令，而不修改该标志。

目前倾向于：question 模式内部清零速度并忽略运动命令，是否修改 `navigation_mode` 保持可配置或后续实测决定。

## YAML 配置建议

建议新增独立配置文件：

```text
src/rl_sar/policy/black/question_pose.yaml
```

建议字段：

```yaml
black:
  prepare_cycles: 200
  question_default_dof_pos: [0.0, 0.82, -1.5, -0.0, -0.82, 1.5, 0.0, 0.82, -1.5, -0.0, -0.82, 1.5]
  kp: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
  kd: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
```

默认姿态可以先复用 `base.yaml` 中的 `default_dof_pos`。后续根据相机视野、屏幕高度、机械臂安装后的重心和干涉情况再调整。

## 模式切换建议

建议支持以下切换：

- `GetUp -> QuestionPose`
- `RL_Locomotion -> QuestionPose`
- `QuestionPose -> RL_Locomotion`
- `QuestionPose -> GetUp`
- `QuestionPose -> Passive`

是否支持 `QuestionPose -> GetDown` 可以保留，但不是核心路径。

调试阶段可以用键盘/topic 绑定一个空闲数字，例如 `2`。比赛时不应依赖人工按键，真正触发应由上层导航或任务规划通过明确的模式指令完成。

当前仓库已有 `/rl_sim/debug_key`，短期可以复用。长期更建议新增语义更清楚的 topic，例如：

```text
/rl_sim/body_mode
```

可选字符串：

```text
question
getup
rl
passive
```

这能避免上层导航依赖键盘数字语义。

## 答题结果展示边界

答题结果由外部答题节点产生，`rl_sar` 不计算、不判断，只展示。

推荐由外部节点发布一个结果 topic，例如：

```text
/question_solver/result
```

消息可以先用 `std_msgs/msg/String` 携带 JSON，便于快速联调：

```json
{
  "success": true,
  "expression": "12+8*3",
  "answer": 36,
  "mod4": 0,
  "confidence": 0.93,
  "message": "ok"
}
```

后续稳定后，可以再由视觉/导航侧定义正式 srv/msg。

UI 订阅该 topic 并展示：

- 识别状态：waiting / success / failed
- 表达式：`expression`
- 答案：`answer`
- 高分区：`mod4`
- 置信度：`confidence`
- 错误信息：`message`

不建议让 `RL_Sim` 控制节点解析答题结果再塞进 `/rl_sim/runtime_status`。这样会让控制核心知道过多任务语义。更清晰的做法是 UI 同时订阅：

- `/rl_sim/runtime_status`
- `/question_solver/result`

前者表示本体状态，后者表示答题结果。

## UI 展示建议

question 模式 UI 应醒目，但不应使用 retry 的红色警戒语义。

建议样式：

- mode 显示 `Question`
- 使用蓝绿色或黄色作为主色
- command 区显示 `X=0.00`、`YAW=0.00`
- 新增或替换一个信息块显示答题结果
- 未收到结果时显示 `Waiting`
- 识别失败时显示 `No high-score area`
- 成功时突出显示 `Area 0/1/2/3`

示例显示内容：

```text
QUESTION
12+8*3
answer=36
area=0
confidence=93%
```

## 外部模块职责建议

视觉/答题节点职责：

- 使用相机获取智力题图像。
- OCR 识别表达式。
- 计算答案。
- 发布 `answer % 4`。
- 给出置信度和失败原因。

上层导航/任务规划职责：

- 决定何时让 `black` 进入 question 姿态。
- 等待答题结果。
- 根据 `mod4` 调整搬箱和归位策略。
- 决定何时退出 question 姿态并继续运动。
- 负责机械臂 pickup/place 触发。

机械臂职责：

- 独立执行机械臂控制。
- 根据导航触发完成抓取、放置等动作。
- 不依赖 `rl_sar` 的 question 模式实现细节。

## 后续待确认问题

1. question 模式是否应强制关闭 `navigation_mode`。
2. question 模式的进入/退出是否用现有 `/rl_sim/debug_key`，还是新增 `/rl_sim/body_mode`。
3. 外部答题结果 topic 的最终名称和消息类型。
4. UI 是否直接订阅答题结果 topic，还是由某个任务状态聚合节点统一发布。
5. question 默认姿态是否需要根据真实相机视野重新标定。
6. 进入 question 模式后，是否需要向上层发布“姿态已就绪”的状态。

## 当前结论

在当前边界下，`rl_sar` 的改动应保持很小：

- `black` 增加一个可配置的固定姿态 FSM 状态。
- 当前 UI 增加 question 模式显示。
- 当前 UI 增加外部答题结果展示。

答题算法、导航决策和机械臂控制都留在各自模块中。这样职责边界清晰，后续联调时也更容易定位问题。
