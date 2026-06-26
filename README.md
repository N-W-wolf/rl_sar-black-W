# rl_sar 使用说明

Copyright (c) 2024-2025 Ziqi Fan  
SPDX-License-Identifier: Apache-2.0

本仓库用于机器人强化学习策略的仿真验证与实物部署，覆盖四足、轮足和部分人形机器人。  
`sar` 表示 `simulation and real`。

> 免责声明：使用本代码产生的风险和后果由使用者自行承担。上机或联调前请先完成限位、急停、支撑与隔离等安全措施。

## 版权与许可证

- 本仓库代码版权与许可证以 [LICENSE](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/LICENSE) 为准
- 当前 README 只描述本地部署与调试流程，不改变原项目许可证
- 若你在此基础上继续分发或修改，请保留原始版权声明与许可证文本

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- C++ 部署使用 `libtorch`

如果你当前就在本仓库内工作，下面的命令默认从仓库根目录执行：

```bash
cd ~/PROJECT/RoboCon/Dog/rl_sar
```

运行前先加载 ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

## 依赖

ROS 2 常用依赖：

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard \
                 ros-$ROS_DISTRO-ros2-control \
                 ros-$ROS_DISTRO-ros2-controllers \
                 ros-$ROS_DISTRO-control-toolbox \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher-gui \
                 ros-$ROS_DISTRO-gazebo-ros2-control \
                 ros-$ROS_DISTRO-gazebo-ros-pkgs \
                 ros-$ROS_DISTRO-xacro
```

还需要：

```bash
sudo apt install liblcm-dev libyaml-cpp-dev
```

`libtorch` 需要提前准备，并将 `Torch_DIR` 指向实际安装目录。

## 编译

本仓库支持多 ROS 版本，必须使用根目录下的脚本编译，不建议直接手写 `colcon build` 替代。

编译全部包：

```bash
source /opt/ros/humble/setup.bash
./build.sh
```

只编译 `rl_sar`：

```bash
source /opt/ros/humble/setup.bash
./build.sh rl_sar
```

清理构建产物：

```bash
./build.sh -c
```

仅做纯 CMake 硬件部署构建：

```bash
./build.sh -m
```

编译完成后加载工作空间：

```bash
source install/setup.bash
```

## 目录约定

策略部署目录：

```text
src/rl_sar/policy/<ROBOT>/<CONFIG>/
```

其中通常需要：

- `policy.pt`
- `config.yaml`
- `../base.yaml`
- 对应机器人 FSM

例如 `blackW/himloco` 和 `blackW/himloco_down`：

- [base.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/base.yaml)
- [himloco/config.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/himloco/config.yaml)
- [himloco_down/config.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/himloco_down/config.yaml)
- [fsm.hpp](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/fsm.hpp)

## 配置原则

部署时要分清两层顺序：

- `policy` 内部顺序
- 外部接口顺序

`policy` 内部顺序由训练侧决定，`rl_sar` 中的这些字段必须始终跟随 `policy` 顺序：

- `default_dof_pos`
- `wheel_indices`
- `action_scale`
- `rl_kp`
- `rl_kd`
- `observations`

真正负责对齐到仿真或实机顺序的是：

- `joint_mapping`

### blackW 当前约定

`blackW` 当前内部策略顺序为：

```text
FL hip thigh calf wheel
FR hip thigh calf wheel
RL hip thigh calf wheel
RR hip thigh calf wheel
```

`blackW` 当前配置使用：

- `num_of_dofs = 16`
- `wheel_indices = [3, 7, 11, 15]`

## 运行方式

### 推荐的交互调试方式

不要再直接依赖 `ros2 launch rl_sar rl_sim.launch.py ...` 下的终端输入。  
在 `launch` 模式下，`rl_sim` 往往拿不到当前 shell 的 `stdin`，表现为按 `0` 没反应。

当前推荐直接使用根目录脚本：

- [run_rl_sim_debug.sh](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/run_rl_sim_debug.sh)

启动方式：

```bash
cd ~/PROJECT/RoboCon/Dog/rl_sar
source /opt/ros/humble/setup.bash
./run_rl_sim_debug.sh blackW himloco
```

这个脚本会：

- 后台启动 `parameter_blackboard`
- 前台运行 `rl_sim`
- 保留当前终端的键盘交互
- 在 `Ctrl-C` 退出时自动清理后台 `parameter_blackboard`

### 如果仍想使用 launch

也可以使用：

```bash
ros2 launch rl_sar rl_sim.launch.py rname:=blackW policy_config:=himloco
```

但这时推荐通过 topic 注入调试键，而不是直接敲键盘：

```bash
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: '0'}"
```

例如：

```bash
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: '1'}"
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: 'p'}"
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: 'space'}"
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: 't'}"
```

关闭 `rl_sar` 这组 launch 子进程：

```bash
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: 'shutdown'}"
```

## 键盘控制

常用键位如下：

| 键盘 | 作用 |
|---|---|
| `0` | 从当前初始姿态切到 `GetUp` |
| `9` | 回到初始姿态 |
| `1` | 基础 locomotion |
| `P` | 电机 passive 模式 |
| `R` | 重置仿真 |
| `Enter` | 暂停/继续仿真 |
| `W/S` | 前后速度 |
| `A/D` | 左右速度 |
| `Q/E` | 偏航速度 |
| `Space` | 清零速度命令 |
| `N` | 切换导航模式 |
| `T` | `black`/`blackW` 下按 `policy_config_cycle` 切换模型 |
| `2` | `blackW` 下进入固定姿态桥模式 |
| `3` | `blackW` 下进入限高杆模式 |

## 手柄控制

常用手柄键位如下：

| 手柄 | 作用 |
|---|---|
| `A` | 从当前初始姿态切到 `GetUp` |
| `B` | 回到初始姿态 |
| `RB + DPadUp` | 基础 locomotion |
| `LB + X` | 电机 passive 模式 |
| `RB + Y` | 重置仿真 |
| `RB + X` | 暂停/继续仿真 |
| `RB + DPadRight` | `blackW` 下进入固定姿态桥模式 |
| `RB + DPadDown` | `blackW` 下进入限高杆模式 |
| `X` | 切换导航模式 |
| `Y` | `black`/`blackW` 下按 `policy_config_cycle` 切换模型 |
| `LY/LX/RX` | 前后、左右、偏航速度 |

手柄轴输入带死区过滤：`x/y/yaw` 绝对值小于 `0.2` 时会置为 `0`，用于避免摇杆中位附近的小幅抖动。键盘输入和 `/cmd_vel` 不做这个死区过滤。

## 运行时模型切换

`black`/`blackW` 支持在运行中切换 `policy/<robot_name>/<policy_config>` 下的模型。切换时会读取目标模型的 `default_dof_pos`：

- 如果目标默认姿态和当前默认姿态一致，直接重新加载目标模型。
- 如果目标默认姿态不同，状态机会先从当前关节位置平滑过渡到目标 `default_dof_pos`，再加载目标模型并进入模型控制。

不同默认姿态之间的过渡周期由 `policy/<robot_name>/policy_switch.yaml` 中的 `posture_transition_cycles` 配置。

支持以下触发方式：

```bash
# 键盘 T 或手柄 Y：按 policy_switch.yaml 中的 policy_config_cycle 切换

# 默认 cycle 在 src/rl_sar/policy/<robot_name>/policy_switch.yaml 中配置。

# debug_key：toggle
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: 't'}"

# 显式指定目标模型，要求目标模型已写入 policy_switch.yaml
ros2 topic pub --once /rl_sim/policy_config std_msgs/msg/String "{data: 'himloco_down'}"
ros2 topic pub --once /rl_sim/policy_config std_msgs/msg/String "{data: 'himloco'}"

# topic toggle
ros2 topic pub --once /rl_sim/policy_config std_msgs/msg/String "{data: 'toggle'}"
```

切换完成状态通过 `/rl_sim/policy_switch_done` 发布：

```bash
ros2 topic echo /rl_sim/policy_switch_done
```

`false` 表示正在切换或切换未完成，`true` 表示已经完成必要的姿态过渡并成功加载目标模型。

更详细的切换状态通过 `/rl_sim/policy_switch_status` 发布：

```bash
ros2 topic echo /rl_sim/policy_switch_status
```

状态格式为 `switching <policy_config>`、`done <policy_config>`、`failed <policy_config>` 或 `ready <policy_config>`。

## blackW 固定姿态车模式

`blackW` 提供固定姿态车模式，用于通过桥、限高杆等需要明确车体姿态的障碍。该类模式不运行 RL 模型，不自动退出；进入后先平滑过渡到配置姿态，再保持腿部关节位置并用轮子低速行驶。`x` 支持前进和后退，`yaw` 用于左右轮差速转向，`x = 0` 且 `yaw != 0` 时可原地差速转向，`y` 会被忽略。

进入方式：

```bash
# 桥模式：键盘 2 或手柄 RB + DPadRight
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: '2'}"

# 限高杆模式：键盘 3 或手柄 RB + DPadDown
ros2 topic pub --once /rl_sim/debug_key std_msgs/msg/String "{data: '3'}"
```

退出方式由上层或人工触发：

- `1` 或 `RB + DPadUp`：先过渡到当前选中 RL 策略的 `default_dof_pos`，再进入 RL locomotion。
- `2` 或 `RB + DPadRight`：进入桥模式。
- `3` 或 `RB + DPadDown`：进入限高杆模式。
- `0` 或 `A`：回到 `GetUp`。
- `9` 或 `B`：回到 `GetDown`。
- `P` 或 `LB + X`：进入 passive。

桥模式和限高杆模式之间互相切换时不会主动清零 `x/yaw`，姿态插值过程中轮子会继续按当前命令行驶。切回 RL、`GetUp`、`GetDown` 或 passive 时会清零速度命令。

桥模式参数位于 [bridge_drive.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/bridge_drive.yaml)，限高杆模式参数位于 [low_bar_drive.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/low_bar_drive.yaml)。主要配置包括固定姿态、进入姿态的 `prepare_cycles`、回到 RL 默认姿态的 `exit_to_rl_cycles`、腿部 `kp/kd`、轮速限制和轮速方向 `wheel_velocity_sign`。首次实测前建议先悬空确认轮速方向，再低速上地调试。

### 新增可切换模型

新增模型时，每个模型需要放在独立的 `policy/<robot_name>/<policy_config>` 目录下。目录名 `<policy_config>` 只能包含字母、数字、下划线和短横线，并且目录内必须包含 `config.yaml` 以及该配置中 `model_name` 指向的模型文件。

运行时允许切换的模型和 `T`/手柄 `Y`/`toggle` 的循环顺序由对应机器人的 `policy_switch.yaml` 统一配置，例如 [blackW/policy_switch.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/policy_switch.yaml) 或 [black/policy_switch.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/black/policy_switch.yaml)。发布端只需要发送 `toggle` 或配置文件中已有的模型名。

以下示例以 `blackW` 为例新增一个模型配置 `himloco_fast`，并使其参与 `T`/手柄 `Y` 的循环切换。`black` 的流程相同，只需把路径和 YAML 顶层 key 中的 `blackW` 替换为 `black`。

1. 创建模型目录。

```bash
cd ~/PROJECT/RoboCon/Dog/rl_sar
mkdir -p src/rl_sar/policy/blackW/himloco_fast
```

2. 复制一个姿态和接口最接近的现有配置作为模板。

```bash
cp src/rl_sar/policy/blackW/himloco/config.yaml \
   src/rl_sar/policy/blackW/himloco_fast/config.yaml
```

3. 修改新配置的 YAML 顶层 key，使其和目录名一致。

```yaml
blackW/himloco_fast:
  model_name: "policy.pt"
  num_observations: 57
  ...
```

如果从 `himloco` 复制模板，原始顶层 key 通常是 `blackW/himloco:`，必须改为 `blackW/himloco_fast:`。否则运行时读取 `blackW/himloco_fast/config.yaml` 时无法找到对应配置节点。

4. 放入模型文件，并确认文件名和 `model_name` 一致。

```bash
cp /path/to/policy.pt src/rl_sar/policy/blackW/himloco_fast/policy.pt
```

如果配置中写的是：

```yaml
model_name: "policy_abc.pt"
```

则目录内需要存在：

```bash
src/rl_sar/policy/blackW/himloco_fast/policy_abc.pt
```

5. 按需要设置 `default_dof_pos`。

- 如果 `himloco_fast` 和当前模型的默认姿态相同，保持两者 `default_dof_pos` 一致。切换时会直接重新加载模型，不执行姿态过渡。
- 如果 `himloco_fast` 使用不同默认姿态，将 `default_dof_pos` 写为目标模型真实初始姿态。切换时会先平滑过渡到该姿态，再加载模型。

6. 将新模型加入切换配置。

编辑 `src/rl_sar/policy/blackW/policy_switch.yaml`，将 `himloco_fast` 加入 `policy_config_cycle`。该列表既是允许切换的模型集合，也是按键循环顺序：

```yaml
blackW:
  policy_config_cycle:
    - himloco
    - himloco_fast
    - himloco_down
```

此时按键循环顺序为：

```text
himloco -> himloco_fast -> himloco_down -> himloco
```

直接通过 topic 显式切换时，目标模型也必须已经写入 `policy_switch.yaml`：

```bash
ros2 topic pub --once /rl_sim/policy_config std_msgs/msg/String "{data: 'himloco_fast'}"
```

切换前建议先监听状态话题：

```bash
ros2 topic echo /rl_sim/policy_switch_status
ros2 topic echo /rl_sim/policy_switch_done
```

## 与 black_mujoco 联调

当前 `blackW` 的 sim2sim 联调推荐分两个终端。

### 终端 1：启动 MuJoCo

```bash
cd ~/PROJECT/RoboCon/Dog/black_mujoco
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujoco_runner mujoco.launch.py rname:=blackW
```

### 终端 2：启动 rl_sar

```bash
cd ~/PROJECT/RoboCon/Dog/rl_sar
source /opt/ros/humble/setup.bash
./run_rl_sim_debug.sh blackW himloco
```

注意：

- `run_rl_sim_debug.sh` 只会清理它自己拉起的 `rl_sar + parameter_blackboard`
- 另一个终端里的 `black_mujoco` 仍需单独退出

## blackW 额外说明

### 1. 观测维度和 black 不同

`black` 单帧观测是 `45` 维。  
`blackW` 单帧观测是 `57` 维。

`blackW` 的单帧结构是：

```text
commands(3)
+ base_ang_vel(3)
+ gravity(3)
+ dof_pos_err(16)
+ dof_vel(16)
+ actions(16)
= 57
```

其中轮子位置误差槽位会被 mask 为 0，但维度仍保留在观测里。

### 2. 旧版 JIT 导出器不适配 blackW

如果 `blackW` 在模型接管时报错：

```text
mat1 and mat2 shapes cannot be multiplied (1x64 and 76x512)
```

这通常不是 `rl_sar` 配置错，而是导出的 `policy.pt` 还沿用了 12DoF 机器人的旧逻辑，把 actor 前缀输入硬编码成了 `45` 维。

当前已经在训练侧修正导出逻辑，重新导出 `policy.pt` 后再替换部署文件即可。

## 常见问题

### 1. `ros2 launch` 下按键没反应

这是 `stdin` 不在 `rl_sim` 进程上的典型表现。  
优先使用：

```bash
./run_rl_sim_debug.sh blackW himloco
```

### 2. `install/setup.bash: COLCON_TRACE: 未绑定的变量`

这是脚本里 `set -u` 与 `colcon setup` 脚本的兼容性问题。  
当前 `run_rl_sim_debug.sh` 已处理，无需手改。

### 3. `policy.pt` 复制过去了，但进入模型接管仍报维度错误

优先检查：

- 这份 `policy.pt` 是否是修复后的 `blackW` JIT
- 是否确实覆盖到了 `src/rl_sar/policy/blackW/himloco/policy.pt`
- `config.yaml` 是否仍是 `blackW/himloco`

### 4. 退出时有残留进程

当前推荐脚本已经处理了 `parameter_blackboard` 的清理。  
如果你是分终端启动 sim2sim，`black_mujoco` 仍需手动退出，这是正常行为。

## 关键文件

- [build.sh](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/build.sh)
- [run_rl_sim_debug.sh](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/run_rl_sim_debug.sh)
- [rl_sim.cpp](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/src/rl_sim.cpp)
- [rl_sdk.cpp](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp)
- [rl_sim.launch.py](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/launch/rl_sim.launch.py)
- [blackW base.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/base.yaml)
- [blackW himloco config.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/himloco/config.yaml)
- [blackW himloco_down config.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/himloco_down/config.yaml)
