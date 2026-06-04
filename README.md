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

例如 `blackW/himloco`：

- [base.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/base.yaml)
- [config.yaml](/home/windnotebook/PROJECT/RoboCon/Dog/rl_sar/src/rl_sar/policy/blackW/himloco/config.yaml)
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
