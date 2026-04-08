# drempower_sdk

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/Language-C++-red.svg)](https://isocpp.org/)

`drempower_sdk` 是为 **DrEmpower 一体化关节电机** 系列打造的高性能 ROS 2 驱动程序。它通过 C++ 重构了原厂 Python 协议逻辑，为开发者提供了一个低延迟、高可靠性的机器人硬件抽象层。

目前 SDK 提供两种通信底层的驱动实现，您可以根据硬件环境选择最合适的方案：
1. **串口驱动 (`drempower_node`)**：基于 `boost::asio` 实现的串口通信（适用于 USB-to-CAN 模块透传等场景）。
2. **SocketCAN 驱动 (`drempower_sc_node`)**：基于 Linux 原生 SocketCAN 接口的通信（适用于具备原生 CAN 总线接口的设备，提供更低延迟和更稳定的底层支持）。

---

## 🚀 核心特性

-   **双底层支持**：同时支持串口透传通信与 Linux 原生 SocketCAN 通信，适应不同的硬件接入需求。
-   **高性能通信**：相比 Python 版本显著降低指令延迟，C++ 实现确保控制的高实时性。
-   **全功能支持**：完整覆盖原厂协议的所有控制模式，包括阻抗控制、运动助力、前馈控制等。
-   **多关节同步**：内置“预设指令 + 广播触发”机制，确保多自由度系统动作的物理同步性。
-   **灵活初始化**：支持通过 ROS 2 参数在启动时自动配置 PID、限位、波特率等电机属性。
-   **标准集成**：反馈数据符合 `sensor_msgs/JointState` 标准，无缝对接 MoveIt2 和 Rviz2。

---

## 🛠️ 架构设计

SDK 采用分层解耦架构，方便二次开发与跨平台移植：

1.  **Hardware Layer**: 提供串口字节流处理（Boost::Asio）与原生 SocketCAN 处理（Linux Sockets）两种实现。
2.  **Driver API Layer**: 纯 C++ 实现的电机控制逻辑，提供统一的方法调用，不依赖 ROS 2 环境。
3.  **ROS 2 Node Layer**: 负责参数映射、消息订阅 (`MotorCommand`) 与状态发布 (`JointState`)，提供 `drempower_node` 和 `drempower_sc_node` 两个可执行节点。

---

## 📦 安装指南

### 依赖项
-   ROS 2 Humble (或其他 Foxy 以上版本)
-   Boost 库 (`libboost-all-dev`)
-   Serial 驱动 (Linux 下需确保串口权限: `sudo usermod -aG dialout $USER`)

### 编译步骤
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-repo/drempower_sdk.git
cd ..
colcon build --packages-select drempower_sdk
source install/setup.bash
```

---

## ⚙️ 配置参数

在 Launch 文件中，您可以配置以下核心参数。注意，串口节点和 SocketCAN 节点的参数略有不同：

| 参数名 | 适用节点 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `port` | `drempower_node` | string | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | `drempower_node` | int | `115200` | 串口波特率 |
| `can_interface` | `drempower_sc_node` | string | `can0` | SocketCAN 接口名称 |
| `motor_ids` | 通用 | int_array | `[1]` | 默认管理的电机 ID 列表 |
| `update_rate` | 通用 | double | `20.0` | 电机状态发布频率 (Hz) |
| `init_motor_settings`| 通用 | bool | `false` | **总开关**：启动时是否应用以下配置 |
| `p_gain` / `i_gain` | 通用 | double | `-1.0` | 启动时设置 PID 增益 |
| `speed_limit` | 通用 | double | `-1.0` | 启动时设置最大转速限制 (rad/s) |
| `min_angle` / `max_angle` | 通用 | double | `-M_PI` / `M_PI` | 启动时设置角度限位 (rad) |
| `set_angle_range` | 通用 | bool | `false` | 启动时是否开启角度限位 |

---

## 📏 单位标准 (Units)

本 SDK 遵循 ROS 2 标准 SI 单位制，确保与其他 ROS 节点无缝集成：

-   **角度 (Angle)**: 弧度 (`Radians`)
-   **角速度 (Velocity)**: 弧度/秒 (`rad/s`)
-   **角加速度 (Acceleration)**: 弧度/秒² (`rad/s²`)
-   **力矩 (Effort/Torque)**: 牛顿·米 (`Nm`)
-   **刚度 (Stiffness - Kp)**: `Nm/rad`
-   **阻尼 (Damping - Kd)**: `Nm/(rad/s)`

---

## 🎮 控制模式 (type 字段)

发布到 `/motor_commands` 话题的消息需遵循以下对照表：

| type | 模式名称 | values 数组定义 | mode 字段说明 |
| :--- | :--- | :--- | :--- |
| **0** | **绝对角度** | `[角度(rad), 速度, 参数]` | mode 0:轨迹, 1:梯形, 2:前馈 |
| **1** | **速度控制** | `[速度(rad/s), 参数]` | mode 0:直接, !=0:匀加速 |
| **2** | **力矩控制** | `[力矩(Nm), 参数]` | mode 0:直接, !=0:匀增量 |
| **3** | **自适应角度**| `[角度(rad), 速度, 力矩]` | 自动适配负载 |
| **4** | **相对角度** | `[增量(rad), 速度, 参数]` | 含义同 type 0 |
| **5** | **阻抗控制** | `[角, 速, Tff, Kp, Kd]` | 实现关节柔性控制 |
| **6** | **运动助力** | `[角, 速, dθ, dω, T]` | 用于人机协作辅助 |

---

## 📖 使用示例

### 1. 快速启动

**使用串口驱动：**
```bash
ros2 launch drempower_sdk drempower_launch.py
```

**使用 SocketCAN 驱动：**
如果你想单独运行 SocketCAN 节点（假设你的 CAN 接口为 `can0`）：
```bash
ros2 run drempower_sdk drempower_sc_node --ros-args -p can_interface:="can0" -p motor_ids:="[1, 2]"
```

### 2. 命令行控制 (同步控制 ID 1 和 2 到 1.57 rad [90°])
```bash
ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand \
"{motor_ids: [1, 2], type: 0, mode: 1, values: [1.57, 2.0, 10.0]}"
```

---

## 📖 详细指令示例

以下是常用控制模式的 `ros2 topic pub` 命令示例：

### 1. 位置控制 (Position Control)
- **轨迹跟踪模式** (Trajectory Tracking):
  ```bash
  ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand "{motor_ids: [1], type: 0, mode: 0, values: [1.57, 2.0, 150.0]}" # values: [角度(rad), 速度(rad/s), 滤波带宽]
  ```
- **梯形轨迹模式** (Trapezoidal / S-curve):
  ```bash
  ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand "{motor_ids: [1], type: 0, mode: 1, values: [1.57, 2.0, 10.0]}"  # values: [角度(rad), 目标速度(rad/s), 加速度(rad/s^2)]
  ```
- **前馈控制模式** (Feedforward):
  ```bash
  ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand "{motor_ids: [1], type: 0, mode: 2, values: [1.57, 2.0, 0.5]}"   # values: [角度(rad), 前馈速度(rad/s), 前馈力矩(Nm)]
  ```

### 2. 速度控制 (Speed Control)
- **速度前馈模式** (Direct/Feedforward):
  ```bash
  ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand "{motor_ids: [1], type: 1, mode: 0, values: [3.14, 0.5]}"        # values: [目标速度(rad/s), 前馈力矩(Nm)]
  ```
- **速度爬升模式** (Ramp/Climb):
  ```bash
  ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand "{motor_ids: [1], type: 1, mode: 1, values: [3.14, 1.0]}"        # values: [目标速度(rad/s), 目标加速度(rad/s^2)]
  ```

### 3. 力矩控制 (Torque Control)
- **直接控制模式** (Direct):
  ```bash
  ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand "{motor_ids: [1], type: 2, mode: 0, values: [1.0, 0.0]}"         # values: [目标力矩(Nm), 无效参数]
  ```
- **力矩爬升模式** (Ramp/Climb):
  ```bash
  ros2 topic pub --once /motor_commands drempower_sdk/msg/MotorCommand "{motor_ids: [1], type: 2, mode: 1, values: [1.0, 0.2]}"         # values: [目标力矩(Nm), 力矩增量Nm/s]
  ```

---

## 🤝 贡献与反馈

欢迎提交 Issue 或 Pull Request 来完善此 SDK。对于新功能的建议，请参考 [DrEmpower 协议手册](https://github.com/DrRobotTech/drempower-wiki)。

---

## 📄 开源协议
本项目采用 [MIT License](LICENSE) 开源。
