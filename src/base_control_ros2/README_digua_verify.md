# base_control_ros2 功能包介绍

## 1. 功能包在项目中的作用

`base_control_ros2` 是地瓜机器人项目中的**底盘串口控制功能包**，主要负责把 ROS 2 系统中的速度控制、里程计、电池、电机控制板 IMU 等信息，与底层运动控制板之间进行转换和通信。

在整个机器人系统中，它处在 **ROS 2 上层导航/控制系统** 和 **底层 STM32/运动控制板** 之间，作用可以概括为：

```text
Nav2 / 遥控节点 / 智能体控制
        |
        |  geometry_msgs/msg/Twist
        v
base_control_ros2
        |
        |  串口协议 /dev/move_base、/dev/ttyS1 等
        v
底层运动控制板 / 电机驱动 / 编码器 / 电池检测 / IMU
```

它不是路径规划器，也不是导航算法本身，而是一个**硬件抽象层/驱动桥接层**：

- 接收上层发布的 `/cmd_vel` 速度指令；
- 将速度指令编码成底盘控制板可识别的串口数据帧；
- 周期性向底盘控制板查询里程计、电池、IMU 等数据；
- 将底层控制板返回的数据解析成标准 ROS 2 消息；
- 向导航系统发布 `/odom`、`/battery`、`/imu` 等话题；
- 可选发布 `odom -> base_footprint` 的 TF 变换。

在完整的地瓜机器人系统中，`base_control_ros2` 通常和 `robot_localization`、Nav2、雷达、深度相机等功能包配合使用：

- `base_control_ros2` 负责输出底盘原始里程计；
- `robot_localization` / EKF 负责融合编码器、IMU 等信息；
- Nav2 使用融合后的定位和地图进行路径规划与控制；
- 上层智能体、飞书控制、语音控制等模块最终也会间接通过 `/cmd_vel` 控制底盘运动。

需要特别注意：在完整机器人系统中，如果已经由 EKF 发布 `odom -> base_footprint`，则不应再让 `base_control_ros2` 同时发布同一条 TF，否则可能造成 TF 冲突。因此项目 launch 文件中默认将 `broadcast_odom_tf` 设置为 `false`。

---

## 2. 文件树与文件作用说明

```text
base_control_ros2/
├── base_control_ros2/
│   ├── __init__.py
│   │   └── Python 包标识文件，使该目录可以被 ROS 2 的 ament_python 机制识别为 Python 模块。
│   ├── base_control_ros2.py
│   │   └── 核心底盘控制节点，负责串口通信、/cmd_vel 接收、/odom /battery /imu 发布以及可选 TF 发布。
│   ├── loopqueue.py
│   │   └── 串口接收数据使用的环形队列工具，用于缓存并按帧解析底层控制板返回的数据。
│   └── test_node.py
│       └── 简单的测试入口节点，用于验证 Python 包和 console_scripts 是否能被正常安装与运行。
│
├── launch/
│   └── base_control.launch.py
│       └── 底盘控制节点启动文件，提供串口、话题名、坐标系、是否发布 IMU、是否发布 odom TF 等参数。
│
├── resource/
│   └── base_control_ros2
│       └── ament_python 包资源索引标记文件，用于让 ROS 2 找到该 Python 功能包。
│
├── script/
│   ├── initsetup.sh
│   │   └── 早期/基础串口 udev 配置脚本，常用于将 CH340 USB 转串口设备固定映射为 /dev/move_base。
│   ├── move_base_udev.sh
│   │   └── 综合 udev 配置脚本，用于为 STM32 USB CDC 或 CH340 串口设备创建稳定的 /dev/move_base 设备名。
│   ├── stm32_udev.sh
│   │   └── STM32 串口控制板相关的 udev 配置脚本，用于避免设备名变化导致节点找不到底盘串口。
│   ├── ttyS0_move_base_udev.sh
│   │   └── 将指定硬件串口 ttyS0 固定映射为 /dev/move_base 的 udev 配置脚本。
│   └── ttyS7_move_base_udev.sh
│       └── 将指定硬件串口 ttyS7 固定映射为 /dev/move_base 的 udev 配置脚本。
│
├── test/
│   ├── test_copyright.py
│   │   └── ROS 2 Python 包版权检查测试文件。
│   ├── test_flake8.py
│   │   └── Python 代码风格检查测试文件。
│   └── test_pep257.py
│       └── Python docstring 规范检查测试文件。
│
├── README.MD
│   └── 原始说明文档，包含该功能包依赖项等基础信息。
├── README_digua_verify.md
│   └── 地瓜机器人项目相关的底盘验证/调试说明文档。
├── package.xml
│   └── ROS 2 功能包元信息和运行依赖声明文件。
├── setup.cfg
│   └── Python 脚本安装路径配置文件。
└── setup.py
    └── ament_python 安装配置文件，声明 console_scripts 入口 base_control_node 和 test_node。
```

---

## 3. ROS 2 功能包类型与依赖

该功能包是一个 ROS 2 `ament_python` 功能包。

### 3.1 包基本信息

| 项目 | 内容 |
|---|---|
| 包名 | `base_control_ros2` |
| 构建类型 | `ament_python` |
| 主节点入口 | `base_control_node = base_control_ros2.base_control_ros2:main` |
| 测试节点入口 | `test_node = base_control_ros2.test_node:main` |
| 主要用途 | ROS 2 串口底盘控制、里程计发布、电池信息发布、IMU 发布 |

### 3.2 主要依赖

| 依赖 | 作用 |
|---|---|
| `rclpy` | ROS 2 Python 客户端库，用于创建节点、话题、定时器和参数。 |
| `geometry_msgs` | 使用 `geometry_msgs/msg/Twist` 接收速度控制指令。 |
| `nav_msgs` | 使用 `nav_msgs/msg/Odometry` 发布底盘里程计。 |
| `sensor_msgs` | 使用 `sensor_msgs/msg/BatteryState` 和 `sensor_msgs/msg/Imu` 发布电池和 IMU 数据。 |
| `tf2_ros` | 可选发布 `odom -> base_footprint` 坐标变换。 |
| `python3-serial` / `pyserial` | 通过串口与底层运动控制板通信。 |

---

## 4. 节点说明

### 4.1 核心节点

| 项目 | 内容 |
|---|---|
| 节点可执行名 | `base_control_node` |
| 默认节点名 | `base_control` |
| 所在文件 | `base_control_ros2/base_control_ros2.py` |
| 主要功能 | 接收速度指令，控制底盘，读取并发布里程计、电池、IMU 数据 |

### 4.2 启动方式

常用启动命令：

```bash
ros2 launch base_control_ros2 base_control.launch.py
```

如果使用固定的 udev 设备名 `/dev/move_base`：

```bash
ros2 launch base_control_ros2 base_control.launch.py port:=/dev/move_base
```

如果直接使用板载串口，例如 `/dev/ttyS1`：

```bash
ros2 launch base_control_ros2 base_control.launch.py port:=/dev/ttyS1 baudrate:=115200
```

---

## 5. ROS 2 接口说明

## 5.1 订阅者 Subscribers

### `/cmd_vel`

| 项目 | 内容 |
|---|---|
| 默认话题名 | `/cmd_vel` |
| 可配置参数 | `cmd_vel_topic` |
| 消息类型 | `geometry_msgs/msg/Twist` |
| 队列深度 | `10` |
| 作用 | 接收上层控制器、遥控节点或 Nav2 输出的底盘速度指令 |

`Twist` 消息中主要使用的字段：

| 字段 | 含义 |
|---|---|
| `linear.x` | 机器人前后方向线速度，单位 m/s。 |
| `linear.y` | 机器人左右方向线速度，单位 m/s；对于普通阿克曼底盘通常不直接使用，但协议中保留该字段。 |
| `angular.z` | 机器人绕 z 轴角速度，单位 rad/s。 |

节点接收到 `/cmd_vel` 后，会将速度值乘以 `1000`，转换成底层控制板协议需要的整数格式，并通过串口发送给底盘控制板。

---

## 5.2 发布者 Publishers

### `/odom`

| 项目 | 内容 |
|---|---|
| 默认话题名 | `/odom` |
| 可配置参数 | `odom_topic` |
| 消息类型 | `nav_msgs/msg/Odometry` |
| 默认发布频率 | `50 Hz` |
| 作用 | 发布底盘控制板返回并积分得到的里程计信息 |

关键字段说明：

| 字段 | 内容 |
|---|---|
| `header.frame_id` | 默认 `odom` |
| `child_frame_id` | 默认 `base_footprint` |
| `pose.pose.position.x/y` | 根据底盘速度和航向角积分得到的位置。 |
| `pose.pose.orientation` | 根据底盘返回的 yaw 角转换得到的四元数。 |
| `twist.twist.linear.x/y` | 底盘返回的线速度。 |
| `twist.twist.angular.z` | 底盘返回的角速度。 |

在导航系统中，`/odom` 通常会作为 EKF 或其他状态估计模块的输入。

---

### `/battery`

| 项目 | 内容 |
|---|---|
| 默认话题名 | `/battery` |
| 可配置参数 | `battery_topic` |
| 消息类型 | `sensor_msgs/msg/BatteryState` |
| 默认发布频率 | `1 Hz` |
| 作用 | 发布底盘控制板检测到的电池电压和电流信息 |

关键字段说明：

| 字段 | 内容 |
|---|---|
| `header.frame_id` | 默认 `base_footprint` |
| `voltage` | 电池电压，单位 V。 |
| `current` | 电池电流，单位 A。 |

该话题可用于上层监控、电量显示、低电量保护等功能。

---

### `/imu`

| 项目 | 内容 |
|---|---|
| 默认话题名 | `/imu` |
| 可配置参数 | `imu_topic` |
| 消息类型 | `sensor_msgs/msg/Imu` |
| 默认发布频率 | `50 Hz` |
| 是否默认发布 | 由 `pub_imu` 参数控制，launch 中默认为 `true` |
| 作用 | 发布底盘控制板或板载 IMU 返回的角速度、线加速度和姿态信息 |

关键字段说明：

| 字段 | 内容 |
|---|---|
| `header.frame_id` | 默认 `imu` |
| `angular_velocity.x/y/z` | 三轴角速度。 |
| `linear_acceleration.x/y/z` | 三轴线加速度。 |
| `orientation.x/y/z/w` | IMU 姿态四元数。 |

如果系统中已经有独立 IMU 或相机/雷达融合定位方案，可以根据需要关闭该话题：

```bash
ros2 launch base_control_ros2 base_control.launch.py pub_imu:=false
```

---

## 5.3 TF 发布

### `odom -> base_footprint`

| 项目 | 内容 |
|---|---|
| TF 话题 | `/tf` |
| 消息类型 | `tf2_msgs/msg/TFMessage` |
| 父坐标系 | 默认 `odom` |
| 子坐标系 | 默认 `base_footprint` |
| 控制参数 | `broadcast_odom_tf` |
| 代码默认值 | `true` |
| launch 默认值 | `false` |

当 `broadcast_odom_tf:=true` 时，节点会根据里程计数据发布：

```text
odom -> base_footprint
```

但是在完整的地瓜机器人系统中，通常由 `robot_localization` / EKF 发布这条 TF。因此推荐保持 launch 默认值：

```bash
ros2 launch base_control_ros2 base_control.launch.py broadcast_odom_tf:=false
```

只有在单独测试底盘、不启动 EKF 的情况下，才建议临时打开：

```bash
ros2 launch base_control_ros2 base_control.launch.py broadcast_odom_tf:=true
```

---

## 6. 参数说明

| 参数名 | 类型 | 默认值 | 作用 |
|---|---:|---|---|
| `port` | string | launch 默认 `/dev/ttyS1`，代码默认 `/dev/move_base` | 底盘控制板串口设备路径。 |
| `baudrate` | int | `115200` | 串口波特率。 |
| `cmd_vel_topic` | string | `cmd_vel` | 速度指令订阅话题名。 |
| `odom_topic` | string | `odom` | 里程计发布话题名。 |
| `battery_topic` | string | `battery` | 电池状态发布话题名。 |
| `imu_topic` | string | `imu` | IMU 发布话题名。 |
| `base_id` / `base_frame` | string | `base_footprint` | 机器人底盘坐标系。 |
| `odom_id` / `odom_frame` | string | `odom` | 里程计坐标系。 |
| `imu_id` / `imu_frame` | string | `imu` | IMU 坐标系。 |
| `odom_freq` | int/float | `50` | 里程计查询与发布频率。 |
| `battery_freq` | int/float | `1` | 电池信息查询与发布频率。 |
| `imu_freq` | int/float | `50` | IMU 查询与发布频率。 |
| `communication_freq` | int/float | `100` | 串口接收缓存读取和协议解析频率。 |
| `pub_imu` | bool | `true` | 是否发布 `/imu`。 |
| `broadcast_odom_tf` | bool | launch 默认 `false` | 是否发布 `odom -> base_footprint` TF。 |
| `legacy_odom_cmd` | bool | `false` | 是否使用旧版里程计查询协议。 |

---

## 7. 串口通信与底层协议关系

`base_control_ros2` 通过串口与底层运动控制板通信，默认波特率为 `115200`。

### 7.1 串口设备

常见设备路径包括：

```text
/dev/move_base
/dev/ttyS1
/dev/ttyS0
/dev/ttyS7
/dev/ttyUSB0
/dev/ttyACM0
```

实际项目中推荐使用 udev 规则固定设备名为：

```text
/dev/move_base
```

这样可以避免 USB 转串口设备重新插拔后设备名变化，例如 `/dev/ttyUSB0` 变成 `/dev/ttyUSB1`，导致节点启动失败。

### 7.2 主要协议功能

| 功能 | ROS 2 侧表现 | 串口协议作用 |
|---|---|---|
| 速度控制 | 订阅 `/cmd_vel` | 将 `linear.x`、`linear.y`、`angular.z` 编码后发送到底盘控制板。 |
| 里程计读取 | 发布 `/odom` | 周期性查询底盘速度、yaw 角或里程计数据。 |
| 电池读取 | 发布 `/battery` | 周期性查询电压、电流信息。 |
| IMU 读取 | 发布 `/imu` | 周期性查询角速度、线加速度、姿态四元数。 |
| 版本/SN/底盘信息 | 日志或内部状态 | 可读取控制板版本、SN、底盘参数等信息。 |

### 7.3 新旧里程计协议

节点支持两种里程计查询方式：

| 参数 | 查询方式 | 说明 |
|---|---|---|
| `legacy_odom_cmd:=false` | 新版协议 | 默认方式，适合当前项目使用。 |
| `legacy_odom_cmd:=true` | 旧版协议 | 兼容旧固件或旧控制板。 |

如果发现 `/odom` 无数据或数据异常，可以检查底层控制板固件协议版本，并尝试切换该参数。

---

## 8. 在项目中的典型数据流

### 8.1 遥控/导航控制底盘

```text
键盘遥控 / Nav2 Controller / 智能体控制
        |
        | /cmd_vel
        | geometry_msgs/msg/Twist
        v
base_control_node
        |
        | 串口速度控制帧
        v
底盘控制板
        |
        v
电机驱动与车轮运动
```

### 8.2 底盘数据回传 ROS 2

```text
编码器 / IMU / 电池检测
        |
        v
底盘控制板
        |
        | 串口数据帧
        v
base_control_node
        |
        ├── /odom     nav_msgs/msg/Odometry
        ├── /battery  sensor_msgs/msg/BatteryState
        └── /imu      sensor_msgs/msg/Imu
```

### 8.3 与 EKF 的关系

推荐完整系统中的关系如下：

```text
base_control_ros2
    ├── /odom
    └── /imu
          |
          v
robot_localization / ekf_filter_node
          |
          ├── /odometry/filtered
          └── odom -> base_footprint TF
```

因此完整系统中建议：

```bash
broadcast_odom_tf:=false
```

防止 `base_control_ros2` 和 EKF 同时发布同一条 TF。

---

## 9. 构建与运行

### 9.1 安装依赖

```bash
sudo apt update
sudo apt install -y python3-serial
```

如果在 ROS 2 工作空间中统一安装依赖，也可以使用：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 9.2 编译功能包

在工作空间根目录下执行：

```bash
colcon build --packages-select base_control_ros2
source install/setup.bash
```

### 9.3 配置串口设备名

推荐执行 udev 配置脚本，将底盘控制板固定为 `/dev/move_base`：

```bash
sudo bash src/base_control_ros2/script/move_base_udev.sh
```

配置后可以检查设备是否存在：

```bash
ls -l /dev/move_base
```

如果串口权限不足，可以确认当前用户是否在 `dialout` 用户组中：

```bash
groups
```

必要时执行：

```bash
sudo usermod -aG dialout $USER
```

然后重新登录终端或重启系统。

### 9.4 启动底盘控制节点

```bash
ros2 launch base_control_ros2 base_control.launch.py port:=/dev/move_base
```

### 9.5 检查话题

```bash
ros2 topic list
```

正常情况下应能看到：

```text
/cmd_vel
/odom
/battery
/imu
```

检查里程计：

```bash
ros2 topic echo /odom --once
```

检查电池：

```bash
ros2 topic echo /battery --once
```

检查 IMU：

```bash
ros2 topic echo /imu --once
```

---

## 10. 调试建议

### 10.1 节点启动后无法连接串口

可能原因：

- `port` 参数设置错误；
- `/dev/move_base` 没有正确创建；
- 串口设备被其他程序占用；
- 当前用户没有串口访问权限；
- 底盘控制板未上电或 USB/串口线未连接。

检查命令：

```bash
ls -l /dev/move_base
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
```

### 10.2 `/cmd_vel` 有数据但底盘不动

建议检查：

- `/cmd_vel` 是否真的有速度数据；
- 速度值是否太小；
- 串口是否连接成功；
- 底层控制板协议是否匹配；
- 底盘急停、电源、电机驱动是否正常。

检查 `/cmd_vel`：

```bash
ros2 topic echo /cmd_vel
```

手动发送速度测试：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

### 10.3 TF 冲突

如果系统中已经启动 EKF，建议不要让本节点发布 `odom -> base_footprint`：

```bash
ros2 launch base_control_ros2 base_control.launch.py broadcast_odom_tf:=false
```

如果只单独测试底盘，没有启动 EKF，可以临时开启：

```bash
ros2 launch base_control_ros2 base_control.launch.py broadcast_odom_tf:=true
```

### 10.4 `/odom` 数据异常

建议检查：

- 底层控制板固件协议是否为新版；
- 是否需要设置 `legacy_odom_cmd:=true`；
- 编码器方向和比例是否正确；
- 机器人实际底盘运动模型是否和上层控制指令一致；
- 是否在机器人悬空、打滑、被搬动时记录了错误里程计。

---

## 11. 总结

`base_control_ros2` 是地瓜机器人项目中非常关键的底盘驱动功能包。它向上提供标准 ROS 2 接口，向下对接真实底盘控制板，使 Nav2、遥控节点、智能体控制、飞书控制等上层模块能够通过统一的 `/cmd_vel` 控制机器人运动，同时获得 `/odom`、`/battery`、`/imu` 等底盘状态反馈。

它的定位可以概括为：

```text
base_control_ros2 = ROS 2 系统 与 底层运动控制板 之间的串口桥接驱动层
```

在完整机器人系统中，推荐让本功能包只负责发布底盘原始数据，由 EKF 负责最终里程计融合和 TF 发布，这样系统结构更清晰，也能避免 TF 重复发布造成的问题。
