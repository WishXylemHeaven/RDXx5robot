# base_control_ros2

`base_control_ros2` 是地瓜机器人项目中的底盘串口控制包，负责把 ROS 2 上层的速度指令转换为底层控制板串口协议，同时把底盘控制板返回的里程计、电池和 IMU 数据转换为标准 ROS 2 话题。

它在整车系统中的位置很靠底层：

```text
Nav2 / 遥控节点 / 语义导航 / 上层智能体
        |
        | /cmd_vel
        | geometry_msgs/msg/Twist
        v
base_control_ros2
        |
        | 串口协议，常用 /dev/ttyS1 或 /dev/move_base
        v
Nano / STM32 底盘控制板
        |
        v
电机、转向舵机、编码器、IMU、电池检测
```

## 在整个项目里的作用

`base_control_ros2` 是地瓜机器人移动能力的硬件抽象层。上层不需要直接理解底盘串口协议，只需要发布标准 `/cmd_vel`；本包负责把速度指令下发到底盘控制板，并把底层状态转换成 ROS 2 标准消息。它让 Nav2、自主探索、命名点位导航、语义目标导航和后续飞书/语音交互都可以通过统一接口控制真实机器人。

一句话概括：

```text
base_control_ros2 = ROS 2 上层算法 与 真实底盘控制板 之间的串口桥接驱动
```

在完整系统中，`base_control_ros2` 通常只负责底盘原始数据和串口控制；`robot_localization` 的 EKF 节点负责融合 `/odom` 与 `/imu`，并发布最终的 `odom -> base_footprint` TF。为了避免 TF 冲突，本包的 launch 文件默认关闭 `broadcast_odom_tf`。

## 文件树

```text
base_control_ros2/
├── base_control_ros2/
│   ├── __init__.py                 # Python 包标识文件
│   ├── base_control_ros2.py         # 核心底盘控制节点
│   ├── loopqueue.py                 # 环形队列工具，保留给串口帧缓存/解析使用
│   └── test_node.py                 # 简单安装测试入口
├── launch/
│   └── base_control.launch.py       # 底盘控制节点启动文件
├── resource/
│   └── base_control_ros2            # ament_python 包资源索引
├── script/
│   ├── initsetup.sh                 # 基础 udev 初始化脚本
│   ├── move_base_udev.sh            # 为底盘串口创建 /dev/move_base 的 udev 脚本
│   ├── stm32_udev.sh                # STM32 控制板串口 udev 脚本
│   ├── ttyS0_move_base_udev.sh      # 将 ttyS0 固定映射为 /dev/move_base
│   └── ttyS7_move_base_udev.sh      # 将 ttyS7 固定映射为 /dev/move_base
├── test/
│   ├── test_copyright.py            # ROS 2 包版权检查
│   ├── test_flake8.py               # Python 代码风格检查
│   └── test_pep257.py               # Python docstring 检查
├── package.xml                      # ROS 2 包元信息和依赖声明
├── setup.cfg                        # Python 脚本安装路径配置
├── setup.py                         # ament_python 安装配置和 console_scripts 入口
└── README.md                        # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `base_control_ros2/base_control_ros2.py` | 核心节点。打开串口、订阅速度指令、发送底盘控制帧、周期查询底盘数据、发布 `/odom`、`/battery`、`/imu`，并可选发布 odom TF。 |
| `base_control_ros2/loopqueue.py` | 环形队列工具，用于缓存串口接收字节并按协议帧解析。当前主节点内部也有一份同类队列实现。 |
| `base_control_ros2/test_node.py` | 极简测试入口，主要用于验证包安装和 console script 是否可运行。 |
| `launch/base_control.launch.py` | 推荐启动入口。集中配置串口、话题名、坐标系、IMU 发布开关、TF 发布开关和新旧里程计协议。 |
| `script/*.sh` | 串口设备固定命名脚本，主要用于把实际控制板设备稳定映射为 `/dev/move_base`。 |
| `package.xml` | 声明 `rclpy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf2_ros`、`python3-serial` 等依赖。 |
| `setup.py` | 声明包名、安装文件和命令入口：`base_control_node`、`test_node`。 |
| `setup.cfg` | 指定 ROS 2 Python 可执行脚本安装到 `lib/base_control_ros2`。 |
| `test/*.py` | ROS 2 默认 lint/版权测试文件。 |

## 节点与入口

| 项目 | 内容 |
| --- | --- |
| 包名 | `base_control_ros2` |
| 构建类型 | `ament_python` |
| 核心可执行入口 | `base_control_node` |
| 核心节点名 | `base_control` |
| 核心源码 | `base_control_ros2/base_control_ros2.py` |
| 测试可执行入口 | `test_node` |

常用启动命令：

```bash
ros2 launch base_control_ros2 base_control.launch.py
```

指定串口启动：

```bash
ros2 launch base_control_ros2 base_control.launch.py port:=/dev/ttyS1 baudrate:=115200
```

如果已经配置好 `/dev/move_base`：

```bash
ros2 launch base_control_ros2 base_control.launch.py port:=/dev/move_base
```

## ROS 2 接口

### 订阅者

| 话题 | 数据类型 | 默认队列 | 作用 |
| --- | --- | ---: | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 10 | 接收 Nav2、遥控节点或上层控制模块输出的底盘速度指令。 |

`/cmd_vel` 使用字段：

| 字段 | 含义 |
| --- | --- |
| `linear.x` | 前后方向线速度，单位 m/s。 |
| `linear.y` | 左右方向线速度，协议中保留该字段；阿克曼底盘通常不主动使用横移能力。 |
| `angular.z` | 绕 z 轴角速度，单位 rad/s。 |

节点接收到 `Twist` 后，会把速度值乘以 `1000` 转为底层协议整数，并写入串口控制帧。

### 发布者

| 话题 | 数据类型 | 默认频率 | 作用 |
| --- | --- | ---: | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | 发布底盘速度和航向积分得到的原始里程计。 |
| `/battery` | `sensor_msgs/msg/BatteryState` | 1 Hz | 发布底盘控制板检测到的电池电压和电流。 |
| `/imu` | `sensor_msgs/msg/Imu` | 50 Hz | 发布底盘控制板或板载 IMU 返回的角速度、线加速度和姿态四元数。 |

`/odom` 关键字段：

| 字段 | 默认/含义 |
| --- | --- |
| `header.frame_id` | 默认 `odom` |
| `child_frame_id` | 默认 `base_footprint` |
| `pose.pose.position.x/y` | 根据底盘速度和 yaw 积分得到的位置。 |
| `pose.pose.orientation` | 根据底盘 yaw 转换得到的四元数。 |
| `twist.twist.linear.x/y` | 底盘回传的线速度。 |
| `twist.twist.angular.z` | 底盘回传的角速度。 |

`/battery` 关键字段：

| 字段 | 默认/含义 |
| --- | --- |
| `header.frame_id` | 默认 `base_footprint` |
| `voltage` | 电池电压，单位 V。 |
| `current` | 电池电流，单位 A。 |

`/imu` 关键字段：

| 字段 | 含义 |
| --- | --- |
| `header.frame_id` | 默认 `imu` |
| `angular_velocity.x/y/z` | 三轴角速度。 |
| `linear_acceleration.x/y/z` | 三轴线加速度。 |
| `orientation.x/y/z/w` | IMU 姿态四元数。 |

### TF

| TF | 数据类型 | 控制参数 | 代码默认值 | launch 默认值 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `odom -> base_footprint` | `tf2_msgs/msg/TFMessage` via `/tf` | `broadcast_odom_tf` | `true` | `false` | 单独测试底盘时可开启；完整系统中通常由 EKF 发布。 |

### 服务与 Action

该包不定义 ROS 2 service，也不定义 ROS 2 action。它只通过 topic、TF 和串口协议工作。

## 参数

| 参数 | 类型 | 代码默认值 | launch 默认值 | 说明 |
| --- | --- | --- | --- | --- |
| `port` | string | `/dev/move_base` | `/dev/ttyS1` | 底盘控制板串口设备路径。 |
| `baudrate` | int | `115200` | `115200` | 串口波特率。 |
| `cmd_vel_topic` | string | `cmd_vel` | `cmd_vel` | 速度指令订阅话题名。 |
| `odom_topic` | string | `odom` | `odom` | 里程计发布话题名。 |
| `battery_topic` | string | `battery` | `battery` | 电池状态发布话题名。 |
| `imu_topic` | string | `imu` | `imu` | IMU 发布话题名。 |
| `base_id` / `base_frame` | string | `base_footprint` | `base_footprint` | 底盘坐标系。 |
| `odom_id` / `odom_frame` | string | `odom` | `odom` | 里程计坐标系。 |
| `imu_id` / `imu_frame` | string | `imu` | `imu` | IMU 坐标系。 |
| `odom_freq` | float | `50.0` | 未暴露 | `/odom` 查询与发布频率。 |
| `battery_freq` | float | `1.0` | 未暴露 | `/battery` 查询与发布频率。 |
| `imu_freq` | float | `50.0` | 未暴露 | `/imu` 查询与发布频率。 |
| `communication_freq` | float | `100.0` | 未暴露 | 串口读取和协议解析频率。 |
| `pub_imu` | bool | `true` | `true` | 是否发布 `/imu`。 |
| `broadcast_odom_tf` | bool | `true` | `false` | 是否发布 `odom -> base_footprint`。 |
| `legacy_odom_cmd` | bool | `false` | `false` | 是否使用旧版里程计查询协议。 |

## 数据流

### 速度控制链路

```text
Nav2 Controller / 键盘遥控 / 上层任务节点
        |
        | /cmd_vel
        v
base_control_node
        |
        | 串口速度控制帧
        v
底盘控制板
        |
        v
电机与转向机构
```

### 底盘状态回传链路

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

### 与 EKF 的关系

完整系统推荐关系：

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

因此，整车运行时建议保持：

```bash
ros2 launch base_control_ros2 base_control.launch.py broadcast_odom_tf:=false
```

单独测试底盘、没有启动 EKF 时，可以临时开启：

```bash
ros2 launch base_control_ros2 base_control.launch.py broadcast_odom_tf:=true
```

## 常用调试

查看话题：

```bash
ros2 topic list
```

查看里程计：

```bash
ros2 topic echo /odom --once
```

查看电池：

```bash
ros2 topic echo /battery --once
```

查看 IMU：

```bash
ros2 topic echo /imu --once
```

检查速度指令：

```bash
timeout 5 ros2 topic echo /cmd_vel
```

手动发送低速前进指令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

检查串口设备：

```bash
ls -l /dev/ttyS1
ls -l /dev/move_base
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
```

配置 `/dev/move_base`：

```bash
sudo bash src/base_control_ros2/script/move_base_udev.sh
```
