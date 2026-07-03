# digua_description

`digua_description` 是地瓜机器人的模型描述包，负责保存整车 URDF/Xacro、RViz 可视化配置和模型显示启动文件。它不控制底盘，也不做建图导航；它的核心作用是向 ROS 2 系统提供 `robot_description` 和传感器安装 TF，让雷达、相机、IMU、底盘和上层算法处在同一套坐标系中。

它在整车系统中的位置：

```text
digua_description
        |
        | robot_description + TF
        v
robot_state_publisher
        |
        | base_footprint -> base_link -> laser_frame / camera_link / imu
        v
digua_bringup / digua_mapping / digua_navigation / digua_semantic_mapping
```

常用启动：

```bash
ros2 launch digua_description display.launch.py rviz:=false
```

需要在本机查看模型时：

```bash
ros2 launch digua_description display.launch.py rviz:=true
```

## 在整个项目里的作用

`digua_description` 是整车空间关系的源头。建图、定位、导航和语义地图都依赖正确的 TF：

- `digua_mapping` 需要知道 `base_footprint`、`laser_frame`、`camera_link` 等关系，才能融合雷达、RGB-D 和里程计。
- `digua_navigation` 需要稳定的 `base_footprint` 和传感器 TF，才能正确生成代价地图和避障行为。
- `digua_semantic_mapping` 需要相机坐标系到机器人/地图坐标系的变换，才能把二维检测框和深度转换为地图中的语义目标。
- `digua_bringup` 会 include 本包的 `display.launch.py`，作为整车启动时的模型和 TF 发布入口。

一句话概括：

```text
digua_description = 地瓜机器人 URDF 模型 + 传感器安装 TF + RViz 可视化入口
```

## 文件树

```text
digua_description/
├── launch/
│   └── display.launch.py              # 启动 robot_state_publisher，可选启动 RViz2
├── rviz/
│   └── digua_description.rviz         # 模型和 TF 可视化配置
├── urdf/
│   └── digua_robot.urdf.xacro         # 地瓜机器人 URDF/Xacro 模型
├── CMakeLists.txt                     # ament_cmake 安装 urdf/launch/rviz
├── package.xml                        # ROS 2 包元信息和运行依赖
└── README.md                          # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `urdf/digua_robot.urdf.xacro` | 机器人模型核心文件。定义 `base_footprint`、`base_link`、`laser_frame`、`camera_link`、`imu` 和轮子可视化 link。 |
| `launch/display.launch.py` | 读取 Xacro，生成 `robot_description` 参数，启动 `robot_state_publisher`；`rviz:=true` 时同时启动 RViz2。 |
| `rviz/digua_description.rviz` | RViz 配置，默认显示 Grid、RobotModel 和 TF，固定坐标系为 `base_footprint`。 |
| `CMakeLists.txt` | 安装 `urdf/`、`launch/`、`rviz/` 到 ROS 2 包 share 目录。 |
| `package.xml` | 声明 `xacro`、`robot_state_publisher`、`rviz2` 等运行依赖。 |

## 模型坐标约定

URDF 中采用常见 ROS 机器人坐标约定：

```text
+X：机器人前方
+Y：机器人左侧
+Z：机器人上方
```

核心坐标系：

| 坐标系 | 说明 |
| --- | --- |
| `base_footprint` | 机器人底盘在地面上的投影坐标系，当前定义为后轴中心在地面的投影。 |
| `base_link` | 机器人本体坐标系，位于 `base_footprint` 上方，作为传感器安装关系的父坐标系。 |
| `laser_frame` | YDLIDAR X2 雷达坐标系。 |
| `camera_link` | Astra 相机外壳/基准坐标系；相机内部 color/depth/optical frame 由 `astra_camera` 发布。 |
| `imu` | 底盘或控制板 IMU 坐标系。 |
| `front_left_wheel_visual` 等 | 轮子可视化 link，只用于 RViz 显示，不参与运动学控制。 |

## TF 树

`digua_robot.urdf.xacro` 当前定义的固定 TF 树：

```text
base_footprint
└── base_link
    ├── laser_frame
    ├── camera_link
    ├── imu
    ├── front_left_wheel_visual
    ├── front_right_wheel_visual
    ├── rear_left_wheel_visual
    └── rear_right_wheel_visual
```

关键安装参数：

| 子坐标系 | 父坐标系 | xyz, 单位 m | rpy, 单位 rad | 说明 |
| --- | --- | --- | --- | --- |
| `base_link` | `base_footprint` | `0 0 0.068` | `0 0 0` | 本体坐标系相对地面投影的高度。 |
| `laser_frame` | `base_link` | `0.100 0.000 0.087` | `0 0 0` | 雷达安装位置。 |
| `camera_link` | `base_link` | `0.170 0.000 0.022` | `0 0 0` | Astra 相机安装位置。 |
| `imu` | `base_link` | `0.006 0.028 0.022` | `0 0 0` | IMU 安装位置。 |

轮子可视化位置：

| link | xyz, 单位 m | 说明 |
| --- | --- | --- |
| `front_left_wheel_visual` | `0.22 0.105 -0.068` | 左前轮显示模型。 |
| `front_right_wheel_visual` | `0.22 -0.105 -0.068` | 右前轮显示模型。 |
| `rear_left_wheel_visual` | `0.00 0.105 -0.068` | 左后轮显示模型。 |
| `rear_right_wheel_visual` | `0.00 -0.105 -0.068` | 右后轮显示模型。 |

## ROS 2 接口

`digua_description` 本身没有自定义 C++/Python 节点。它通过 `display.launch.py` 启动标准 ROS 2 节点。

### `robot_state_publisher`

| 发布/订阅 | 话题或 TF | 数据类型 | 作用 |
| --- | --- | --- | --- |
| 参数输入 | `robot_description` | string 参数 | 由 `xacro digua_robot.urdf.xacro` 生成的机器人模型 XML。 |
| 发布 | `/tf_static` | `tf2_msgs/msg/TFMessage` | 发布 URDF 中固定关节对应的静态 TF。 |
| 发布 | `/tf` | `tf2_msgs/msg/TFMessage` | 若模型存在动态关节，会发布动态 TF；当前模型主要是固定关节。 |
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | 动态关节状态输入。当前模型没有用于控制的动态关节，通常不需要额外发布。 |

### `rviz2`

当 `rviz:=true` 时启动：

| 使用内容 | 数据类型/来源 | 作用 |
| --- | --- | --- |
| RobotModel | `robot_description` / `/robot_description` | 显示机器人模型。 |
| TF | `/tf`、`/tf_static` | 显示坐标系树。 |
| Grid | RViz 内置显示 | 以 `base_footprint` 为固定坐标系显示网格。 |

### Service 与 Action

该包不定义 ROS 2 service，也不定义 ROS 2 action。

## Launch 参数

`display.launch.py` 支持：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `false` | 是否使用仿真时间。真实机器人应保持 `false`。 |
| `rviz` | `false` | 是否启动 RViz2。RDK X5 上正常 bringup 通常保持 `false`，在虚拟机或开发机可设为 `true`。 |

## 常用命令

只发布模型与 TF，不启动 RViz：

```bash
ros2 launch digua_description display.launch.py rviz:=false
```

启动 RViz 查看模型和 TF：

```bash
ros2 launch digua_description display.launch.py rviz:=true
```

检查 `base_link` 到雷达坐标系：

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

检查 `base_link` 到相机坐标系：

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

检查 `base_link` 到 IMU 坐标系：

```bash
ros2 run tf2_ros tf2_echo base_link imu
```
