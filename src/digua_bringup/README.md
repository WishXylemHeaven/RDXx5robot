# digua_bringup

`digua_bringup` 是地瓜机器人真实底盘的总启动包。它本身不实现新的算法节点，而是把机器人描述、底盘串口控制、EKF 融合定位、YDLIDAR X2、Astra RGB-D 相机和 RGB-D 同步节点组合到同一个启动入口中。

它在项目里的位置：

```text
digua_description      -> 机器人模型和静态 TF
base_control_ros2      -> 底盘 /odom /imu /battery /cmd_vel
robot_localization     -> EKF 融合 odom 与 imu
ydlidar_ros2_driver    -> /scan
astra_camera           -> 彩色图、深度图、相机内参、点云
rtabmap_sync           -> /camera/rgbd_image
        |
        v
digua_bringup          -> 真实机器人底层一键启动入口
        |
        v
digua_mapping / digua_navigation / digua_semantic_mapping
```

常用一键启动：

```bash
ros2 launch digua_bringup bringup_real.launch.py
```

## 在整个项目里的作用

`digua_bringup` 是地瓜机器人从“零散功能包”进入“整车可运行状态”的入口。它把底盘、雷达、相机、TF 和 EKF 的启动顺序、默认参数和互斥关系整理到一起，使后续建图、导航、探索、YOLO 识别和语义地图节点可以直接建立在稳定的底层数据流上。

一句话概括：

```text
digua_bringup = 真实机器人底层传感器、底盘与 TF/EKF 的总启动编排层
```

## 文件树

```text
digua_bringup/
├── config/
│   └── ekf.yaml                    # robot_localization EKF 参数
├── launch/
│   ├── bringup_real.launch.py      # 真实机器人底层一键启动入口
│   ├── ekf.launch.py               # 单独启动 EKF
│   ├── lidar_x2.launch.py          # 单独启动 YDLIDAR X2
│   └── rgbd_sync.launch.xml        # 单独启动 RGB-D 同步节点
├── CMakeLists.txt                  # ament_cmake 安装 launch/config
├── package.xml                     # ROS 2 包元信息和运行依赖
└── README.md                       # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `launch/bringup_real.launch.py` | 总启动入口。按开关参数 include 机器人描述、底盘控制、EKF、雷达、Astra 相机和 RGB-D 同步。 |
| `launch/ekf.launch.py` | 启动 `robot_localization/ekf_node`，加载 `config/ekf.yaml`。 |
| `launch/lidar_x2.launch.py` | 启动 `ydlidar_ros2_driver_node`，默认加载 `ydlidar_ros2_driver/params/X2.yaml`。 |
| `launch/rgbd_sync.launch.xml` | 启动 `rtabmap_sync/rgbd_sync`，将彩色图、深度图和相机内参同步成 `/camera/rgbd_image`。 |
| `config/ekf.yaml` | EKF 配置。融合 `/odom` 和 `/imu`，输出 `/odometry/filtered`，并发布 `odom -> base_footprint`。 |
| `CMakeLists.txt` | 安装 `launch/` 和 `config/` 到包 share 目录。 |
| `package.xml` | 声明 `digua_description`、`base_control_ros2`、`robot_localization`、`ydlidar_ros2_driver`、`astra_camera`、`rtabmap_sync` 等运行依赖。 |

## 启动文件

### `bringup_real.launch.py`

真实机器人底层总启动入口，默认启动以下模块：

| 模块 | 默认开关 | 启动内容 |
| --- | --- | --- |
| 机器人描述 | `start_description:=true` | include `digua_description/display.launch.py`，启动 `robot_state_publisher`，不启动 RViz。 |
| 底盘控制 | `start_base:=true` | include `base_control_ros2/base_control.launch.py`，默认串口 `/dev/ttyS1`，波特率 `115200`。 |
| EKF | `start_ekf:=true` | include `digua_bringup/ekf.launch.py`。 |
| 雷达 | `start_lidar:=true` | include `digua_bringup/lidar_x2.launch.py`。 |
| Astra 相机 | `start_camera:=true` | include `astra_camera/astra.launch.xml`。 |
| RGB-D 同步 | `start_rgbd_sync:=true` | 延迟 3 秒 include `rgbd_sync.launch.xml`，等待相机话题先出现。 |

常用命令：

```bash
ros2 launch digua_bringup bringup_real.launch.py
```

只启动底盘、描述和 EKF，不启动雷达相机：

```bash
ros2 launch digua_bringup bringup_real.launch.py start_lidar:=false start_camera:=false start_rgbd_sync:=false
```

指定底盘串口：

```bash
ros2 launch digua_bringup bringup_real.launch.py base_port:=/dev/move_base base_baudrate:=115200
```

### `ekf.launch.py`

单独启动 EKF：

```bash
ros2 launch digua_bringup ekf.launch.py
```

该启动文件启动：

| 节点 | 包 | 可执行文件 | 作用 |
| --- | --- | --- | --- |
| `ekf_filter_node` | `robot_localization` | `ekf_node` | 融合底盘里程计和 IMU，发布滤波后的里程计与 TF。 |

### `lidar_x2.launch.py`

单独启动 YDLIDAR X2：

```bash
ros2 launch digua_bringup lidar_x2.launch.py
```

默认参数文件：

```text
ydlidar_ros2_driver/params/X2.yaml
```

### `rgbd_sync.launch.xml`

单独启动 RGB-D 同步：

```bash
ros2 launch digua_bringup rgbd_sync.launch.xml
```

该节点把 Astra 的彩色图、深度图和相机内参同步成 RTAB-Map 使用的 RGBDImage。

## ROS 2 接口

`digua_bringup` 本身没有自定义 Python/C++ 节点，因此不直接定义新的 publisher、subscriber、service 或 action。它通过 launch 组合其他包的节点。下面列出本包默认启动链路中最重要的 ROS 2 接口。

### 机器人描述与 TF

| 节点 | 发布/订阅 | 话题或 TF | 数据类型 | 作用 |
| --- | --- | --- | --- | --- |
| `robot_state_publisher` | 发布 | `/tf_static`、`/tf` | `tf2_msgs/msg/TFMessage` | 根据 `digua_description` 的 URDF/Xacro 发布机器人本体和传感器坐标关系。 |
| `robot_state_publisher` | 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | 若模型包含可动关节，则读取关节状态。当前主要依赖固定传感器安装关系。 |

### 底盘控制

由 `base_control_ros2` 启动：

| 节点 | 发布/订阅 | 话题 | 数据类型 | 作用 |
| --- | --- | --- | --- | --- |
| `base_control` | 订阅 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 接收 Nav2 或遥控节点输出的底盘速度。 |
| `base_control` | 发布 | `/odom` | `nav_msgs/msg/Odometry` | 发布底盘原始里程计。 |
| `base_control` | 发布 | `/imu` | `sensor_msgs/msg/Imu` | 发布底盘 IMU 数据。 |
| `base_control` | 发布 | `/battery` | `sensor_msgs/msg/BatteryState` | 发布电池状态。 |

`bringup_real.launch.py` 强制传入 `broadcast_odom_tf:=false`，避免底盘节点和 EKF 同时发布 `odom -> base_footprint`。

### EKF 融合

由 `robot_localization` 启动：

| 节点 | 发布/订阅 | 话题或 TF | 数据类型 | 作用 |
| --- | --- | --- | --- | --- |
| `ekf_filter_node` | 订阅 | `/odom` | `nav_msgs/msg/Odometry` | 输入底盘原始里程计。 |
| `ekf_filter_node` | 订阅 | `/imu` | `sensor_msgs/msg/Imu` | 输入 IMU yaw 和 yaw 角速度。 |
| `ekf_filter_node` | 发布 | `/odometry/filtered` | `nav_msgs/msg/Odometry` | 输出滤波后的里程计。 |
| `ekf_filter_node` | 发布 | `/tf` | `tf2_msgs/msg/TFMessage` | 发布 `odom -> base_footprint`。 |

`config/ekf.yaml` 当前为 2D 模式，`world_frame` 为 `odom`，主要融合 x/y/yaw、速度和 yaw rate。

### 激光雷达

由 `ydlidar_ros2_driver` 启动：

| 节点 | 发布/订阅 | 话题 | 数据类型 | 作用 |
| --- | --- | --- | --- | --- |
| `ydlidar_ros2_driver_node` | 发布 | `/scan` | `sensor_msgs/msg/LaserScan` | 发布 YDLIDAR X2 激光扫描数据。 |

默认参数要点：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | 雷达串口。 |
| `frame_id` | `laser_frame` | 雷达坐标系。静态 TF 由 `digua_description` 提供。 |
| `range_min` / `range_max` | `0.1` / `8.0` | 有效距离范围。 |
| `frequency` | `6.0` | 期望扫描频率。 |

### Astra RGB-D 相机

由 `astra_camera/astra.launch.xml` 启动，默认 namespace 为 `/camera`：

| 节点 | 发布/订阅 | 话题 | 数据类型 | 作用 |
| --- | --- | --- | --- | --- |
| `camera` | 发布 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 彩色图像。 |
| `camera` | 发布 | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色相机内参。 |
| `camera` | 发布 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图像。 |
| `camera` | 发布 | `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度相机内参。 |
| `camera` | 发布 | `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` | 点云。`bringup_real.launch.py` 默认 `astra_enable_point_cloud:=true`。 |
| `camera` | 发布 | `/tf` | `tf2_msgs/msg/TFMessage` | 相机内部 TF。由 `astra_publish_tf` 控制，默认 `true`。 |

### RGB-D 同步

由 `rtabmap_sync/rgbd_sync` 启动：

| 节点 | 发布/订阅 | 话题 | 数据类型 | 作用 |
| --- | --- | --- | --- | --- |
| `rgbd_sync` | 订阅 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 输入彩色图。 |
| `rgbd_sync` | 订阅 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 输入深度图。 |
| `rgbd_sync` | 订阅 | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 输入相机内参。 |
| `rgbd_sync` | 发布 | `/camera/rgbd_image` | `rtabmap_msgs/msg/RGBDImage` | 输出 RTAB-Map 建图/定位使用的同步 RGB-D 数据。 |

## Launch 参数

### `bringup_real.launch.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `false` | 真实机器人应保持 false。 |
| `start_description` | `true` | 是否启动机器人描述和静态 TF。 |
| `start_base` | `true` | 是否启动底盘控制。 |
| `start_ekf` | `true` | 是否启动 EKF。 |
| `start_lidar` | `true` | 是否启动 YDLIDAR X2。 |
| `start_camera` | `true` | 是否启动 Astra S 相机。 |
| `start_rgbd_sync` | `true` | 是否启动 RGB-D 同步。 |
| `base_port` | `/dev/ttyS1` | 底盘下位机串口。 |
| `base_baudrate` | `115200` | 底盘串口波特率。 |
| `astra_enable_point_cloud` | `true` | 是否启用 Astra 点云。 |
| `astra_enable_ir` | `false` | 是否启用 IR 图像。 |
| `astra_publish_tf` | `true` | 是否让 Astra 发布内部相机 TF。 |

### `rgbd_sync.launch.xml`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `approx_sync` | `true` | 是否使用近似时间同步。 |
| `sync_queue_size` | `10` | 同步队列大小。 |
| `topic_queue_size` | `10` | 话题订阅队列大小。 |
| `approx_sync_max_interval` | `0.03` | 近似同步最大时间差，单位秒。 |
| `rgb_image_topic` | `/camera/color/image_raw` | 彩色图输入。 |
| `depth_image_topic` | `/camera/depth/image_raw` | 深度图输入。 |
| `rgb_camera_info_topic` | `/camera/color/camera_info` | 彩色相机内参输入。 |
| `rgbd_image_topic` | `/camera/rgbd_image` | RGB-D 同步输出。 |

## 常用命令

底层一键启动：

```bash
ros2 launch digua_bringup bringup_real.launch.py
```

分模块启动：

```bash
ros2 launch digua_description display.launch.py rviz:=false
ros2 launch base_control_ros2 base_control.launch.py
ros2 launch digua_bringup ekf.launch.py
ros2 launch digua_bringup lidar_x2.launch.py
ros2 launch astra_camera astra.launch.xml
ros2 launch digua_bringup rgbd_sync.launch.xml
```

检查雷达帧率：

```bash
ros2 topic hz /scan
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

检查 RGB-D 同步输出：

```bash
ros2 topic hz /camera/rgbd_image
```
