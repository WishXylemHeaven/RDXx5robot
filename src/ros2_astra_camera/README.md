# ros2_astra_camera

`ros2_astra_camera` 是地瓜机器人工作空间里的 Astra/Orbbec RGB-D 相机驱动集合。它不是单个 ROS 2 包，而是由两个包组成：

| 子包 | 类型 | 作用 |
| --- | --- | --- |
| `astra_camera` | `ament_cmake` 驱动包 | 连接 Astra/Orbbec 深度相机，发布彩色图、深度图、相机内参、TF、可选点云，并提供相机参数读取和曝光、增益、开关等控制服务。 |
| `astra_camera_msgs` | ROS 2 接口包 | 定义驱动使用的自定义消息和服务，例如设备信息、相机内参、外参、SDK 版本、曝光/增益读写等。 |

在整车系统中，它是视觉和 RGB-D 数据入口。`digua_bringup` 会用 `rgbd_sync.launch.xml` 把它发布的彩色图、深度图和相机内参融合成 `/camera/rgbd_image`，后续供 RTAB-Map 建图/定位、YOLO 识别、语义建图和目标导航使用。

```text
Astra/Orbbec RGB-D Camera
        |
        v
astra_camera_node
        |
        +-- /camera/color/image_raw
        +-- /camera/depth/image_raw
        +-- /camera/color/camera_info
        +-- /tf or /tf_static
        |
        v
digua_bringup/rgbd_sync -> RTAB-Map / YOLO / semantic mapping
```

常用命令：

```bash
# 启动默认 Astra 相机，默认命名空间为 /camera
ros2 launch astra_camera astra.launch.xml

# 查看 Orbbec/Astra 设备
ros2 launch astra_camera list_devices.launch.xml

# 清理共享内存残留，适合相机异常退出后排查
ros2 run astra_camera clean_shm_node

# 在本项目中预融合 RGB-D，供 RTAB-Map 使用
ros2 launch digua_bringup rgbd_sync.launch.xml
```

## 文件树

```text
ros2_astra_camera/
|-- .clang-format                         # 顶层 C/C++ 格式化配置
|-- .gitattributes                        # Git 文本属性配置
|-- .gitignore                            # 忽略构建产物和临时文件
|-- README.md                             # 本说明文档
|-- astra_camera/                         # Astra/Orbbec ROS 2 相机驱动包
|   |-- .clang-format                     # 驱动包格式化配置
|   |-- .clang-tidy                       # C++ 静态检查配置
|   |-- CMakeLists.txt                    # 构建 astra_camera 库和 3 个可执行节点
|   |-- package.xml                       # ROS 2 包元信息和依赖
|   |-- config/
|   |   `-- .gitkeep                      # 预留相机配置目录
|   |-- include/
|   |   |-- astra_camera/                 # 驱动核心头文件
|   |   |   |-- constants.h               # 默认 frame、深度比例、图像尺寸等常量
|   |   |   |-- dynamic_params.h          # 动态参数辅助封装
|   |   |   |-- json.hpp                  # JSON 工具头文件
|   |   |   |-- ob_camera_node.h          # 相机节点主体声明
|   |   |   |-- ob_camera_node_factory.h  # 设备发现和节点工厂声明
|   |   |   |-- ob_context.h              # Orbbec SDK 上下文封装
|   |   |   |-- ob_timer_filter.h         # 图像帧时间过滤辅助
|   |   |   |-- ros_param_backend.h       # ROS 参数读写后端
|   |   |   |-- types.h                   # 流类型、QoS、参数类型等通用定义
|   |   |   |-- utils.h                   # 字符串、参数、SDK 转换等工具函数
|   |   |   |-- uvc_camera_driver.h       # UVC 彩色相机驱动声明
|   |   |   `-- point_cloud_proc/         # 深度点云和彩色点云处理模块头文件
|   |   |-- magic_enum/                  # 枚举字符串转换第三方头文件
|   |   `-- openni2/                     # OpenNI2 SDK 头文件
|   |-- launch/
|   |   |-- astra.launch.xml             # 本项目最常用的单相机启动文件
|   |   |-- list_devices.launch.xml      # 枚举相机设备
|   |   |-- multi_*.launch.xml           # 多相机启动模板
|   |   `-- *.launch.xml                 # 不同 Orbbec/Astra 型号的启动模板
|   |-- rviz/
|   |   |-- multi_camera.rviz            # 多相机显示配置
|   |   `-- pointcloud.rviz              # 点云显示配置
|   |-- scripts/
|   |   |-- 56-orbbec-usb.rules          # Linux USB 设备权限规则
|   |   |-- depth_to_color.py            # 深度和彩色对齐辅助脚本
|   |   |-- format_output_camera_params.py
|   |   |-- format_output_supported_video_modes.py
|   |   `-- install.sh                   # 规则和依赖安装脚本
|   `-- src/
|       |-- main.cpp                     # astra_camera_node 入口
|       |-- list_devices_node.cpp        # list_devices_node 入口
|       |-- clean_up_shm_node.cpp        # clean_shm_node 入口
|       |-- ob_camera_node.cpp           # 图像流、TF、点云、服务的核心实现
|       |-- ob_camera_node_factory.cpp   # 设备选择、连接延迟、UVC/SDK 路径选择
|       |-- ros_service.cpp              # 曝光、增益、设备信息等服务实现
|       |-- uvc_camera_driver.cpp        # UVC 彩色相机路径实现
|       |-- dynamic_params.cpp           # 动态参数实现
|       |-- ob_camera_info.cpp           # 相机内参管理
|       |-- ob_context.cpp               # SDK 上下文实现
|       |-- ob_timer_filter.cpp          # 帧时间过滤实现
|       |-- ros_param_backend.cpp        # ROS 参数后端实现
|       |-- utils.cpp                    # 工具函数实现
|       `-- point_cloud_proc/
|           |-- point_cloud_xyz.cpp      # 深度图 -> XYZ 点云
|           `-- point_cloud_xyzrgb.cpp   # 深度图 + 彩色图 + 内参 -> 彩色点云
`-- astra_camera_msgs/                   # 自定义 msg/srv 接口包
    |-- CMakeLists.txt                   # rosidl 接口生成配置
    |-- package.xml                      # 接口包元信息
    |-- msg/
    |   |-- DeviceInfo.msg               # 相机设备信息
    |   |-- Extrinsics.msg               # 深度到彩色的外参
    |   `-- Metadata.msg                 # JSON 元数据
    `-- srv/
        |-- GetCameraInfo.srv            # 读取 sensor_msgs/CameraInfo
        |-- GetCameraParams.srv          # 读取左右目内参、畸变和外参
        |-- GetDeviceInfo.srv            # 读取设备信息
        |-- GetInt32.srv                 # 读取整数参数
        |-- GetString.srv                # 读取字符串参数
        `-- SetInt32.srv                 # 写入整数参数
```

## 节点与可执行文件

| 可执行文件 | 节点/用途 | 说明 |
| --- | --- | --- |
| `astra_camera_node` | 主相机节点 | 由 `astra.launch.xml` 启动，连接设备并发布图像、内参、TF、可选点云和控制服务。 |
| `list_devices_node` | 设备枚举工具 | 输出当前可见的 Orbbec/Astra 设备，通常用于检查 USB 连接、序列号和驱动状态。 |
| `clean_shm_node` | 共享内存清理工具 | 清理 Orbbec SDK 可能残留的共享内存，适合相机节点异常退出后使用。 |

默认启动入口是：

```bash
ros2 launch astra_camera astra.launch.xml
```

`astra.launch.xml` 默认把节点放到 `camera_name` 指定的命名空间中，默认值是 `camera`，因此常见话题都是 `/camera/...`。

## ROS 2 接口

### 发布者

以下话题以 `camera_name:=camera` 的默认启动为例。`color` 和 `depth` 默认启用，`ir`、外参、点云需要对应参数打开。

| 话题 | 数据类型 | 来源/条件 | 作用 |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `enable_color:=true` | RGB 彩色图，供 YOLO、RViz、RGB-D 同步和语义建图使用。 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `enable_color:=true` | 彩色相机内参，供像素反投影、RGB-D 同步和 RTAB-Map 使用。 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `enable_depth:=true` | 深度图，编码通常为 `16UC1`，深度比例在驱动中按毫米到米处理。 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `enable_depth:=true` | 深度相机内参。 |
| `/camera/ir/image_raw` | `sensor_msgs/msg/Image` | `enable_ir:=true` | 红外图像。 |
| `/camera/ir/camera_info` | `sensor_msgs/msg/CameraInfo` | `enable_ir:=true` | 红外相机内参。 |
| `/camera/extrinsic/depth_to_color` | `astra_camera_msgs/msg/Extrinsics` | `enable_publish_extrinsic:=true` | 深度相机到彩色相机的外参。 |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | `enable_point_cloud:=true` | 由深度图生成的 XYZ 点云。 |
| `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` | `enable_colored_point_cloud:=true` | 彩色点云。驱动内部发布 `depth/color/points`，`astra.launch.xml` 重映射为该话题。 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `publish_tf:=true` 且 `tf_publish_rate>0` | 动态发布相机坐标系。 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `publish_tf:=true` 且 `tf_publish_rate<=0` | 静态发布相机坐标系。 |

常见坐标系包括 `camera_link`、`camera_depth_frame`、`camera_depth_optical_frame`、`camera_color_frame` 和 `camera_color_optical_frame`。本项目中语义建图通常会查 `map <- camera_color_optical_frame`，所以相机 TF 是否正常会直接影响目标落点。

### 订阅者

驱动节点是传感器源，正常情况下不订阅底盘、导航或建图话题。只有在启用点云处理时，包内部的点云模块会订阅驱动自己发布的图像流：

| 订阅话题 | 数据类型 | 所属模块 | 作用 |
| --- | --- | --- | --- |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `PointCloudXyzNode`、`PointCloudXyzrgbNode` | 生成 XYZ 点云或彩色点云所需的深度输入。 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `PointCloudXyzrgbNode` | 生成彩色点云所需的 RGB 输入。 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `PointCloudXyzrgbNode` | 彩色点云投影所需的相机内参。 |

真正消费这些相机输出的是项目里的其他包，例如 `digua_bringup/rgbd_sync.launch.xml` 订阅 `/camera/color/image_raw`、`/camera/depth/image_raw` 和 `/camera/color/camera_info`，再发布 `/camera/rgbd_image`。

### 服务

服务名称同样受 `camera_name` 命名空间影响，默认前缀为 `/camera/`。

| 服务 | 数据类型 | 作用 |
| --- | --- | --- |
| `/camera/get_device_info` | `astra_camera_msgs/srv/GetDeviceInfo` | 读取设备名称、VID/PID、序列号、固件版本、硬件版本等。 |
| `/camera/get_sdk_version` | `astra_camera_msgs/srv/GetString` | 读取底层 Orbbec SDK 版本。 |
| `/camera/get_camera_params` | `astra_camera_msgs/srv/GetCameraParams` | 读取左右目内参、畸变参数和左右目外参。 |
| `/camera/get_camera_info` | `astra_camera_msgs/srv/GetCameraInfo` | 读取 ROS 标准 `sensor_msgs/msg/CameraInfo`。 |
| `/camera/set_fan_mode` | `std_srvs/srv/SetBool` | 开关设备风扇模式。 |
| `/camera/set_laser_enable` | `std_srvs/srv/SetBool` | 开关激光器。 |
| `/camera/set_ldp_enable` | `std_srvs/srv/SetBool` | 开关 LDP 相关功能。 |
| `/camera/get_<stream>_exposure` | `astra_camera_msgs/srv/GetInt32` | 读取 `depth/color/ir` 某路图像流曝光值。 |
| `/camera/set_<stream>_exposure` | `astra_camera_msgs/srv/SetInt32` | 设置某路图像流曝光值。 |
| `/camera/get_<stream>_gain` | `astra_camera_msgs/srv/GetInt32` | 读取某路图像流增益。 |
| `/camera/set_<stream>_gain` | `astra_camera_msgs/srv/SetInt32` | 设置某路图像流增益。 |
| `/camera/set_<stream>_auto_exposure` | `std_srvs/srv/SetBool` | 开关某路图像流自动曝光。 |
| `/camera/toggle_<stream>` | `std_srvs/srv/SetBool` | 开关某路图像流。 |
| `/camera/get_<stream>_auto_white_balance` | `astra_camera_msgs/srv/GetInt32` | 读取自动白平衡状态。 |
| `/camera/set_<stream>_auto_white_balance` | `std_srvs/srv/SetBool` | 开关自动白平衡。 |
| `/camera/set_<stream>_mirror` | `std_srvs/srv/SetBool` | 设置图像镜像。 |
| `/camera/get_<stream>_supported_video_modes` | `astra_camera_msgs/srv/GetString` | 查询某路图像流支持的分辨率和帧率。 |

`<stream>` 一般是 `depth`、`color` 或 `ir`。如果 `use_uvc_camera:=true`，`uvc_camera_driver.cpp` 还会提供 `get_uvc_exposure`、`set_uvc_exposure`、`get_uvc_gain`、`set_uvc_gain`、`get_uvc_white_balance`、`set_uvc_white_balance`、`set_uvc_auto_exposure`、`set_uvc_auto_white_balance`、`get_uvc_mirror`、`set_uvc_mirror`、`toggle_uvc_camera` 等 UVC 彩色相机控制服务。

### 自定义消息

| 类型 | 字段摘要 | 作用 |
| --- | --- | --- |
| `astra_camera_msgs/msg/DeviceInfo` | `header`、`name`、`vid`、`pid`、`serial_number`、`firmware_version`、`supported_min_sdk_version`、`hardware_version` | 描述当前连接的相机设备。 |
| `astra_camera_msgs/msg/Extrinsics` | `header`、`rotation[9]`、`translation[3]` | 表示两个相机坐标系之间的外参。 |
| `astra_camera_msgs/msg/Metadata` | `header`、`json_data` | 以 JSON 字符串承载额外元数据。 |

### 自定义服务

| 类型 | 请求 | 响应 |
| --- | --- | --- |
| `GetDeviceInfo` | 空 | `DeviceInfo info`、`bool success`、`string message` |
| `GetCameraInfo` | 空 | `sensor_msgs/msg/CameraInfo info`、`bool success`、`string message` |
| `GetCameraParams` | 空 | 左右目内参、畸变、旋转、平移、`success`、`message` |
| `GetInt32` | 空 | `int32 data`、`bool success`、`string message` |
| `GetString` | 空 | `string data`、`bool success`、`string message` |
| `SetInt32` | `int32 data` | `bool success`、`string message` |

## 关键启动参数

`astra.launch.xml` 是本项目最常用入口，主要参数如下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `camera_name` | `camera` | ROS 命名空间，也是话题前缀。 |
| `serial_number` | 空 | 指定相机序列号，多相机时建议填写。 |
| `device_num` | `1` | 期望连接的设备数量。 |
| `vendor_id` | `0x2bc5` | Orbbec USB VID。 |
| `product_id` | 空 | 可选 USB PID 过滤。 |
| `connection_delay` | `100` | 设备连接等待时间。 |
| `depth_registration` | `true` | 是否启用深度和彩色对齐。 |
| `enable_color` | `true` | 发布彩色图。 |
| `color_width` / `color_height` / `color_fps` | `640` / `480` / `30` | 彩色图分辨率和帧率。 |
| `enable_depth` | `true` | 发布深度图。 |
| `depth_width` / `depth_height` / `depth_fps` | `640` / `480` / `30` | 深度图分辨率和帧率。 |
| `enable_ir` | `false` | 是否发布红外图。 |
| `enable_point_cloud` | `false` | 是否发布 XYZ 点云。 |
| `enable_colored_point_cloud` | `false` | 是否发布彩色点云。 |
| `publish_tf` | `true` | 是否发布相机 TF。 |
| `tf_publish_rate` | `10.0` | TF 发布频率。大于 0 时走 `/tf`。 |
| `color_depth_synchronization` | `true` | 是否启用彩色和深度同步。 |
| `enable_publish_extrinsic` | `false` | 是否发布深度到彩色外参。 |

如果只想给 RTAB-Map 和语义建图提供输入，通常保持默认 `color + depth + camera_info + tf` 即可。点云会增加计算和带宽压力，只有需要直接查看或处理点云时再打开。

## 在整个项目里的作用

`ros2_astra_camera` 位于感知链路最前端，输出的相机数据会被多个上层模块复用：

| 下游模块 | 使用的数据 | 作用 |
| --- | --- | --- |
| `digua_bringup/rgbd_sync.launch.xml` | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info` | 生成 `/camera/rgbd_image`，作为 RTAB-Map RGB-D 输入。 |
| `digua_mapping` | `/camera/rgbd_image`、相机 TF | 在线建图、视觉定位、视觉 ICP 增强定位。 |
| `digua_bpu_yolo` | `/camera/color/image_raw` | 对彩色画面做目标检测。 |
| `digua_semantic_mapping` | YOLO 检测结果、`/camera/depth/image_raw`、`/camera/color/camera_info`、TF | 将 2D 检测框反投影到 `map` 坐标系，生成语义地图。 |
| RViz 调试 | 图像、点云、TF | 检查相机画面、深度、点云和坐标系是否正常。 |

因此，相机驱动是否稳定，直接决定视觉建图、视觉定位、语义建图和目标导航是否可用。排查上层问题时，建议先确认本包输出的图像、内参和 TF 正常。

## 调试命令

```bash
# 检查彩色图和深度图帧率
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# 查看相机内参是否在发布
ros2 topic echo /camera/color/camera_info --once

# 检查相机 TF
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame

# 查看相机相关服务
ros2 service list | grep camera

# 查询设备信息
ros2 service call /camera/get_device_info astra_camera_msgs/srv/GetDeviceInfo "{}"
```

如果 `/camera/color/image_raw` 正常但 `/camera/rgbd_image` 没有输出，优先检查 `digua_bringup/rgbd_sync.launch.xml` 是否启动，以及 `/camera/depth/image_raw`、`/camera/color/camera_info` 和相机 TF 是否同步可用。
