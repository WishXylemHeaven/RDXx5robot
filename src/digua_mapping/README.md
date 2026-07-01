# digua_mapping

`digua_mapping` 是地瓜机器人的 RTAB-Map 建图与视觉定位启动包。它本身不实现新的 C++/Python 节点，而是把 `rtabmap_launch`、`rtabmap_ros` 和 `nav2_map_server` 按本车的话题、TF、地图路径约定组织起来，用于在线建图、RTAB-Map 数据库定位和 Nav2 2D 地图保存。

它在系统里的位置：

```text
Astra RGB-D + YDLIDAR X2 + EKF odom
        |
        v
digua_mapping / RTAB-Map
        |
        | rtabmap .db / OccupancyGrid / map->odom
        v
digua_navigation / digua_exploration / digua_semantic_mapping
```

常用命令：

```bash
# 在线建图
ros2 launch digua_mapping rtabmap_online.launch.py

# 保存 Nav2 2D 地图，需要在线建图正在运行
ros2 launch digua_mapping save_nav2_map.launch.py

# RTAB-Map 视觉定位
ros2 launch digua_mapping rtabmap_localization.launch.py

# RTAB-Map 视觉 + ICP 增强定位
ros2 launch digua_mapping rtabmap_localization_visicp.launch.py

# RTAB-Map 视觉 3DoF 定位，scan 不直接拉动定位
ros2 launch digua_mapping rtabmap_localization_visual_3dof.launch.py
```

## 文件树

```text
digua_mapping/
├── launch/
│   ├── rtabmap_online.launch.py                    # 在线建图，生成/更新 RTAB-Map 数据库
│   ├── rtabmap_localization.launch.py              # RTAB-Map 普通视觉定位
│   ├── rtabmap_localization_visicp.launch.py       # RTAB-Map 视觉 + ICP 增强定位
│   ├── rtabmap_localization_visual_3dof.launch.py  # RTAB-Map 视觉 3DoF 定位，避免 scan 拉动位姿
│   └── save_nav2_map.launch.py                     # 调用 Nav2 map_saver_cli 保存 2D 栅格地图
├── CMakeLists.txt                                  # 安装 launch 目录
├── package.xml                                     # ROS 2 包元信息和运行依赖
└── README.md                                       # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `launch/rtabmap_online.launch.py` | 在线建图入口。创建 `~/digua_ws/digua_maps/rtabmap` 和 `~/digua_ws/digua_maps/nav2`，写入 `current_map_name.txt`，启动 `rtabmap_launch/rtabmap.launch.py`。 |
| `launch/rtabmap_localization.launch.py` | 普通 RTAB-Map 定位。读取已有 `.db`，关闭增量建图，用 RGB-D 与已有地图匹配定位，并输出 `/map` 给 Nav2/RViz。 |
| `launch/rtabmap_localization_visicp.launch.py` | 视觉 + ICP 增强定位。使用视觉匹配作为基础，再用 `/scan` 做 ICP 几何细化，适合纹理和几何都比较可靠的环境。 |
| `launch/rtabmap_localization_visual_3dof.launch.py` | 视觉 3DoF 定位。只让视觉配准影响定位，`/scan` 仍可用于 2D 栅格输出，但不通过 ICP 直接修正位姿。 |
| `launch/save_nav2_map.launch.py` | 调用 `nav2_map_server map_saver_cli`，把 `/rtabmap/map` 保存为 Nav2 使用的 `.yaml` 和 `.pgm`。 |
| `CMakeLists.txt` | `ament_cmake` 构建文件，只负责安装 `launch/`。 |
| `package.xml` | 声明 `rtabmap_launch`、`rtabmap_ros`、`rtabmap_msgs`、`nav2_map_server` 等运行依赖。 |

## 地图文件约定

本包默认围绕 `~/digua_ws/digua_maps` 管理地图：

```text
~/digua_ws/digua_maps/
├── current_map_name.txt             # 当前地图名
├── rtabmap/
│   └── <map_name>.db                # RTAB-Map 数据库，保存视觉词袋、图优化、传感器数据等
└── nav2/
    ├── <map_name>_map.yaml          # Nav2/AMCL 使用的 2D 地图描述
    └── <map_name>_map.pgm           # Nav2/AMCL 使用的 2D 栅格图像
```

在线建图时：

- `map_name` 为空时自动生成 `digua_online_YYYYMMDD_HHMMSS`。
- `database_path` 为空时保存到 `~/digua_ws/digua_maps/rtabmap/<map_name>.db`。
- `current_map_name.txt` 会被更新为当前地图名。

定位时：

1. 优先读取 `/home/sunrise/digua_ws/digua_maps/current_map_name.txt`。
2. 如果当前地图名能对应到 `rtabmap/<name>.db`，就使用这份数据库。
3. 如果找不到当前数据库，就自动选 `rtabmap/` 目录下最新的 `.db`。
4. 如果仍找不到，就回退到代码里的兜底数据库名。

保存 Nav2 地图时：

- 默认读取 `current_map_name.txt`。
- 输出到 `~/digua_ws/digua_maps/nav2/<map_name>_map.yaml` 和 `<map_name>_map.pgm`。

## 建图与定位方式

这个项目里容易混淆的是“2D 定位”和“三种 RTAB-Map 定位”。它们不是同一类东西。

| 方式 | 启动命令 | 使用地图 | 主要传感器 | 是否使用 RTAB-Map `.db` | 是否使用 Nav2 2D `.yaml/.pgm` | 适合场景 |
| --- | --- | --- | --- | --- | --- | --- |
| 在线建图 | `ros2 launch digua_mapping rtabmap_online.launch.py` | 新建或覆盖 RTAB-Map 数据库 | RGB-D、`/scan`、`/odometry/filtered` | 会生成 | 可后续保存 | 第一次扫图、更新地图、语义建图前建立环境模型。 |
| Nav2/AMCL 2D 定位 | `ros2 launch digua_navigation localization.launch.py` | Nav2 2D 地图 | `/scan`、底盘里程计 | 不使用 | 使用 | 只需要二维导航定位，速度快、依赖少，适合常规导航。 |
| RTAB-Map 普通视觉定位 | `ros2 launch digua_mapping rtabmap_localization.launch.py` | RTAB-Map 数据库 | RGB-D、`/scan`、`/odometry/filtered` | 使用 | 不直接依赖 | 需要恢复 RTAB-Map 的 `map -> odom`，或需要给语义地图/视觉定位提供 3D 地图上下文。 |
| RTAB-Map 视觉 + ICP 增强定位 | `ros2 launch digua_mapping rtabmap_localization_visicp.launch.py` | RTAB-Map 数据库 | RGB-D、`/scan`、`/odometry/filtered` | 使用 | 不直接依赖 | 视觉特征和雷达几何都可靠时，用 ICP 进一步约束定位。 |
| RTAB-Map 视觉 3DoF 定位 | `ros2 launch digua_mapping rtabmap_localization_visual_3dof.launch.py` | RTAB-Map 数据库 | RGB-D、`/scan`、`/odometry/filtered` | 使用 | 不直接依赖 | 雷达容易误拉位置、相似墙面/走廊导致 scan 约束不稳定时，让定位主要由视觉决定。 |
| 保存 Nav2 2D 地图 | `ros2 launch digua_mapping save_nav2_map.launch.py` | RTAB-Map 输出的 OccupancyGrid | `/rtabmap/map` | 读取运行中的地图输出 | 生成 | 在线建图完成后，把地图导出给 AMCL/Nav2 使用。 |

### 在线建图

启动文件：`rtabmap_online.launch.py`

在线建图会启动 `rtabmap_launch/rtabmap.launch.py`，关键配置是：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `visual_odometry` | `false` | 不启用 RTAB-Map 自带视觉里程计，使用外部 EKF 里程计。 |
| `odom_topic` | `/odometry/filtered` | 来自 `robot_localization` 的融合里程计。 |
| `frame_id` | `base_footprint` | 机器人底盘坐标系。 |
| `map_frame_id` | `map` | 地图坐标系。 |
| `subscribe_rgbd` | `true` | 使用 RGB-D 同步图像。 |
| `rgbd_topic` | `/camera/rgbd_image` | 由 `digua_bringup/rgbd_sync.launch.xml` 生成。 |
| `subscribe_scan` | `true` | 使用 2D 激光雷达。 |
| `scan_topic` | `/scan` | YDLIDAR X2 雷达话题。 |
| `approx_sync` | `true` | 默认使用近似时间同步。 |
| `args` | `--delete_db_on_start --Grid/Sensor 0 --GridGlobal/FullUpdate true --Grid/RangeMax 8.0` | 每次以干净数据库开始建图，使用激光生成 2D 栅格并全量更新。 |

在线建图适合“从头扫一张地图”。如果你要在已有 `.db` 上继续积累，不能直接使用默认参数，因为默认 `--delete_db_on_start` 会清空数据库。

### Nav2/AMCL 2D 定位

启动文件不在本包，而在 `digua_navigation`：

```bash
ros2 launch digua_navigation localization.launch.py
```

它使用 `nav2_bringup/localization_launch.py`，加载 `digua_navigation/config/localization_params.yaml`，核心是 `map_server + amcl`：

- 地图来自 `~/digua_ws/digua_maps/nav2/<map_name>_map.yaml`。
- 主要传感器是 `/scan`。
- 输出标准 Nav2 所需的 `map -> odom`。
- 不读取 RTAB-Map `.db`，也不需要 RGB-D。

这一路最适合普通 2D 导航：启动快、依赖少、行为稳定。但它不知道视觉特征、3D 点云和 RTAB-Map 图优化信息。

### RTAB-Map 普通视觉定位

启动文件：`rtabmap_localization.launch.py`

关键参数：

| 参数 | 值 | 作用 |
| --- | --- | --- |
| `localization` | `true` | RTAB-Map 进入定位模式。 |
| `Mem/IncrementalMemory` | `false` | 不新增地图节点，不继续建图。 |
| `Mem/InitWMWithAllNodes` | `true` | 启动时加载已有地图节点用于匹配。 |
| `RGBD/LocalizationSmoothing` | `true` | 平滑定位输出。 |
| `RGBD/MaxOdomCacheSize` | `0` | 关闭 odom cache，避免异常里程计缓存影响验证。 |
| `Vis/MinInliers` | `25` | 提高视觉匹配质量门槛。 |
| `Grid/Sensor` | `0` | 使用 `/scan` 生成/维护 2D 栅格输出。 |

这一路适合大多数视觉定位调试。它依赖 RGB-D 图像、已有 `.db` 和 `/odometry/filtered`，定位结果来自 RTAB-Map 对当前视觉观测和历史地图节点的匹配。

### RTAB-Map 视觉 + ICP 增强定位

启动文件：`rtabmap_localization_visicp.launch.py`

和普通视觉定位相比，它额外打开：

| 参数 | 值 | 作用 |
| --- | --- | --- |
| `Reg/Strategy` | `2` | 使用 Visual + ICP 配准。 |
| `Reg/Force3DoF` | `true` | 约束为平面机器人位姿，只估计 `x/y/yaw`。 |
| `RGBD/ProximityPathMaxNeighbors` | `0` | 关闭部分 proximity 近邻影响，减少相似场景牵拉。 |
| `Icp/MaxCorrespondenceDistance` | `0.15` | 限制 ICP 点匹配距离。 |
| `Icp/CorrespondenceRatio` | `0.45` | 要求足够比例的激光点能匹配。 |
| `Icp/MaxTranslation` | `0.06` | 限制单次 ICP 平移修正幅度。 |
| `Icp/MaxRotation` | `0.12` | 限制单次 ICP 旋转修正幅度。 |

这一路适合环境几何明显、雷达和视觉外参可靠的场景。好处是定位可以被激光几何进一步拉准；风险是如果 `/scan` 与地图局部形状误匹配，位置可能被 ICP 拉偏。

### RTAB-Map 视觉 3DoF 定位

启动文件：`rtabmap_localization_visual_3dof.launch.py`

和增强版相比，它故意让雷达不直接影响定位修正：

| 参数 | 值 | 作用 |
| --- | --- | --- |
| `Reg/Strategy` | `0` | 只使用视觉配准，不使用 ICP/scan 几何配准修正位姿。 |
| `Reg/Force3DoF` | `true` | 定位仍约束在平面 `x/y/yaw`。 |
| `RGBD/ProximityPathMaxNeighbors` | `0` | 关闭基于 scan proximity 的近邻闭环，降低相似场景误拉风险。 |
| `subscribe_scan` | `true` | 仍订阅 `/scan`，用于 2D 栅格输出等用途。 |

这一路适合“雷达不想影响位置”的定位调试。比如狭长走廊、重复墙面、雷达局部几何相似，ICP 可能把机器人拉到错误位置；这时用视觉 3DoF 能把定位主导权交给 RGB-D 视觉匹配，同时保持平面机器人约束。

## ROS 2 接口

`digua_mapping` 没有自定义节点，因此没有自己写出的 publisher/subscriber 类。下面列的是本包 launch 后组织起来的关键 ROS 2 接口。

### 输入订阅

| 话题/输入 | 数据类型 | 使用者 | 作用 |
| --- | --- | --- | --- |
| `/camera/rgbd_image` | `rtabmap_msgs/msg/RGBDImage` | RTAB-Map 建图/定位 | RGB 图、深度图和相机内参的同步输入。 |
| `/scan` | `sensor_msgs/msg/LaserScan` | RTAB-Map 建图/定位、AMCL 对照 | 生成 2D 栅格，视觉 + ICP 模式下参与几何配准。 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | RTAB-Map 建图/定位 | EKF 融合里程计，为 RTAB-Map 提供连续运动先验。 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | RTAB-Map、Nav2 | 提供 `map/odom/base_footprint/camera/laser` 等坐标转换。 |
| `/rtabmap/map` | `nav_msgs/msg/OccupancyGrid` | `save_nav2_map.launch.py` | 保存 Nav2 `.yaml/.pgm` 时读取的 2D 栅格地图。 |

### 输出发布

| 话题/输出 | 数据类型 | 来源 | 作用 |
| --- | --- | --- | --- |
| `/rtabmap/map` | `nav_msgs/msg/OccupancyGrid` | RTAB-Map 在线建图 | RTAB-Map 输出的 2D 栅格地图，探索和保存地图常用。 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | RTAB-Map 定位模式重映射 | 定位启动文件把 RTAB-Map 地图输出重映射到 Nav2/RViz 常用的标准 `/map`。 |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | RTAB-Map 定位模式重映射 | 地图增量更新输出。 |
| `/rtabmap/info` | `rtabmap_msgs/msg/Info` | RTAB-Map | 闭环、近邻检测、定位匹配等调试信息。 |
| `map -> odom` TF | `tf2_msgs/msg/TFMessage` | RTAB-Map 或 AMCL | 定位系统对全局位姿的修正。 |
| `<map_name>_map.yaml/.pgm` | 文件 | `nav2_map_server map_saver_cli` | Nav2/AMCL 使用的静态 2D 地图文件。 |

### Service 与 Action

本包不定义自有 service 或 action。`save_nav2_map.launch.py` 通过 `ExecuteProcess` 调用：

```bash
ros2 run nav2_map_server map_saver_cli
```

## Launch 参数

### `rtabmap_online.launch.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map_name` | 空 | 为空时自动生成时间戳地图名。 |
| `database_path` | 空 | 为空时使用 `~/digua_ws/digua_maps/rtabmap/<map_name>.db`。 |
| `rtabmap_args` | 空 | 为空时使用默认清库建图参数。 |
| `rviz` | `false` | 是否启动 RViz。RDK X5 上建议保持 `false`。 |
| `rtabmap_viz` | `false` | 是否启动 RTAB-Map GUI。RDK X5 上建议保持 `false`。 |
| `wait_for_transform` | `2.0` | 等待 TF 的超时时间。 |
| `approx_sync` | `true` | 是否使用近似时间同步。 |

### 三个 RTAB-Map 定位 launch

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `database_path` | 自动选择 | 默认根据 `current_map_name.txt`、最新 `.db`、兜底 `.db` 依次选择。 |
| `odom_topic` | `/odometry/filtered` | 定位使用的外部里程计。 |
| `rviz` | `false` | 是否启动 RViz。 |
| `rtabmap_viz` | `false` | 是否启动 RTAB-Map GUI。 |

### `save_nav2_map.launch.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map_name` | 空 | 为空时读取 `current_map_name.txt`，再为空则生成时间戳名。 |
| `map_topic` | `/rtabmap/map` | 要保存的 `OccupancyGrid` 话题。 |
| `output_prefix` | 空 | 为空时输出到 `~/digua_ws/digua_maps/nav2/<map_name>_map`。 |
| `save_map_timeout` | `20.0` | 保存地图超时时间。 |
| `map_subscribe_transient_local` | `true` | 保存地图时使用 transient local QoS 订阅地图。 |

## 常用检查命令

检查在线建图地图输出：

```bash
ros2 topic echo /rtabmap/map --once
```

检查闭环和近邻检测：

```bash
ros2 topic echo /rtabmap/info | grep -E "loop_closure_id|proximity_detection_id|ref_id"
```

检查 RGB-D 同步输入：

```bash
ros2 topic hz /camera/rgbd_image
```

检查雷达输入：

```bash
ros2 topic hz /scan
```

检查定位 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
```

查看当前地图名：

```bash
cat ~/digua_ws/digua_maps/current_map_name.txt
```

## 在整个项目里的作用

`digua_mapping` 是几何地图和视觉定位的中枢：

- `digua_bringup` 先启动底盘、EKF、雷达、相机和 RGB-D 同步，为本包提供 `/scan`、`/camera/rgbd_image`、`/odometry/filtered` 和 TF。
- `digua_mapping` 在线建图生成 RTAB-Map `.db`，并输出 `/rtabmap/map`。
- `save_nav2_map.launch.py` 把 `/rtabmap/map` 保存成 Nav2/AMCL 所需的 `.yaml/.pgm`。
- `digua_navigation` 可以使用 AMCL 2D 定位，也可以在 RTAB-Map 定位提供 `map -> odom` 时执行导航。
- `digua_exploration` 依赖在线建图期间的 `/rtabmap/map` 自动寻找 frontier。
- `digua_semantic_mapping` 依赖稳定的 `map` 坐标系，把视觉识别结果落到语义地图中。

一句话概括：

```text
digua_mapping = RTAB-Map 在线建图 + RTAB-Map 视觉定位 + Nav2 2D 地图导出
```

## 使用建议

- 新环境先运行 `rtabmap_online.launch.py` 建图，再运行 `save_nav2_map.launch.py` 导出 Nav2 地图。
- 只做普通二维导航时，优先使用 `digua_navigation localization.launch.py` 的 AMCL 2D 定位。
- 需要语义地图、视觉重定位或 RTAB-Map 图优化上下文时，使用本包的 RTAB-Map 定位。
- 如果视觉定位正常但雷达 ICP 会把位置拉偏，使用 `rtabmap_localization_visual_3dof.launch.py`。
- 如果环境几何稳定、外参准确，并且希望雷达进一步约束位置，使用 `rtabmap_localization_visicp.launch.py`。
