# digua_navigation

`digua_navigation` 是地瓜机器人的 Nav2 导航包，负责 2D 定位、路径规划、局部避障、阿克曼底盘友好的行为树、命名点位保存与自动前往。它把 `digua_mapping` 生成的 Nav2 2D 地图、`digua_bringup` 提供的 `/scan`、`/odom`、TF 和 `base_control_ros2` 的 `/cmd_vel` 底盘控制链路连接起来。

它在整车系统中的位置：

```text
digua_maps/nav2/<map_name>_map.yaml
        |
        v
AMCL / map_server
        |
        | map -> odom
        v
Nav2 planner + controller + behavior tree
        |
        | /cmd_vel
        v
base_control_ros2 -> 底盘
```

常用命令：

```bash
# 启动 2D 定位，自动选择当前 Nav2 地图
ros2 launch digua_navigation localization.launch.py

# 启动导航
ros2 launch digua_navigation navigation.launch.py

# 建图/探索时启动导航，使用 /rtabmap/map 作为静态层
ros2 launch digua_navigation navigation_exploration.launch.py

# 自动发送初始位姿
ros2 run digua_navigation auto_initial_pose.py home

# 保存当前位置为命名点位
ros2 run digua_navigation save_named_pose.py home

# 前往命名点位
ros2 run digua_navigation go_to_named_pose.py home --timeout 180
```

## 在整个项目里的作用

`digua_navigation` 是整车自主移动能力的执行层：

- `digua_mapping` 负责建图和保存 Nav2 地图，本包负责加载地图并定位。
- `digua_bringup` 提供底盘、雷达、EKF、相机和 TF，本包利用这些输入完成路径规划和避障。
- `base_control_ros2` 接收本包最终输出的 `/cmd_vel` 并驱动底盘。
- `digua_exploration` 把 frontier 转成 `/navigate_to_pose` 目标，本包负责执行这些目标。
- `digua_semantic_mapping` 把语义目标转成地图坐标，本包负责导航到目标附近。

一句话概括：

```text
digua_navigation = AMCL 2D 定位 + Nav2 阿克曼导航 + 命名点位工具 + 巡航脚本
```

## 文件树

```text
digua_navigation/
├── behavior_trees/
│   └── navigate_to_pose_ackermann_no_spin.xml                 # 阿克曼底盘导航行为树
├── config/
│   ├── localization_params.yaml                               # AMCL + map_server 定位参数
│   ├── nav2_params.yaml                                       # 常规导航 Nav2 参数
│   ├── nav2_exploration_params.yaml                           # 在线探索导航参数，静态层接 /rtabmap/map
│   ├── nav2_mapping_params.yaml                               # 建图阶段 Nav2 参数，当前与探索配置接近
│   ├── nav2_params.yaml.bak_aggressive_20260605_170313        # 历史激进调参备份
│   └── nav2_params_before_lidar_visual_merge_20260512_220653.yaml # 合并雷达/视觉前的历史备份
├── launch/
│   ├── localization.launch.py                                 # 2D AMCL 定位入口
│   ├── navigation.launch.py                                   # Nav2 导航入口
│   └── navigation_exploration.launch.py                       # 探索/在线建图导航入口
├── scripts/
│   ├── auto_initial_pose.py                                   # 从命名点位发布 /initialpose
│   ├── follow_named_route.py                                  # 按命名点位序列巡航
│   ├── go_to_named_pose.py                                    # 前往保存的命名点位
│   ├── go_to_pose.py                                          # 前往指定 x/y/yaw
│   ├── list_named_poses.py                                    # 列出命名点位
│   ├── save_named_pose.py                                     # 从 TF 保存当前位姿
│   ├── start_nav_from_home.sh                                 # tmux 一键启动定位、初始位姿、导航
│   └── wait_for_tf.py                                         # 等待指定 TF 可用
├── CMakeLists.txt                                             # 安装 launch/config/behavior_trees/scripts
├── package.xml                                                # ROS 2 包元信息和运行依赖
└── README.md                                                  # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `launch/localization.launch.py` | 启动 `nav2_bringup/localization_launch.py`，加载 AMCL 和 map_server；默认根据 `current_map_name.txt` 自动选择 Nav2 2D 地图。 |
| `launch/navigation.launch.py` | 启动 `nav2_bringup/navigation_launch.py`，加载 planner、controller、bt_navigator、costmap、behavior server、velocity smoother 等 Nav2 节点。 |
| `launch/navigation_exploration.launch.py` | 用探索专用参数启动 `navigation.launch.py`，让 Nav2 在在线建图/探索阶段使用 `/rtabmap/map`。 |
| `config/localization_params.yaml` | AMCL 2D 定位参数，使用 `/scan`、`map/odom/base_footprint`，并让 `map_server` 加载静态地图。 |
| `config/nav2_params.yaml` | 常规导航参数。使用 Regulated Pure Pursuit 控制器、Smac Hybrid-A* 规划器、阿克曼行为树和激光雷达代价地图。 |
| `config/nav2_exploration_params.yaml` | 探索导航参数。与常规导航接近，但 costmap 的静态层显式订阅 `/rtabmap/map`。 |
| `config/nav2_mapping_params.yaml` | 建图阶段导航参数，当前主要用于在线地图场景，配置风格与探索参数一致。 |
| `behavior_trees/navigate_to_pose_ackermann_no_spin.xml` | 自定义 `NavigateToPose` 行为树。恢复动作使用清代价地图、后退、等待，不使用原地旋转，适合阿克曼底盘。 |
| `scripts/save_named_pose.py` | 从 `map -> base_footprint` TF 读取当前位姿，保存到 `named_poses.yaml`。 |
| `scripts/auto_initial_pose.py` | 从命名点位读取位姿，重复发布到 `/initialpose`，给 AMCL 自动初始化。 |
| `scripts/go_to_pose.py` | 直接发送指定 `x/y/yaw` 到 Nav2 `NavigateToPose` action。 |
| `scripts/go_to_named_pose.py` | 从 `named_poses.yaml` 读取点位名，并发送 Nav2 目标。 |
| `scripts/follow_named_route.py` | 按命名点位列表依次发送 Nav2 目标，可循环、可设置每个目标超时。 |
| `scripts/list_named_poses.py` | 打印当前点位文件中的所有命名点位。 |
| `scripts/wait_for_tf.py` | 等待指定 TF 变换出现，用于脚本编排。 |
| `scripts/start_nav_from_home.sh` | 创建 tmux 会话，自动启动定位、发送初始位姿、等待 `map -> odom`，再启动导航。 |

## 启动入口

### 2D 定位

```bash
ros2 launch digua_navigation localization.launch.py
```

这个入口启动 `map_server + amcl`：

- 优先读取 `/home/sunrise/digua_ws/digua_maps/current_map_name.txt`。
- 如果找到当前地图名，就加载 `/home/sunrise/digua_ws/digua_maps/nav2/<map_name>_map.yaml`。
- 如果当前地图不存在，就使用 `nav2/` 目录下最新的 `*_map.yaml`。
- 如果仍找不到，回退到 `/home/sunrise/digua_ws/digua_maps/nav2/map.yaml`。

它是标准的 Nav2/AMCL 2D 定位，不使用 RTAB-Map `.db`，主要依赖 `/scan` 和底盘里程计。

### 常规导航

```bash
ros2 launch digua_navigation navigation.launch.py
```

这个入口只启动 Nav2 导航栈。实际使用前通常需要先启动定位，让系统已经有稳定的 `map -> odom -> base_footprint` TF。

默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `params_file` | `config/nav2_params.yaml` | Nav2 参数文件。 |
| `use_sim_time` | `false` | 真实机器人使用系统时间。 |
| `autostart` | `true` | Nav2 lifecycle 节点自动激活。 |

### 探索导航

```bash
ros2 launch digua_navigation navigation_exploration.launch.py
```

这个入口实际 include `navigation.launch.py`，但参数文件换成：

```text
/home/sunrise/digua_ws/src/digua_navigation/config/nav2_exploration_params.yaml
```

它适合和 `digua_mapping rtabmap_online.launch.py`、`digua_exploration frontier_explorer_node` 一起使用。主要区别是 costmap 静态层使用 `/rtabmap/map`，也就是在线建图期间 RTAB-Map 正在更新的地图。

## Nav2 配置重点

`nav2_params.yaml` 和探索/建图参数的核心配置如下：

| 模块 | 关键配置 | 说明 |
| --- | --- | --- |
| `bt_navigator` | `navigate_to_pose_ackermann_no_spin.xml` | 使用阿克曼友好的行为树，恢复时不原地 spin。 |
| `controller_server` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 使用 Regulated Pure Pursuit 跟踪路径，允许低速倒车。 |
| `planner_server` | `nav2_smac_planner/SmacPlannerHybrid` | 使用 Hybrid-A*，运动模型为 `REEDS_SHEPP`，最小转弯半径 `0.3`。 |
| `local_costmap` | `ObstacleLayer + InflationLayer` | 本地滚动窗口 `3m x 3m`，使用 `/scan` 避障。 |
| `global_costmap` | `StaticLayer + ObstacleLayer + InflationLayer` | 全局代价地图叠加静态地图和雷达障碍。 |
| `velocity_smoother` | `max_velocity [0.18, 0.0, 0.45]` | 平滑速度输出，限制线速度和角速度。 |
| `waypoint_follower` | `WaitAtWaypoint` | 支持 Nav2 waypoint follower，每个点可短暂停留。 |
| `behavior_server` | `BackUp / DriveOnHeading / Wait / AssistedTeleop` | 提供恢复行为。行为树中主要使用后退和等待。 |

机器人 footprint：

```text
[[0.32, 0.10], [0.32, -0.10], [-0.04, -0.10], [-0.04, 0.10]]
```

这说明代价地图按一个前长后短、宽约 `0.20m` 的车体矩形处理机器人占用空间。

## 命名点位数据

命名点位默认保存在：

```text
~/digua_ws/digua_navigation_data/named_poses.yaml
```

文件结构：

```yaml
poses:
  home:
    frame_id: map
    base_frame: base_footprint
    x: 0.0
    y: 0.0
    yaw: 0.0
    yaw_deg: 0.0
```

这里的 `yaw` 使用弧度，`yaw_deg` 只是便于人读。

常用命令：

```bash
# 保存当前位置为 home
ros2 run digua_navigation save_named_pose.py home

# 列出所有命名点位
ros2 run digua_navigation list_named_poses.py

# 自动发送 home 初始位姿给 AMCL
ros2 run digua_navigation auto_initial_pose.py home

# 前往 home
ros2 run digua_navigation go_to_named_pose.py home --timeout 180

# 按路线巡航
ros2 run digua_navigation follow_named_route.py home Waypoint_1 home --timeout-per-goal 180
```

## ROS 2 接口

本包没有自定义 message、service 或 action 类型；它使用 Nav2、AMCL、TF 和标准消息。

### 本包脚本定义的发布者

| 脚本/节点 | 发布话题 | 数据类型 | 作用 |
| --- | --- | --- | --- |
| `auto_initial_pose.py` / `digua_auto_initial_pose` | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 向 AMCL 发布初始位姿。脚本会重复发布多次，提高接收稳定性。 |

### 本包脚本使用的订阅/TF 输入

| 脚本/节点 | 输入 | 数据类型 | 作用 |
| --- | --- | --- | --- |
| `save_named_pose.py` / `digua_save_named_pose` | `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 查询 `map -> base_footprint`，把当前机器人位姿保存为命名点位。 |
| `wait_for_tf.py` / `digua_wait_for_tf` | `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 等待指定 TF 可用，用于启动编排。 |

### 本包脚本使用的 Action Client

| 脚本/节点 | Action | 数据类型 | 作用 |
| --- | --- | --- | --- |
| `go_to_pose.py` / `digua_go_to_pose_client` | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 发送指定 `x/y/yaw` 目标。 |
| `go_to_named_pose.py` / `digua_go_to_named_pose` | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 从 `named_poses.yaml` 读取目标并发送。 |
| `follow_named_route.py` / `digua_follow_named_route` | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 按点位序列依次发送目标。 |

这些脚本填入的 action goal 是 `geometry_msgs/msg/PoseStamped`，默认坐标系为 `map`。

### Nav2/AMCL 关键输入

| 输入 | 数据类型 | 使用者 | 作用 |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | AMCL、local/global costmap | AMCL 2D 定位和代价地图避障。 |
| `/odom` | `nav_msgs/msg/Odometry` | Nav2 controller、velocity smoother | 局部运动反馈。 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Nav2 static layer、RViz | 静态 2D 地图。 |
| `/rtabmap/map` | `nav_msgs/msg/OccupancyGrid` | 探索/建图参数里的 static layer | 在线建图阶段使用的动态地图。 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | AMCL、Nav2、脚本工具 | 坐标转换。 |

### Nav2/AMCL 关键输出

| 输出 | 数据类型 | 来源 | 作用 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 controller/velocity smoother | 给 `base_control_ros2` 的底盘速度命令。 |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL | 当前 2D 定位结果。 |
| `/particle_cloud` | `geometry_msgs/msg/PoseArray` | AMCL | AMCL 粒子云调试输出。 |
| `map -> odom` | `tf2_msgs/msg/TFMessage` | AMCL 或 RTAB-Map 定位 | 全局定位修正 TF。 |
| `/plan` | `nav_msgs/msg/Path` | planner_server | 全局路径规划结果。 |
| `/local_costmap/costmap`、`/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 costmap | 本地和全局代价地图。 |

### Nav2 Action Server

| Action | 数据类型 | 使用者 |
| --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | RViz、`go_to_pose.py`、`go_to_named_pose.py`、`follow_named_route.py`、`digua_exploration`、语义导航节点。 |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 多点导航或 RViz 工具。 |

## 常用调试命令

检查 Nav2 action：

```bash
ros2 action info /navigate_to_pose
```

检查速度输出：

```bash
timeout 5 ros2 topic echo /cmd_vel
```

检查 AMCL 位姿：

```bash
ros2 topic echo /amcl_pose --once
```

检查定位 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
```

检查底盘 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

等待定位完成后再继续脚本：

```bash
ros2 run digua_navigation wait_for_tf.py map odom --timeout 60
```

## 一键从 home 启动导航

脚本：

```bash
ros2 run digua_navigation start_nav_from_home.sh --pose home
```

它会创建 tmux 会话，分窗口启动：

1. `localization.launch.py`
2. `auto_initial_pose.py <pose>`
3. `wait_for_tf.py map odom`
4. `navigation.launch.py`
5. 一个用于手动发送点位命令的 command 窗口

默认日志目录：

```text
~/digua_ws/digua_navigation_data/logs/nav_YYYYMMDD_HHMMSS/
```

## 与语义导航的关系

`digua_navigation` 负责几何导航和命名点位导航；语义目标管理和“去最近某类目标”属于 `digua_semantic_mapping`。

例如：

```bash
ros2 run digua_semantic_mapping semantic_goto_node --list
ros2 run digua_semantic_mapping semantic_goto_node footwear --distance 0.6
ros2 run digua_semantic_mapping semantic_goto_node --id 4 --distance 0.6
```

这些语义命令最终仍会转换成 Nav2 的 `/navigate_to_pose` 目标，所以它们依赖本包启动的 Nav2 导航栈。

## 使用建议

- 保存点位前先确认 `map -> odom -> base_footprint` TF 正常。
- 常规导航使用 `localization.launch.py + navigation.launch.py`。
- 在线建图和自动探索时使用 `navigation_exploration.launch.py`。
- 阿克曼底盘不适合原地旋转，行为树已经使用后退和等待替代 spin；调参时也应避免把 yaw 目标收得过紧。
- 如果导航目标能发送但小车不动，优先检查 `/cmd_vel`、Nav2 lifecycle 状态、costmap 是否被障碍占满，以及底盘控制节点是否在线。
