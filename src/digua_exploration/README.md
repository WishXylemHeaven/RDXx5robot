# digua_exploration

`digua_exploration` 是地瓜机器人的自主探索包，核心目标是把正在增长的 RTAB-Map 栅格地图转换成 Nav2 可执行的探索目标点。它会从地图中寻找 frontier，也就是已知空闲区域和未知区域的边界，再结合机器人当前位姿、阿克曼底盘转向约束、目标点安全距离和历史失败记录，选择一个适合前往的 `NavigateToPose` 目标。

这个包不直接控制底盘，也不直接建图。它位于建图和导航之间：

```text
RTAB-Map /rtabmap/map
        |
        v
digua_exploration frontier_explorer_node
        |
        | nav2_msgs/action/NavigateToPose
        v
Nav2 navigate_to_pose
        |
        v
/cmd_vel -> base_control_ros2 -> 底盘
```

推荐从整车导航探索入口启动：

```bash
ros2 launch digua_navigation navigation_exploration.launch.py
```

只单独调试本包时可以直接启动：

```bash
ros2 launch digua_exploration frontier_explorer.launch.py
```

注意：`frontier_explorer.launch.py` 默认 `dry_run:=true`、`once:=true`，只会选点并打印日志，不会真的向 Nav2 发送目标。实车自动探索时需要显式关闭 dry run：

```bash
ros2 launch digua_exploration frontier_explorer.launch.py dry_run:=false once:=false max_goals:=20
```

## 文件树

```text
digua_exploration/
├── digua_exploration/
│   ├── __init__.py                    # Python 包标记文件
│   └── frontier_explorer_node.py      # frontier 探索核心节点
├── launch/
│   └── frontier_explorer.launch.py    # 自主探索节点启动文件和参数入口
├── resource/
│   └── digua_exploration              # ament_python 资源索引标记
├── package.xml                        # ROS 2 包元信息和运行依赖
├── setup.cfg                          # colcon 安装脚本路径配置
├── setup.py                           # Python 包安装配置和 console_scripts 入口
└── README.md                          # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `digua_exploration/frontier_explorer_node.py` | 核心节点。订阅栅格地图，读取 `map -> base_footprint` TF，检测 frontier，生成候选观察点，并通过 Nav2 `NavigateToPose` action 发送目标。 |
| `launch/frontier_explorer.launch.py` | 常用启动入口。声明地图话题、坐标系、目标筛选、失败恢复、阿克曼友好评分、staging fallback 等参数。 |
| `setup.py` | 声明 `frontier_explorer_node` 命令行入口，并安装 launch 文件。 |
| `setup.cfg` | 指定 `ros2 run` 安装脚本目录为 `$base/lib/digua_exploration`。 |
| `package.xml` | 声明包名、版本、许可证和运行依赖，包括 `rclpy`、`nav_msgs`、`geometry_msgs`、`nav2_msgs`、`tf2_ros`。 |
| `resource/digua_exploration` | ROS 2 ament 资源索引文件，用于让系统发现本包。 |
| `digua_exploration/__init__.py` | Python 包初始化文件。当前为空。 |

## 节点与算法

### `frontier_explorer_node`

入口：

```bash
ros2 run digua_exploration frontier_explorer_node
```

常规情况下建议通过 launch 启动，因为参数较多：

```bash
ros2 launch digua_exploration frontier_explorer.launch.py
```

节点运行流程：

1. 等待地图话题，默认是 `/rtabmap/map`。
2. 通过 TF 查询机器人在 `map` 坐标系下的位姿，默认查询 `map -> base_footprint`。
3. 在 `OccupancyGrid` 中寻找 frontier cell：当前代码把 `0..25` 视为空闲，`<0` 视为未知，`>=65` 视为障碍。
4. 将相邻 frontier cell 聚类，过滤过小 frontier。
5. 对每个 frontier cluster 生成多视角候选观察点，检查目标点 clearance、到 frontier 的连线、距离范围、转向角度和历史记录。
6. 根据 cluster 大小、行驶距离、当前朝向误差、目标最终朝向误差等因素评分，优先选择对阿克曼底盘更友好的目标。
7. 本地搜索失败时尝试全局 frontier fallback。
8. 仍然失败时尝试 staging waypoint fallback，先去一个安全的已知空闲点，再继续探索 frontier。
9. 如果没有可用目标，可按参数执行一次短距离后退恢复。
10. 目标成功后记录近期目标、frontier 和 cluster；目标失败后把目标点和 cluster 加入黑名单，避免反复撞同一个不可达位置。

## ROS 2 接口

### 发布者

本包没有显式创建普通 topic publisher。探索目标通过 Nav2 action client 发送，不通过普通 topic 发布。

### 订阅者

| 话题 | 数据类型 | 默认值 | QoS | 作用 |
| --- | --- | --- | --- | --- |
| `/rtabmap/map` | `nav_msgs/msg/OccupancyGrid` | `map_topic` | `RELIABLE`、`TRANSIENT_LOCAL`、depth 1 | 输入当前建图结果，用于寻找 frontier、判断目标点是否安全、计算已知/未知边界。 |

### Action Client

| Action 名称 | 数据类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `action_name` | 向 Nav2 发送探索目标点。目标 pose 的 `frame_id` 为 `map_frame`，默认 `map`。 |

`NavigateToPose.Goal` 中实际填入的是 `geometry_msgs/msg/PoseStamped`：

| 字段 | 来源 | 说明 |
| --- | --- | --- |
| `pose.header.frame_id` | `map_frame` | 默认 `map`。 |
| `pose.pose.position.x/y` | frontier 选点结果 | 目标点位置。 |
| `pose.pose.orientation` | 目标 yaw 转四元数 | 默认朝向 frontier 或 staging cluster。 |

代码中把 action result status `4` 视为成功，也就是 ROS 2 action 的 `STATUS_SUCCEEDED`。

### TF 输入

| 方向 | 数据类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `map_frame -> robot_frame` | `tf2_msgs/msg/TFMessage` | `map -> base_footprint` | 获取机器人当前 `x`、`y`、`yaw`，用于计算候选点距离和朝向误差。 |

TF 由 `tf2_ros.Buffer` 和 `TransformListener` 读取，底层来自 `/tf` 和 `/tf_static`。

### Service 与普通 Action Server

本包不提供 service，也不提供 action server。

## Launch 参数

`frontier_explorer.launch.py` 会覆盖一部分节点内部默认值。实际通过 launch 启动时，以本表为准。

### 基础接口

| 参数 | launch 默认值 | 说明 |
| --- | --- | --- |
| `map_topic` | `/rtabmap/map` | 输入栅格地图话题。 |
| `map_frame` | `map` | 地图坐标系，也是发送给 Nav2 的目标坐标系。 |
| `robot_frame` | `base_footprint` | 机器人底盘坐标系。 |
| `action_name` | `navigate_to_pose` | Nav2 NavigateToPose action 名称。 |
| `dry_run` | `true` | 是否只打印选点结果而不发送 Nav2 目标。调试建议保持 `true`。 |
| `once` | `true` | 是否只执行一次选点/发点流程。连续探索应设为 `false`。 |
| `max_goals` | `20` | 最多成功发送并完成的探索目标数量。 |
| `goal_timeout_sec` | `60.0` | 单个 Nav2 目标最长等待时间，超时会取消目标。 |
| `wait_after_goal` | `3.0` | 到达目标后等待地图更新的时间。 |

### Frontier 目标筛选

| 参数 | launch 默认值 | 说明 |
| --- | --- | --- |
| `min_frontier_size` | `8` | frontier cluster 的最小 cell 数量。 |
| `min_goal_distance` | `0.35` | 候选目标距离机器人太近时丢弃。 |
| `max_goal_distance` | `2.5` | 本地 frontier 候选目标最大距离。 |
| `frontier_backoff` | `0.45` | 目标点相对 frontier 向已知区域回退的距离。 |
| `goal_clearance_radius` | `0.25` | 目标点周围必须保持空闲的半径。 |
| `max_heading_error_deg` | `70.0` | 本地候选目标允许的最大朝向误差。 |
| `multiview_angle_step_deg` | `30.0` | 围绕 frontier 生成多视角候选点的角度步长。 |
| `multiview_lateral_offset` | `0.20` | 多视角候选点的横向偏移。 |

### 回退策略与历史抑制

| 参数 | launch 默认值 | 说明 |
| --- | --- | --- |
| `global_fallback_enabled` | `true` | 本地搜索失败时是否扩大搜索范围。 |
| `global_fallback_max_goal_distance` | `4.0` | 全局 fallback 候选目标最大距离。 |
| `global_fallback_max_heading_error_deg` | `85.0` | 全局 fallback 允许的最大朝向误差。 |
| `blacklist_radius` | `0.45` | 失败目标点黑名单半径。 |
| `cluster_blacklist_radius` | `0.75` | 失败 frontier cluster 黑名单半径。 |
| `recent_goal_radius` | `0.35` | 近期目标点去重半径。 |
| `recent_frontier_radius` | `0.45` | 近期 frontier 去重半径。 |
| `cluster_recent_radius` | `0.55` | 近期 cluster 去重半径。 |
| `recent_history_size` | `20` | 近期目标/frontier/cluster 记忆长度。 |
| `stop_on_nav_failure` | `true` | Nav2 目标失败后是否立即停止探索。 |

### 阿克曼友好评分

| 参数 | launch 默认值 | 说明 |
| --- | --- | --- |
| `ackermann_heading_weight` | `0.60` | 当前朝向误差在评分中的惩罚权重。 |
| `ackermann_final_yaw_weight` | `0.20` | 到达目标后的最终朝向误差惩罚权重。 |
| `cluster_size_weight` | `0.015` | frontier cluster 大小奖励权重。 |
| `global_fallback_distance_bonus` | `0.20` | 全局 fallback 中对较远候选点的补偿权重。 |
| `min_progress_to_reset_reverse` | `0.55` | 成功目标之间位移超过该值时，重置后退恢复计数。 |

### Staging fallback

| 参数 | launch 默认值 | 说明 |
| --- | --- | --- |
| `staging_fallback_enabled` | `true` | 没有直接 frontier 目标时，是否先去安全 staging 点。 |
| `staging_sample_stride_cells` | `4` | 在已知空闲区域采样 staging 点的 cell 步长。 |
| `staging_clearance_radius` | `0.30` | staging 点周围的空闲半径。 |
| `staging_min_distance` | `0.50` | staging 点距离机器人最小值。 |
| `staging_max_distance` | `2.20` | staging 点距离机器人最大值。 |
| `staging_max_heading_error_deg` | `75.0` | staging 目标允许的最大朝向误差。 |
| `staging_cluster_distance_weight` | `0.55` | staging 点到 frontier cluster 距离惩罚权重。 |
| `staging_robot_distance_weight` | `0.35` | staging 点到机器人距离惩罚权重。 |
| `staging_heading_weight` | `0.80` | staging 行驶朝向误差惩罚权重。 |
| `staging_cluster_size_weight` | `0.020` | staging 目标对 cluster 大小的奖励权重。 |

### 后退恢复

| 参数 | launch 默认值 | 说明 |
| --- | --- | --- |
| `enable_reverse_recovery` | `true` | 无可用目标时是否尝试短距离后退。 |
| `reverse_recovery_distance` | `0.25` | 后退恢复距离。 |
| `max_reverse_recovery_count` | `1` | 连续允许的后退恢复次数。 |

## 常用命令

预览选点，不发送 Nav2 目标：

```bash
ros2 launch digua_exploration frontier_explorer.launch.py
```

连续自动探索：

```bash
ros2 launch digua_exploration frontier_explorer.launch.py dry_run:=false once:=false max_goals:=20
```

只发送一个探索目标：

```bash
ros2 launch digua_exploration frontier_explorer.launch.py dry_run:=false once:=true max_goals:=1
```

检查地图是否存在：

```bash
ros2 topic echo /rtabmap/map --once
```

检查机器人位姿 TF：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

检查 Nav2 action：

```bash
ros2 action info /navigate_to_pose
```

## 在整个项目里的作用

`digua_exploration` 是在线建图阶段的自动“找路人”。当 `digua_mapping` 正在扩展地图、`digua_navigation` 已经启动 Nav2、`digua_bringup` 提供底盘与传感器数据时，本包负责不断寻找新的未知边界，并把这些边界转化为导航目标。

它和主要包的关系如下：

| 相关包 | 关系 |
| --- | --- |
| `digua_bringup` | 提供雷达、相机、底盘、EKF 和 TF，是探索运行的底层基础。 |
| `digua_mapping` | 提供 `/rtabmap/map`，探索节点依赖这张地图寻找 frontier。 |
| `digua_navigation` | 提供 Nav2 `navigate_to_pose` action，执行探索节点发出的目标。 |
| `digua_description` | 提供 `base_footprint` 等 TF 关系，保证地图坐标、机器人坐标和传感器坐标一致。 |
| `base_control_ros2` | 最终接收 Nav2 输出的 `/cmd_vel`，驱动底盘运动。 |

一句话概括：

```text
digua_exploration = 基于 RTAB-Map 栅格地图的 frontier 选点器 + Nav2 探索目标发送器
```

## 调试建议

- 第一次调参时先保持 `dry_run:=true`，观察日志中的 frontier 数量、cluster 数量、reject stats 和 selected goal。
- 如果一直提示 `cannot get robot pose`，优先检查 `map -> base_footprint` TF。
- 如果 frontier 很多但没有有效目标，重点看 `no_safe_backoff_or_clearance`、`heading_too_large`、`line_blocked` 等 reject stats。
- 如果 Nav2 目标反复失败，先检查 costmap、机器人是否在障碍区、目标点是否太靠近未知区域，以及 `goal_clearance_radius` 是否过小。
- 阿克曼底盘不适合原地大角度转向，必要时适当减小 `max_heading_error_deg`，或提高 `ackermann_heading_weight`。
