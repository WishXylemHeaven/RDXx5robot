# digua_semantic_mapping

`digua_semantic_mapping` 是地瓜机器人的语义地图包。它把 `digua_bpu_yolo` 发布的目标检测框、Astra 深度图、相机内参和 TF 组合起来，计算目标在 `map` 坐标系下的位置，再把多次观测融合成语义地图 JSON。后续可以在 RViz 中显示语义目标，也可以按类别或 id 让机器人导航到某个语义目标附近。

它在系统里的位置：

```text
digua_bpu_yolo
        |
        | /semantic/detections_json
        v
semantic_observer_node
        |
        | /semantic/observations_json
        v
semantic_fusion_node
        |
        | semantic_map.json + /semantic/map_json
        v
semantic_marker_node / semantic_goto_node / semantic_map_tool
```

常用命令：

```bash
# 启动当前地图对应的语义建图，需要 RTAB-Map 在线建图已经启动并写入 current_map_name.txt
ros2 launch digua_semantic_mapping semantic_mapping_current.launch.py

# 列出当前语义地图里的目标
ros2 run digua_semantic_mapping semantic_map_tool --list

# 在 RViz/Nav2 已启动时，导航到最近的某类目标
ros2 run digua_semantic_mapping semantic_goto_node footwear --distance 0.6

# 按 id 精确导航
ros2 run digua_semantic_mapping semantic_goto_node --id 4 --distance 0.6
```

## 文件树

```text
digua_semantic_mapping/
├── config/
│   └── semantic_mapping.yaml                 # 语义观测、融合、marker 参数
├── digua_semantic_mapping/
│   ├── __init__.py                           # Python 包标记
│   ├── depth_utils.py                        # 预留深度工具文件，当前为空
│   ├── semantic_fusion_node.py               # 语义观测融合与 semantic_map.json 写入
│   ├── semantic_goto_node.py                 # 按语义目标发送 Nav2 NavigateToPose
│   ├── semantic_map_tool.py                  # 语义地图查询、清理、备份、导出工具
│   ├── semantic_marker_node.py               # 发布 RViz MarkerArray
│   ├── semantic_observer_node.py             # 检测框 + 深度 + TF -> map 坐标观测
│   ├── semantic_paths.py                     # current/legacy/显式路径解析
│   └── storage.py                            # 预留存储工具文件，当前为空
├── launch/
│   ├── semantic_mapping.launch.py            # 固定 legacy 语义地图路径的启动入口
│   └── semantic_mapping_current.launch.py    # 根据 current_map_name.txt 选择语义地图路径
├── resource/
│   └── digua_semantic_mapping                # ament_python 资源索引标记
├── test/
│   ├── test_copyright.py                     # ament 测试模板
│   ├── test_flake8.py                        # flake8 测试模板
│   └── test_pep257.py                        # pep257 测试模板
├── package.xml                               # ROS 2 包元信息和依赖
├── setup.cfg                                 # colcon 安装脚本路径配置
├── setup.py                                  # Python 包安装配置和 console_scripts 入口
└── README.md                                 # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `semantic_observer_node.py` | 订阅 YOLO 检测 JSON、深度图和相机内参，从检测框中心区域取深度，反投影到相机坐标，再通过 TF 转换到 `map` 坐标，发布语义观测 JSON。 |
| `semantic_fusion_node.py` | 订阅语义观测，按 label 和空间距离合并目标，累计观测次数，达到阈值后把目标从 `candidate` 升级为 `confirmed`，并写入语义地图文件。 |
| `semantic_marker_node.py` | 周期读取语义地图文件，发布 `/semantic/markers`，在 RViz 中显示目标球体和文字标签。 |
| `semantic_goto_node.py` | 从语义地图中按 label 或 id 选择目标，结合机器人当前位置生成一个离目标指定距离的导航点，并发送 Nav2 `NavigateToPose`。 |
| `semantic_map_tool.py` | 命令行维护工具，支持 summary、list、group、backup、清理候选目标、删除低观测目标、按 id 删除和导出成套地图。 |
| `semantic_paths.py` | 统一解析语义地图路径。支持 `current`、`auto`、空值、`legacy` 和显式路径。 |
| `semantic_mapping.yaml` | 配置观测话题、深度范围、ROI 采样比例、类别白名单、动态类别、融合距离、确认阈值和 marker 话题。 |
| `semantic_mapping.launch.py` | 启动 observer、fusion、marker，默认写入 `/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json`。 |
| `semantic_mapping_current.launch.py` | 读取 `/home/sunrise/digua_ws/digua_maps/current_map_name.txt`，将语义地图写入 `semantic_slam/<map_name>/semantic_map.json`。 |

## 节点与数据流

### 1. 检测结果进入语义观测

`semantic_observer_node` 输入：

- `/semantic/detections_json`：YOLO 检测框 JSON。
- `/camera/depth/image_raw`：深度图。
- `/camera/color/camera_info`：相机内参。
- TF：默认查询 `map <- camera_color_optical_frame`。

处理流程：

1. 从检测 JSON 中读取 `label`、`confidence`、`bbox.cx/cy/w/h`。
2. 使用 `class_whitelist` 过滤不需要进入语义地图的类别。
3. 在检测框中心按 `roi_scale` 取一块 ROI，计算有效深度中位数。
4. 使用相机内参把像素点反投影为相机坐标。
5. 通过 TF 转换到 `map` 坐标。
6. 发布 `/semantic/observations_json`。

### 2. 语义观测融合成地图

`semantic_fusion_node` 输入 `/semantic/observations_json`，输出 `/semantic/map_json`，并持久化写入 `semantic_map.json`。

融合规则：

- 动态类别会被跳过，不写入长期语义地图。
- 新目标默认状态为 `candidate`。
- 同 label 且 3D 距离小于 `merge_distance` 的观测会合并到同一对象。
- 合并时用观测次数做平均，更新 `x/y/z` 和 `confidence`。
- 观测次数达到 `min_observations_confirmed` 后，状态变为 `confirmed`。

### 3. 语义地图可视化

`semantic_marker_node` 周期读取语义地图文件并发布 `visualization_msgs/msg/MarkerArray`：

- `confirmed` 目标显示为绿色球体。
- `candidate` 目标显示为黄色球体。
- 每个目标会附带文字标签：`label#id`、状态、置信度和观测次数。

### 4. 语义目标导航

`semantic_goto_node` 从语义地图里选择一个目标，然后发 Nav2 目标：

1. 读取语义地图文件。
2. 根据 `--id` 或 label 过滤对象。
3. 默认只选择 `confirmed` 目标；加 `--allow-candidate` 可允许 candidate。
4. 获取机器人当前 `map -> base_footprint` 位置。
5. 在机器人与目标连线方向上，生成距离目标 `--distance` 的 approach pose。
6. 发送到 Nav2 `/navigate_to_pose` action。

## ROS 2 接口

### `semantic_observer_node`

| 发布/订阅 | 话题/输入 | 数据类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/semantic/detections_json` | `std_msgs/msg/String` | YOLO 检测结果 JSON。 |
| 订阅 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图，支持 `16UC1`/`mono16` 毫米深度和浮点深度。 |
| 订阅 | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参，用于像素反投影。 |
| 读取 TF | `map <- camera_color_optical_frame` | `tf2_msgs/msg/TFMessage` | 将相机坐标点转换到地图坐标。 |
| 发布 | `/semantic/observations_json` | `std_msgs/msg/String` | 语义观测 JSON。 |

### `semantic_fusion_node`

| 发布/订阅 | 话题/文件 | 数据类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/semantic/observations_json` | `std_msgs/msg/String` | 语义观测 JSON。 |
| 发布 | `/semantic/map_json` | `std_msgs/msg/String` | 完整语义地图 JSON。 |
| 读写文件 | `semantic_map_path` | JSON 文件 | 持久化保存语义地图。 |

### `semantic_marker_node`

| 发布/订阅 | 话题/文件 | 数据类型 | 说明 |
| --- | --- | --- | --- |
| 读取文件 | `semantic_map_path` | JSON 文件 | 周期读取语义地图。 |
| 发布 | `/semantic/markers` | `visualization_msgs/msg/MarkerArray` | RViz 语义目标 marker。 |

### `semantic_goto_node`

| 类型 | 名称 | 数据类型 | 说明 |
| --- | --- | --- | --- |
| 读取文件 | `semantic_map_path` | JSON 文件 | 根据 label 或 id 选择目标。 |
| 读取 TF | `map -> base_footprint` | `tf2_msgs/msg/TFMessage` | 获取机器人当前位置。 |
| Action Client | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 发送语义目标导航点。 |

`semantic_map_tool` 是纯命令行文件维护工具，不创建 ROS topic publisher/subscriber。

## JSON 数据结构

### `/semantic/detections_json`

上游来自 `digua_bpu_yolo`，`String.data` 是 JSON：

```json
{
  "stamp": 1780726290.0,
  "frame_id": "camera_color_optical_frame",
  "detections": [
    {
      "label": "footwear",
      "confidence": 0.61,
      "bbox": {
        "cx": 320.0,
        "cy": 240.0,
        "w": 80.0,
        "h": 60.0
      },
      "class_id": 391
    }
  ]
}
```

### `/semantic/observations_json`

由 `semantic_observer_node` 发布：

```json
{
  "stamp": 1780726290.1,
  "frame_id": "map",
  "source_frame": "camera_color_optical_frame",
  "observations": [
    {
      "label": "footwear",
      "confidence": 0.61,
      "depth": 1.25,
      "position_camera": {
        "x": 0.12,
        "y": -0.03,
        "z": 1.25
      },
      "position_map": {
        "x": 0.05,
        "y": -0.94,
        "z": 0.10
      },
      "status_hint": "candidate"
    }
  ]
}
```

### `semantic_map.json`

由 `semantic_fusion_node` 保存：

```json
{
  "map_frame": "map",
  "version": "0.1",
  "updated_at": 1780734554.4,
  "next_id": 8,
  "objects": [
    {
      "id": 7,
      "label": "footwear",
      "x": 0.056,
      "y": -0.938,
      "z": 0.101,
      "confidence": 0.607,
      "observations": 187,
      "status": "confirmed",
      "first_seen": 1780726290.9,
      "last_seen": 1780726372.4,
      "merge_distance": 0.5
    }
  ]
}
```

`status` 的含义：

| 状态 | 含义 |
| --- | --- |
| `candidate` | 新出现或观测次数不足的目标，不建议直接导航。 |
| `confirmed` | 观测次数达到确认阈值的目标，默认可作为语义导航目标。 |

## Launch 入口

### `semantic_mapping.launch.py`

```bash
ros2 launch digua_semantic_mapping semantic_mapping.launch.py
```

启动：

- `semantic_observer_node`
- `semantic_fusion_node`
- `semantic_marker_node`

默认语义地图路径：

```text
/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json
```

这是 legacy 路径，适合单张长期语义地图。

### `semantic_mapping_current.launch.py`

```bash
ros2 launch digua_semantic_mapping semantic_mapping_current.launch.py
```

它会读取：

```text
/home/sunrise/digua_ws/digua_maps/current_map_name.txt
```

然后将语义地图保存到：

```text
/home/sunrise/digua_ws/digua_maps/semantic_slam/<map_name>/semantic_map.json
```

这个入口适合和 `digua_mapping rtabmap_online.launch.py` 配合使用，让每一套 RTAB-Map 地图拥有独立的语义地图。

## 参数说明

### `semantic_observer_node`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map_frame` | `map` | 语义点最终保存的地图坐标系。 |
| `camera_frame` | `camera_color_optical_frame` | 相机光学坐标系。 |
| `detections_topic` | `/semantic/detections_json` | YOLO 检测结果输入。 |
| `depth_topic` | `/camera/depth/image_raw` | 深度图输入。 |
| `camera_info_topic` | `/camera/color/camera_info` | 相机内参输入。 |
| `observations_topic` | `/semantic/observations_json` | 语义观测输出。 |
| `depth_min` | `0.2` | 有效深度最小值，单位 m。 |
| `depth_max` | `4.0` | 有效深度最大值，单位 m。 |
| `roi_scale` | `0.25` | 在检测框中心取深度 ROI 的比例。 |
| `class_whitelist` | 见配置文件 | 允许进入语义地图的类别。 |
| `dynamic_classes` | 见配置文件 | 不写入长期语义地图的动态类别。 |

### `semantic_fusion_node`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `observations_topic` | `/semantic/observations_json` | 语义观测输入。 |
| `semantic_map_topic` | `/semantic/map_json` | 完整语义地图发布话题。 |
| `semantic_map_path` | `/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json` | 语义地图 JSON 保存路径。 |
| `merge_distance_default` | `0.5` | 同 label 目标的默认合并距离，单位 m。 |
| `min_observations_confirmed` | `3` | 目标变为 `confirmed` 所需观测次数。 |
| `dynamic_classes` | 见配置文件 | 融合阶段跳过的动态类别。 |

### `semantic_marker_node`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `semantic_map_path` | `/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json` | 要可视化的语义地图。 |
| `marker_topic` | `/semantic/markers` | RViz marker 话题。 |
| `map_frame` | `map` | marker 坐标系。 |
| `refresh_period` | `1.0` | marker 刷新周期，单位秒。 |

### `semantic_goto_node`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `semantic_map_path` | `current` | 默认使用当前地图对应的语义地图。也可传 `legacy` 或显式路径。 |
| `map_frame` | `map` | 导航目标坐标系。 |
| `base_frame` | `base_footprint` | 机器人底盘坐标系。 |
| `approach_distance` | `0.7` | 导航目标距离语义目标的距离，单位 m。 |
| `allow_candidate` | `false` | 是否允许导航到 candidate 目标。 |

## 语义地图路径规则

`semantic_paths.py` 支持：

| 值 | 解析结果 |
| --- | --- |
| `current`、`auto`、空值、`__current__` | `/home/sunrise/digua_ws/digua_maps/semantic_slam/<current_map_name>/semantic_map.json` |
| `legacy`、`old`、`default` | `/home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json` |
| 其他字符串 | 作为显式文件路径，支持 `~` 展开。 |

也可以通过环境变量 `DIGUA_WS` 改变默认工作空间根目录。

## 常用命令

启动语义建图：

```bash
ros2 launch digua_semantic_mapping semantic_mapping_current.launch.py
```

查看完整 JSON：

```bash
cat /home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json | python3 -m json.tool
```

列出当前语义地图：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --list
```

按类别筛选：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --list --label footwear
```

只显示 confirmed 目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --list --confirmed-only
```

按 label 分组：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --group
```

备份当前语义地图：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --backup
```

预览清理 candidate 目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --clean-candidates
```

真正清理 candidate 目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --clean-candidates --yes
```

清理观测次数太少的目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --clean-low-observations 3
```

按 id 删除目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --delete-id 3 --yes
```

列出语义目标并显示距离：

```bash
ros2 run digua_semantic_mapping semantic_goto_node --list
```

导航到最近的某类目标：

```bash
ros2 run digua_semantic_mapping semantic_goto_node footwear --distance 0.6
```

按 id 精确导航：

```bash
ros2 run digua_semantic_mapping semantic_goto_node --id 4 --distance 0.6
```

只预览语义导航目标，不发送给 Nav2：

```bash
ros2 run digua_semantic_mapping semantic_goto_node footwear --distance 0.6 --dry-run
```

## 与其他包的关系

| 相关包 | 关系 |
| --- | --- |
| `digua_bpu_yolo` | 发布 `/semantic/detections_json`，是语义地图的检测输入。 |
| `ros2_astra_camera` / `astra_camera` | 提供深度图和相机内参。 |
| `digua_description` | 提供相机、底盘、雷达等坐标关系的基础 TF。 |
| `digua_mapping` | 提供稳定的 `map` 坐标系；`semantic_mapping_current.launch.py` 依赖 `current_map_name.txt`。 |
| `digua_navigation` | 执行 `semantic_goto_node` 发出的 `/navigate_to_pose` 目标。 |

一句话概括：

```text
digua_semantic_mapping = YOLO 检测 + 深度/TF 投影 + 多次观测融合 + 语义目标导航
```

## 调试建议

- 没有语义点生成时，先检查 `/semantic/detections_json`、`/camera/depth/image_raw`、`/camera/color/camera_info` 是否都有数据。
- 如果日志提示 TF 失败，检查 `map -> camera_color_optical_frame` 是否可用。
- 如果检测有结果但被过滤，检查 `semantic_mapping.yaml` 中的 `class_whitelist`。
- 如果目标一直是 `candidate`，检查同一目标是否能被连续观测到，以及 `min_observations_confirmed` 是否过高。
- 如果语义导航找不到目标，先用 `semantic_goto_node --list` 看目标 id、状态和 label，再确认是否需要 `--allow-candidate`。
- 如果语义导航目标能生成但小车不动，优先检查 Nav2 `/navigate_to_pose` action、`map -> base_footprint` TF 和 `/cmd_vel`。
