# digua_bpu_yolo

`digua_bpu_yolo` 是地瓜机器人项目中的 RDK X5 BPU YOLO 视觉识别包。它负责从 Astra 彩色图像中读取画面，调用 RDK X5 BPU 上的 YOLOv8s Open Images V7 模型完成目标检测，并把检测框、类别和置信度发布为语义地图模块可消费的 JSON 话题。

这个包在整车系统中的位置：

```text
Astra RGB-D 相机
        |
        | /camera/color/image_raw
        | sensor_msgs/msg/Image
        v
digua_bpu_yolo
        |
        | /semantic/detections_json
        | std_msgs/msg/String(JSON)
        v
digua_semantic_mapping
        |
        | 深度图 + TF + 检测框
        v
semantic_map.json / 语义目标导航
```

## 在整个项目里的作用

`digua_bpu_yolo` 是地瓜机器人“看见物体”的入口。它不负责深度定位，也不直接写入语义地图；它只负责把相机图像转换成稳定、可过滤、可复用的二维目标检测结果。后续的 `digua_semantic_mapping` 会继续结合深度图、相机内参和 TF，把二维检测框转换成 `map` 坐标系下的语义目标。

一句话概括：

```text
digua_bpu_yolo = RDK X5 BPU 视觉检测结果 -> 语义地图观测输入
```

## 文件树

```text
digua_bpu_yolo/
├── config/
│   ├── oiv7_aliases.json                         # Open Images 类别到语义类别的别名映射
│   ├── oiv7_classes.list                         # YOLOv8s OIV7 601 类类别表
│   └── yolov8_oiv7_workconfig.template.json      # dnn_node_example 工作配置模板
├── digua_bpu_yolo/
│   ├── __init__.py                               # Python 包标识文件
│   ├── offline_detections_publisher_node.py      # 离线检测结果 ROS 发布节点
│   ├── offline_yolov8_oiv7_postprocess.py        # 离线 BPU dump 后处理脚本
│   └── realtime_bpu_yolo_node.py                 # 实时 BPU YOLO 推理节点
├── launch/
│   ├── digua_bpu_yolo_feedback.launch.py         # 调用 dnn_node_example 做单图反馈测试
│   ├── offline_detections_pub.launch.py          # 发布离线检测 JSON
│   └── realtime_bpu_yolo.launch.py               # 启动实时 BPU YOLO 节点
├── resource/
│   └── digua_bpu_yolo                            # ament_python 包资源索引
├── test/
│   ├── test_copyright.py                         # ROS 2 包版权检查
│   ├── test_flake8.py                            # Python 代码风格检查
│   └── test_pep257.py                            # Python docstring 检查
├── package.xml                                   # ROS 2 包元信息和依赖声明
├── setup.cfg                                     # Python 脚本安装路径配置
├── setup.py                                      # ament_python 安装配置和 console_scripts 入口
└── README.md                                     # 本说明文档
```

## 文件作用说明

| 文件 | 作用 |
| --- | --- |
| `digua_bpu_yolo/realtime_bpu_yolo_node.py` | 核心实时节点。订阅相机图像，转换为 NV12，调用 RDK X5 BPU 模型，完成 YOLO 后处理和 NMS，发布检测 JSON。 |
| `digua_bpu_yolo/offline_yolov8_oiv7_postprocess.py` | 离线后处理工具。读取 BPU dump 出来的 6 个输出 tensor，解析检测框，输出 JSON 和预览图。 |
| `digua_bpu_yolo/offline_detections_publisher_node.py` | 离线发布节点。读取离线检测 JSON，按固定频率发布到 `/semantic/detections_json`，用于不跑实时 BPU 时调试语义地图。 |
| `launch/realtime_bpu_yolo.launch.py` | 实时识别启动文件，暴露 `image_topic`、`score_threshold`、`infer_fps`、`publish_labels`、`use_semantic_whitelist` 等参数。 |
| `launch/offline_detections_pub.launch.py` | 离线检测结果发布启动文件，适合配合语义地图节点做回放测试。 |
| `launch/digua_bpu_yolo_feedback.launch.py` | 动态生成 `dnn_node_example` workconfig，并调用 TROS 示例反馈 launch，用于验证模型文件、类别表和单图推理链路。 |
| `config/oiv7_classes.list` | Open Images V7 类别表，当前包含 601 个类别。 |
| `config/oiv7_aliases.json` | 类别别名映射。例如把模型原始类别映射为语义地图希望保存的 label。 |
| `config/yolov8_oiv7_workconfig.template.json` | `dnn_node_example` 的 YOLOv8 OIV7 配置模板。 |
| `package.xml` | 声明 `rclpy`、`std_msgs`、`sensor_msgs`、`cv_bridge`、`dnn_node_example` 等依赖。 |
| `setup.py` | 声明包安装内容和两个 console script：`realtime_bpu_yolo_node`、`offline_detections_publisher_node`。 |

## 节点与入口

| 可执行入口 | 节点名 | 源码 | 作用 |
| --- | --- | --- | --- |
| `realtime_bpu_yolo_node` | `realtime_bpu_yolo_node` | `realtime_bpu_yolo_node.py` | 实时订阅相机图像并调用 BPU YOLO 推理。 |
| `offline_detections_publisher_node` | `offline_detections_publisher_node` | `offline_detections_publisher_node.py` | 将离线 JSON 检测结果发布成 ROS 2 话题。 |

常用实时启动：

```bash
ros2 launch digua_bpu_yolo realtime_bpu_yolo.launch.py score_threshold:=0.30 infer_fps:=1.0
```

离线检测发布：

```bash
ros2 launch digua_bpu_yolo offline_detections_pub.launch.py
```

单图模型反馈测试：

```bash
ros2 launch digua_bpu_yolo digua_bpu_yolo_feedback.launch.py
```

## ROS 2 接口

### `realtime_bpu_yolo_node`

#### 订阅者

| 话题 | 数据类型 | 默认队列 | 作用 |
| --- | --- | ---: | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 1 | 输入相机彩色图像。节点使用 `cv_bridge` 转为 BGR，再缩放到 640x640 并转换为 NV12 输入 BPU。 |

#### 发布者

| 话题 | 数据类型 | 默认队列 | 作用 |
| --- | --- | ---: | --- |
| `/semantic/detections_json` | `std_msgs/msg/String` | 10 | 发布检测结果 JSON，供 `digua_semantic_mapping` 读取并结合深度图生成语义地图。 |

#### 发布 JSON 格式

`/semantic/detections_json` 的 `String.data` 是 JSON 字符串，结构如下：

```json
{
  "stamp": 1780000000.0,
  "frame_id": "camera_color_optical_frame",
  "image_width": 640,
  "image_height": 480,
  "source": "realtime_bpu_yolo",
  "detections": [
    {
      "label": "footwear",
      "confidence": 0.61,
      "bbox": {
        "cx": 320.0,
        "cy": 240.0,
        "w": 100.0,
        "h": 80.0
      },
      "class_id": 391
    }
  ]
}
```

其中 `bbox` 使用图像像素坐标，`cx/cy` 是检测框中心点，`w/h` 是检测框宽高。语义地图节点会用检测框中心区域读取深度，并通过 TF 转换到 `map` 坐标系。

### `offline_detections_publisher_node`

#### 订阅者

该节点不订阅 ROS 2 话题，只读取本地 JSON 文件。

#### 发布者

| 话题 | 数据类型 | 默认队列 | 默认频率 | 作用 |
| --- | --- | ---: | ---: | --- |
| `/semantic/detections_json` | `std_msgs/msg/String` | 10 | 1 Hz | 周期发布离线检测结果，用于调试语义地图链路。 |

#### 服务与 Action

该包不定义 ROS 2 service，也不定义 ROS 2 action。

## 参数

### `realtime_bpu_yolo_node`

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `model_file` | string | `/home/sunrise/digua_ws/models/bpu_yolov8s_oiv7/yolov8s-oiv7_bayese_640x640_nv12.bin` | RDK X5 BPU 模型文件。 |
| `classes_file` | string | `/home/sunrise/digua_ws/src/digua_bpu_yolo/config/oiv7_classes.list` | 类别表文件。 |
| `image_topic` | string | `/camera/color/image_raw` | 输入图像话题。 |
| `detections_topic` | string | `/semantic/detections_json` | 检测结果输出话题。 |
| `frame_id` | string | `camera_color_optical_frame` | 检测结果所在相机坐标系。 |
| `model_width` | int | `640` | 模型输入宽度。 |
| `model_height` | int | `640` | 模型输入高度。 |
| `score_threshold` | float | `0.25` | 置信度阈值。 |
| `nms_threshold` | float | `0.7` | NMS IoU 阈值。 |
| `top_k` | int | `50` | 最多发布的检测框数量。 |
| `cls_mode` | string | `auto` | 分类分数解释方式：`auto`、`sigmoid` 或 `raw`。 |
| `infer_fps` | float | `3.0` | 推理频率上限。 |
| `publish_empty` | bool | `true` | 没有检测框时是否仍发布空 detections。 |
| `publish_labels` | string | 空字符串 | 逗号分隔的 label 过滤器。非空时优先使用它。 |
| `use_semantic_whitelist` | bool | `true` | `publish_labels` 为空时，是否读取语义地图白名单。 |
| `semantic_whitelist_yaml` | string | `/home/sunrise/digua_ws/src/digua_semantic_mapping/config/semantic_mapping.yaml` | 语义地图类别白名单配置。 |
| `aliases_file` | string | `/home/sunrise/digua_ws/src/digua_bpu_yolo/config/oiv7_aliases.json` | 类别别名映射文件。 |

### `offline_detections_publisher_node`

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `detections_json` | string | `/home/sunrise/rdk_normal_yolo_test/result/digua_oiv7_offline_detections.json` | 离线后处理输出 JSON。 |
| `topic` | string | `/semantic/detections_json` | 输出话题。 |
| `frame_id` | string | `camera_color_optical_frame` | 检测结果坐标系。 |
| `model_width` | float | `640.0` | 离线模型输出坐标宽度。 |
| `model_height` | float | `640.0` | 离线模型输出坐标高度。 |
| `image_width` | float | `640.0` | 目标相机图像宽度。 |
| `image_height` | float | `480.0` | 目标相机图像高度。 |
| `publish_rate` | float | `1.0` | 发布频率。 |
| `min_confidence` | float | `0.0` | 低于该置信度的检测会被过滤。 |
| `lowercase_label` | bool | `true` | 是否将 label 转为小写。 |

## 数据流

### 实时语义识别链路

```text
/camera/color/image_raw
        |
        v
cv_bridge: ROS Image -> BGR
        |
        v
resize 640x640 + BGR -> NV12
        |
        v
hobot_dnn / RDK X5 BPU
        |
        v
YOLOv8 OIV7 后处理 + NMS + label 过滤
        |
        v
/semantic/detections_json
        |
        v
digua_semantic_mapping
```

### 离线调试链路

```text
BPU dump 输出 tensor
        |
        v
offline_yolov8_oiv7_postprocess.py
        |
        ├── digua_oiv7_offline_detections.json
        └── digua_oiv7_offline_preview.jpg
        |
        v
offline_detections_publisher_node
        |
        v
/semantic/detections_json
```

## 类别过滤与语义地图配合

实时节点支持两层类别控制：

1. `publish_labels` 非空时，直接使用命令行传入的 label 列表。
2. `publish_labels` 为空且 `use_semantic_whitelist:=true` 时，读取 `digua_semantic_mapping/config/semantic_mapping.yaml` 中 `semantic_observer_node.class_whitelist`。

`oiv7_aliases.json` 用于处理模型类别和语义类别命名不一致的问题。例如语义地图希望保存 `tv`，而 OIV7 类别表里可能是 `television`，就可以通过别名表映射。

这种设计可以减少 BPU 后处理压力：有白名单时，节点会优先把白名单类别转换为 OIV7 class id，只处理目标类别对应通道。

## 常用命令

启动实时识别：

```bash
ros2 launch digua_bpu_yolo realtime_bpu_yolo.launch.py score_threshold:=0.30 infer_fps:=1.0
```

指定输入图像话题：

```bash
ros2 launch digua_bpu_yolo realtime_bpu_yolo.launch.py image_topic:=/camera/color/image_raw
```

只发布指定类别：

```bash
ros2 launch digua_bpu_yolo realtime_bpu_yolo.launch.py publish_labels:=footwear,bottle,chair
```

查看检测结果：

```bash
ros2 topic echo /semantic/detections_json
```

离线后处理：

```bash
python3 src/digua_bpu_yolo/digua_bpu_yolo/offline_yolov8_oiv7_postprocess.py \
  --dump_dir /home/sunrise/rdk_normal_yolo_test/dump \
  --image /home/sunrise/rdk_normal_yolo_test/calib_040.jpg
```

发布离线检测结果：

```bash
ros2 launch digua_bpu_yolo offline_detections_pub.launch.py
```

验证模型反馈链路：

```bash
ros2 launch digua_bpu_yolo digua_bpu_yolo_feedback.launch.py
```
