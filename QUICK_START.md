# 快速开始与关键数据文件

本文档整理地瓜机器人在 RDK X5 / Ubuntu / ROS 2 环境中的常用启动、检查、建图、定位、导航和语义地图命令。默认工作空间路径为：

```bash
~/digua_ws
# 常见绝对路径：/home/sunrise/digua_ws
```

建议每个新终端先执行：

```bash
cd ~/digua_ws
source install/setup.bash
```

## 构建工作空间

首次部署或更新源码后构建：

```bash
cd ~/digua_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

如果依赖安装失败，优先确认板端 ROS 2/TROS、Nav2、RTAB-Map、robot_localization、Astra 相机驱动、YDLIDAR 依赖和 RDK X5 BPU 运行库是否已经安装。

## 底层启动与硬件检查

### 底层一键启动

```bash
ros2 launch digua_bringup bringup_real.launch.py
```

该命令会按 `digua_bringup` 的默认参数启动整车基础系统，包括机器人描述、底盘、EKF、雷达、相机和 RGB-D 同步等模块。

### 自检脚本

```bash
python3 ~/digua_ws/tools/digua_hw_check.py
```

### 底层分模块启动

整车坐标：

```bash
ros2 launch digua_description display.launch.py rviz:=false
```

底盘：

```bash
ros2 launch base_control_ros2 base_control.launch.py
```

EKF：

```bash
ros2 launch digua_bringup ekf.launch.py
```

雷达：

```bash
ros2 launch digua_bringup lidar_x2.launch.py
```

相机：

```bash
ros2 launch astra_camera astra.launch.xml
```

相机画面预融合：

```bash
ros2 launch digua_bringup rgbd_sync.launch.xml
```

### 基础运行检查

检查雷达帧率：

```bash
ros2 topic hz /scan
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

检查速度指令：

```bash
timeout 5 ros2 topic echo /cmd_vel
```

虚拟机启动 RViz 操控小车：

```bash
ros2 launch nav2_bringup rviz_launch.py
```

## 在线建图、自动探索与语义建图

### 启动在线建图

```bash
ros2 launch digua_mapping rtabmap_online.launch.py
```

`rtabmap_online.launch.py` 会更新 `current_map_name.txt`，后续保存地图和语义建图会用它拼出当前地图路径。

### 启动自动探索导航

```bash
ros2 launch digua_navigation navigation_exploration.launch.py
```

### 启动语义建图

需要在 RTAB-Map 在线建图运行中启动：

```bash
ros2 launch digua_semantic_mapping semantic_mapping_current.launch.py
```

### 启动 YOLO 识别

```bash
ros2 launch digua_bpu_yolo realtime_bpu_yolo.launch.py score_threshold:=0.30 infer_fps:=1.0
```

### 建图时检查闭环

```bash
ros2 topic echo /rtabmap/info | grep -E "loop_closure_id|proximity_detection_id|ref_id"
```

### 保存地图

保存 2D 地图。需要在在线建图运行中执行，地图会保存到工作空间的 `digua_maps/nav2/` 目录：

```bash
ros2 launch digua_mapping save_nav2_map.launch.py
```

保存整套地图。该脚本主要用于备份：

```bash
/home/sunrise/digua_ws/tools/export_current_map_set.sh
```

查看当前成套地图：

```bash
/home/sunrise/digua_ws/tools/show_current_map_set.sh
```

## 定位与导航

启动 2D 定位：

```bash
ros2 launch digua_navigation localization.launch.py
```

启动视觉定位：

```bash
ros2 launch digua_mapping rtabmap_localization.launch.py
```

启动增强版视觉定位：

```bash
ros2 launch digua_mapping rtabmap_localization_visicp.launch.py
```

启动雷达不影响位置的视觉 3DoF 版本：

```bash
ros2 launch digua_mapping rtabmap_localization_visual_3dof.launch.py
```

自动发送初始位姿：

```bash
ros2 run digua_navigation auto_initial_pose.py home
```

启动导航：

```bash
ros2 launch digua_navigation navigation.launch.py
```

## 语义地图管理

检查白名单映射：

```bash
/home/sunrise/digua_ws/tools/check_alias.sh
```

检测地图上现有的语义节点：

```bash
cat /home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json | python3 -m json.tool
```

删除语义节点文件。这个命令会清空当前语义地图文件，建议先备份：

```bash
rm -f /home/sunrise/digua_ws/digua_maps/semantic/semantic_map.json
```

列出所有语义目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --list
```

按类别筛选：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --list --label footwear
```

显示 confirmed 目标，也就是可导航目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --list --confirmed-only
```

按 label 分组显示：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --group
```

备份当前语义地图。默认备份到 `/home/sunrise/digua_ws/digua_maps/semantic/backups/`：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --backup
```

预览清理 candidate 目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --clean-candidates
```

真正删除 candidate 目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --clean-candidates --yes
```

清理观测次数太少的目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --clean-low-observations 3
```

按 ID 删除目标：

```bash
ros2 run digua_semantic_mapping semantic_map_tool --delete-id 3
```

## 命名地点与语义目标导航

保存地点名称：

```bash
ros2 run digua_navigation save_named_pose.py 名字
```

例如保存 `home`：

```bash
ros2 run digua_navigation save_named_pose.py home
```

查看命名点列表：

```bash
ros2 run digua_navigation list_named_poses.py
```

自动前往命名点：

```bash
ros2 run digua_navigation go_to_named_pose.py home --timeout 180
```

`--timeout` 可加可不加。

自动发送初始位姿：

```bash
ros2 run digua_navigation auto_initial_pose.py home
```

列出当前地图里的语义目标：

```bash
ros2 run digua_semantic_mapping semantic_goto_node --list
```

导航到最近的某类目标：

```bash
ros2 run digua_semantic_mapping semantic_goto_node footwear --distance 0.6
```

按 ID 精确导航：

```bash
ros2 run digua_semantic_mapping semantic_goto_node --id 4 --distance 0.6
```

## 关键数据文件

| 路径 | 说明 |
| --- | --- |
| `digua_navigation_data/named_poses.yaml` | 命名导航点位数据，例如 `home`、`Waypoint_1` 等。 |
| `digua_maps/current_map_name.txt` | 当前地图名。建图、保存地图、定位和语义建图会根据它定位当前地图数据。 |
| `digua_maps/nav2/` | Nav2 使用的 2D 栅格地图目录，通常保存 `.yaml` 和 `.pgm`。 |
| `digua_maps/rtabmap/` | RTAB-Map 数据库目录，通常保存 `.db`。 |
| `digua_maps/semantic/semantic_map.json` | 当前语义地图文件，保存目标类别、坐标、置信度、观测次数和状态。 |
| `digua_maps/semantic/backups/` | 语义地图备份目录。 |
| `src/digua_semantic_mapping/config/semantic_mapping.yaml` | 语义观测、类别白名单、动态类别、融合距离和 marker 发布配置。 |
| `src/digua_bpu_yolo/config/oiv7_aliases.json` | OIV7 类别别名映射，用于把模型类别映射到语义白名单。 |
| `src/digua_bpu_yolo/config/oiv7_classes.list` | YOLO Open Images V7 类别列表。 |
| `models/bpu_yolov8s_oiv7/yolov8s-oiv7_bayese_640x640_nv12.bin` | RDK X5 BPU YOLOv8s OIV7 模型文件。 |
| `src/digua_navigation/behavior_trees/navigate_to_pose_ackermann_no_spin.xml` | 适配阿克曼底盘的 Nav2 行为树。 |
| `render_feedback_0_0.jpeg` | 一次相机/视觉回传样例图。 |

## 常见顺序参考

基础导航常见顺序：

```text
bringup_real
-> localization
-> auto_initial_pose
-> navigation
-> go_to_named_pose 或 semantic_goto_node
```

建图与语义建图常见顺序：

```text
bringup_real
-> rtabmap_online
-> navigation_exploration
-> realtime_bpu_yolo
-> semantic_mapping_current
-> save_nav2_map
```
