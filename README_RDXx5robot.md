# 地瓜机器人 / RDXx5robot

> 基于 RDK X5 的室内自主移动巡检机器人项目。做一台能够在室内环境中自主建图、定位、导航、识别目标，并最终支持语义指令的移动巡检机器人。

## 项目最终目标

本项目的最终目标是实现一台具备自主移动能力和语义理解能力的室内巡检机器人：

- 能够使用激光雷达、深度相机、里程计等传感器完成室内建图、定位和导航。
- 能够在已知地图中自主规划路径、避开障碍物，并到达指定目标点。
- 能够建立语义地图，不仅知道“坐标点在哪里”，还知道“饮水机、门口、桌子、房间、充电区”等语义目标在哪里。
- 能够理解类似“去饮水机”“去门口看看”“到指定区域巡检”这样的自然语言指令，并转换为机器人可执行的导航目标。
- 能够作为巡检底盘使用，完成移动、观察、识别、记录等任务。

最终效果可以概括为：

> 用户发出语音或文本指令 → 机器人理解目标语义 → 查询语义地图 → 规划路径 → 自主导航到目标区域 → 执行观察/巡检任务。

## 当前项目进度

目前仓库已经具备一个较完整的 ROS 2 移动机器人基础框架，主要进度如下。

### 1. 硬件与底盘基础

当前项目面向的核心硬件包括：

- 主控：RDK X5
- 激光雷达：YDLIDAR X2
- 深度相机：Astra S
- 移动底盘：差速移动底盘，底盘控制通过 ROS 2 节点接入

仓库中已经包含底盘控制相关 ROS 2 包，例如 `base_control_ros2`，用于底盘串口通信、速度控制、里程计发布等基础功能。

### 2. 传感器驱动

仓库中已经包含与传感器相关的驱动和源码：

- `ydlidar_ros2_driver`：YDLIDAR 雷达 ROS 2 驱动
- `YDLidar-SDK`：YDLIDAR SDK
- `ros2_astra_camera`：Astra 深度相机 ROS 2 驱动

这些模块为后续 SLAM、定位、避障和视觉识别提供基础数据来源。

### 3. 机器人模型与可视化

仓库中已经包含 `digua_description`，用于存放机器人模型描述相关内容：

- URDF 模型
- RViz 配置
- 机器人描述启动文件

该部分用于在 ROS 2 和 RViz 中正确显示机器人模型、TF 结构以及传感器安装关系。

### 4. 建图与地图管理

仓库中已经包含 `digua_mapping` 和 `digua_maps`：

- `digua_mapping`：建图相关 ROS 2 包
- `digua_maps/nav2`：Nav2 使用的二维地图数据
- `digua_maps/rtabmap`：RTAB-Map 相关地图数据
- `digua_maps/current_map_name.txt`：当前地图名称记录

这说明项目已经开始进行地图保存、地图切换和导航地图管理。

### 5. 导航与路径规划

仓库中已经包含 `digua_navigation` 和 `digua_navigation_data`：

- `digua_navigation`：导航相关 ROS 2 包，包含配置、启动文件和脚本
- `digua_navigation_data/named_poses.yaml`：命名导航点位数据

其中 `named_poses.yaml` 是后续实现“去饮水机”“去门口”等语义导航的重要基础。当前阶段可以先把语义目标手动绑定到地图坐标，后续再结合视觉识别和语义地图自动维护。

### 6. 视觉识别与 BPU 模型

仓库中已经包含 `models/bpu_yolov8s_oiv7`，其中包括面向 RDK X5 BPU 推理的 YOLOv8s Open Images V7 模型文件和类别文件。

该部分后续可以用于：

- 室内物体识别
- 饮水机、椅子、桌子、门等目标检测
- 语义地图标注
- 巡检过程中的视觉观察与记录

### 7. 调试与验证工具

仓库中已经包含 `tools` 目录，用于硬件和方向验证，例如：

- `digua_hw_check.py`：硬件检查
- `check_base_serial_ttyS1.py`：底盘串口检查
- `check_lidar_direction.py`：雷达方向检查
- `check_imu_direction.py`：IMU 方向检查
- `check_ekf_yaw.py`：EKF 航向检查
- `capture_50_images.py`：图像采集工具

这些工具对定位问题、TF 问题、雷达方向问题、IMU 方向问题和底盘通信问题的排查很有帮助。

## 仓库目录说明

```text
RDXx5robot/
├── calib_images/                 # 相机/模型标定图片
├── digua_maps/                   # 地图数据
│   ├── nav2/                     # Nav2 二维地图
│   ├── rtabmap/                  # RTAB-Map 地图数据
│   └── current_map_name.txt      # 当前使用的地图名称
├── digua_navigation_data/         # 导航业务数据
│   └── named_poses.yaml          # 命名点位/语义点位
├── models/
│   └── bpu_yolov8s_oiv7/         # RDK X5 BPU YOLOv8s 模型
├── src/                          # ROS 2 工作空间源码
│   ├── YDLidar-SDK/              # YDLIDAR SDK
│   ├── base_control_ros2/        # 底盘控制节点
│   ├── digua_bringup/            # 总启动/系统启动配置
│   ├── digua_description/        # 机器人模型、URDF、RViz
│   ├── digua_mapping/            # 建图相关功能
│   ├── digua_navigation/         # 导航相关功能
│   ├── ros2_astra_camera/        # Astra 相机驱动
│   └── ydlidar_ros2_driver/      # YDLIDAR ROS 2 驱动
├── tools/                        # 调试、检查、采集工具
├── LICENSE                       # Apache-2.0 License
└── README.md                     # 项目说明文档
```

## 功能路线图

### 阶段一：基础可运行

- [x] 底盘控制节点接入 ROS 2
- [x] 雷达驱动接入 ROS 2
- [x] 深度相机驱动接入 ROS 2
- [x] URDF / RViz 模型基础配置
- [x] 基础硬件检查脚本
- [ ] 完成稳定的一键 bringup 启动流程
- [ ] 整理统一的安装、编译、启动文档

### 阶段二：建图、定位与导航

- [x] 建图相关目录与数据结构
- [x] Nav2 地图目录
- [x] RTAB-Map 地图目录
- [x] 导航包和命名点位文件
- [ ] 优化 AMCL / SLAM / RTAB-Map 定位稳定性
- [ ] 优化 Nav2 全局规划和局部避障参数
- [ ] 完成机器人在室内的稳定自主导航
- [ ] 支持地图保存、加载和多地图切换

### 阶段三：语义地图

- [x] 已有 `named_poses.yaml` 作为命名点位基础
- [ ] 建立语义点位管理格式，例如“饮水机 → map 坐标”
- [ ] 支持通过脚本添加、删除、查询语义点位
- [ ] 支持在 RViz 或网页端可视化语义点位
- [ ] 结合视觉识别自动标注常见物体位置

### 阶段四：语音/自然语言交互

- [ ] 接入 ASR 语音识别
- [ ] 接入 TTS 语音播报
- [ ] 支持启动时播报“地瓜启动”
- [ ] 支持“去 XXX”格式的语义导航指令
- [ ] 支持多轮交互，例如确认目标、反馈导航状态、报告异常

### 阶段五：自主巡检

- [ ] 支持巡检路线配置
- [ ] 支持定点巡检、循环巡检、异常重试
- [ ] 支持识别巡检目标并记录结果
- [ ] 支持巡检日志、图片或视频记录
- [ ] 支持低电量返回、任务中断恢复等基础机器人行为

## 推荐开发方向

当前项目下一步建议重点推进以下内容：

1. **整理 bringup 启动流程**
   将底盘、雷达、相机、TF、机器人模型、建图、导航拆分为清晰的启动文件，并提供一键启动入口。

2. **稳定 TF 树和传感器方向**
   优先确认 `base_link`、`odom`、`map`、`laser`、`camera_link` 等坐标系关系正确，避免后续导航和建图出现系统性偏差。

3. **优化 Nav2 参数**
   根据机器人尺寸、雷达安装位置、速度能力和室内通道宽度，优化 footprint、inflation radius、cost scaling factor、局部规划器速度限制等参数。

4. **完善 named_poses 语义点位**
   先从手动语义点位开始，例如：

   ```yaml
   water_dispenser:
     name: 饮水机
     frame_id: map
     x: 1.23
     y: 2.34
     yaw: 0.0
   ```

   后续再将它扩展为真正的语义地图。

5. **接入语音交互**
   先实现简单闭环：

   ```text
   语音输入：“去饮水机”
   ↓
   ASR 转文字
   ↓
   解析目标：“饮水机”
   ↓
   查询 named_poses.yaml
   ↓
   调用 Nav2 NavigateToPose
   ↓
   播报：“正在前往饮水机”
   ```

## 基础构建方式

> 具体依赖和启动命令需要结合 RDK X5 上的实际 ROS 2 版本、系统环境和硬件连接情况补充。

常见 ROS 2 工作空间构建方式如下：

```bash
cd ~/RDXx5robot
colcon build --symlink-install
source install/setup.bash
```

如果是首次部署，建议先分别验证：

```bash
# 1. 底盘串口
python3 tools/check_base_serial_ttyS1.py

# 2. 雷达方向
python3 tools/check_lidar_direction.py

# 3. IMU 方向
python3 tools/check_imu_direction.py

# 4. EKF 航向
python3 tools/check_ekf_yaw.py
```

然后再启动 bringup、建图或导航相关 launch 文件。

## 项目状态说明

本项目当前处于持续开发阶段。现阶段重点是打通移动机器人基础能力，包括硬件驱动、底盘控制、建图、定位、导航和命名点位管理。后续会逐步扩展语义地图、语音交互、视觉识别和自主巡检能力。

## License

本项目使用 Apache-2.0 License。
