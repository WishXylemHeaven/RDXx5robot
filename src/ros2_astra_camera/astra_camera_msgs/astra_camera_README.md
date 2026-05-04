# Astra S 深度相机在 RDK X5 上的验证记录

本文档记录 Orbbec Astra S 深度相机在地瓜派 / RDK X5 上的基础验证流程。目标是确认相机可以稳定输出 RGB、Depth、CameraInfo、TF，并通过 `rgbd_sync` 预同步生成 `/camera/rgbd_image`，为后续 RTAB-Map 建图与导航做准备。

> 建议放置位置：`~/digua_ws/src/ros2_astra_camera/astra_camera/README.md`  
> 系统环境：Ubuntu 22.04 + ROS 2 Humble + RDK X5  
> 相机型号：Orbbec Astra S

---

## 1. 验证目标

本次相机验证需要确认以下内容：

1. RDK X5 能通过 USB 正确识别 Astra S。
2. `astra_camera` 驱动能识别相机设备。
3. RGB 图像、Depth 图像、CameraInfo 能正常输出。
4. 相机内部 TF 能正常发布。
5. 临时接入 `base_footprint -> camera_link` 后，完整 TF 链路可用。
6. 通过 `rtabmap_sync/rgbd_sync` 将 RGB、Depth、CameraInfo 预同步成 `/camera/rgbd_image`。
7. `/camera/rgbd_image` 输出稳定，可作为后续 RTAB-Map 的 RGB-D 输入。

---

## 2. 基础环境加载

每次打开新终端后，建议先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash
```

如果当前是 root 登录，可以切换到普通用户：

```bash
su - sunrise
```

进入工作空间：

```bash
cd ~/digua_ws
```

---

## 3. 编译相机相关包

如果是第一次使用，先安装依赖：

```bash
cd ~/digua_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

如果编译时报错缺少 `nlohmann/json.hpp`，安装：

```bash
sudo apt update
sudo apt install -y nlohmann-json3-dev
```

编译 Astra 相机驱动：

```bash
cd ~/digua_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select astra_camera_msgs astra_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

如果还需要编译项目启动包：

```bash
cd ~/digua_ws
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash

colcon build --symlink-install --packages-select digua_bringup
source install/setup.bash
```

---

## 4. USB 与驱动识别验证

接入 Astra S 后，检查 USB 设备：

```bash
lsusb
```

正常情况下应能看到类似：

```text
ID 2bc5:0402 Orbbec 3D Technology International, Inc ASTRA S
```

查看内核识别信息：

```bash
dmesg | grep -i usb
```

驱动层检查相机：

```bash
source ~/digua_ws/install/setup.bash
ros2 run astra_camera list_devices_node
```

正常输出类似：

```text
Found 1 devices
Device connected: Astra
URI: 2bc5/0402@1/3
Serial number: xxxxxxxxxxx
```

如果这里能看到 Astra，说明硬件连接和驱动识别已经通过。

---

## 5. 启动 Astra S 相机

推荐首次验证时关闭 IR 和点云，降低 USB 带宽和 CPU 压力：

```bash
source ~/digua_ws/install/setup.bash

ros2 launch astra_camera astra.launch.xml \
  enable_ir:=false \
  enable_point_cloud:=false \
  enable_color:=true \
  enable_depth:=true
```

说明：

- `enable_ir:=false`：关闭 IR 图像。
- `enable_point_cloud:=false`：关闭驱动侧点云输出。
- `enable_color:=true`：开启 RGB 图像。
- `enable_depth:=true`：开启深度图像。

后续 RTAB-Map 可以直接使用 RGB-D 数据，不一定需要相机驱动实时发布点云。

---

## 6. 检查 ROS 话题和节点

另开一个终端：

```bash
su - sunrise
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash
```

查看相机话题：

```bash
ros2 topic list | grep camera
```

正常应至少看到：

```text
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/camera_info
/camera/depth/image_raw
```

查看节点：

```bash
ros2 node list
```

正常应看到：

```text
/camera/camera
```

查看相机服务：

```bash
ros2 service list | grep camera
```

---

## 7. 检查 RGB 和 Depth 帧率

检查 RGB 图像频率：

```bash
ros2 topic hz /camera/color/image_raw
```

检查 Depth 图像频率：

```bash
ros2 topic hz /camera/depth/image_raw
```

本次 RDK X5 实测结果：

```text
/camera/color/image_raw 约 29 Hz
/camera/depth/image_raw 约 29 Hz
```

判断标准：

- RGB 和 Depth 都能稳定输出，说明相机数据链路正常。
- 10 Hz 以上可以进入后续功能验证。
- 25 Hz 以上效果较好。
- 接近 30 Hz 说明状态很好。

---

## 8. 检查 CameraInfo

查看 RGB CameraInfo：

```bash
ros2 topic echo /camera/color/camera_info --once
```

查看 Depth CameraInfo：

```bash
ros2 topic echo /camera/depth/camera_info --once
```

重点检查：

```text
height: 480
width: 640
frame_id: camera_color_optical_frame
frame_id: camera_depth_optical_frame
```

本次验证中，CameraInfo 能正常输出，说明 RGB 和 Depth 图像都有对应的相机内参信息。

---

## 9. TF 验证

### 9.1 查看 TF 话题

```bash
ros2 topic list | grep tf
```

正常应看到：

```text
/tf
/tf_static
```

注意：本驱动中相机内部 TF 主要发布在 `/tf`，不是只依赖 `/tf_static`。如果：

```bash
ros2 topic echo /tf_static --once
```

没有输出，不一定是错误。

### 9.2 查看 `/tf` 内容

```bash
ros2 topic echo /tf --once
```

本次验证中可看到以下链路：

```text
camera_link -> camera_color_frame
camera_color_frame -> camera_color_optical_frame
camera_link -> camera_depth_frame
camera_depth_frame -> camera_depth_optical_frame
camera_depth_frame -> camera_color_frame
```

### 9.3 查看 `/tf` 频率

```bash
ros2 topic hz /tf
```

本次验证中 `/tf` 约为：

```text
9.96 Hz
```

### 9.4 用 tf2_echo 验证相机内部 TF

验证 RGB optical frame：

```bash
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

验证 Depth optical frame：

```bash
ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
```

如果能持续输出 Translation、Rotation、RPY、Matrix，说明相机内部 TF 正常。

启动瞬间可能出现：

```text
Invalid frame ID "camera_link" passed ... frame does not exist
```

只要后面能持续输出变换，就不是问题。这通常只是 tf2_echo 刚启动时还没收到 TF 缓存。

---

## 10. 临时接入 base_footprint 到 camera_link

因为相机还没有最终安装到车体上，先用临时静态 TF 验证完整链路。

新开一个终端：

```bash
su - sunrise
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  0 0 0.25 0 0 0 \
  base_footprint camera_link
```

含义：

```text
camera_link 位于 base_footprint 上方 0.25 m
无旋转
```

注意：这个值只是临时验证用，不是最终安装参数。

另开一个终端验证完整链路：

```bash
su - sunrise
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash

ros2 run tf2_ros tf2_echo base_footprint camera_color_optical_frame
```

再验证 Depth：

```bash
ros2 run tf2_ros tf2_echo base_footprint camera_depth_optical_frame
```

如果都有输出，说明完整链路可用：

```text
base_footprint
  -> camera_link
  -> camera_color_frame / camera_depth_frame
  -> camera_color_optical_frame / camera_depth_optical_frame
```

---

## 11. 生成 TF 树图

安装工具：

```bash
sudo apt install -y ros-humble-tf2-tools graphviz python3-pydot
```

生成 TF 树：

```bash
cd ~/digua_ws
ros2 run tf2_tools view_frames
```

本次验证中 `view_frames` 打印出的 TF 结构为：

```text
camera_link:
  parent: base_footprint

camera_color_frame:
  parent: camera_link

camera_depth_frame:
  parent: camera_link

camera_color_optical_frame:
  parent: camera_color_frame

camera_depth_optical_frame:
  parent: camera_depth_frame
```

如果命令提示生成 `frames.pdf`，但当前目录找不到，可以用下面命令查找：

```bash
find ~ -name "frames.pdf" -o -name "frames.gv" -o -name "frames.yaml"
find ~/digua_ws -name "frames*"
```

即使没有找到 PDF，只要 `view_frames` 结果中出现上面的 parent 关系，也可以判断 TF 链路已经通过。

---

## 12. RGB-D 预同步验证

### 12.1 安装 rtabmap_sync

如果启动时报错：

```text
package 'rtabmap_sync' not found
```

安装：

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap-sync
```

如果找不到该包，再安装：

```bash
sudo apt install -y ros-humble-rtabmap-ros
```

验证是否安装成功：

```bash
ros2 pkg prefix rtabmap_sync
ros2 pkg executables rtabmap_sync
```

正常应能看到 `rgbd_sync` 可执行文件。

### 12.2 用命令直接启动 rgbd_sync

保持 Astra 相机驱动正在运行，新开终端：

```bash
su - sunrise
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash

ros2 run rtabmap_sync rgbd_sync --ros-args \
  -p approx_sync:=true \
  -p queue_size:=10 \
  -r rgb/image:=/camera/color/image_raw \
  -r depth/image:=/camera/depth/image_raw \
  -r rgb/camera_info:=/camera/color/camera_info \
  -r rgbd_image:=/camera/rgbd_image
```

### 12.3 用 digua_bringup 启动 rgbd_sync

如果已经创建并编译了 `digua_bringup` 包，可以直接运行：

```bash
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash

ros2 launch digua_bringup rgbd_sync.launch.xml
```

推荐 `rgbd_sync.launch.xml` 中使用以下参数：

```xml
<arg name="approx_sync" default="true"/>
<arg name="sync_queue_size" default="10"/>
<arg name="topic_queue_size" default="10"/>
<arg name="approx_sync_max_interval" default="0.05"/>
```

队列大小说明：

- 队列太小：RGB、Depth、CameraInfo 可能还没配对就被丢弃，导致 `/camera/rgbd_image` 频率低或无输出。
- 队列太大：会增加延迟和内存占用。
- 当前 Astra S 在 RDK X5 上 RGB/Depth 接近 30 Hz，建议先用 `10 / 10 / 0.05s`。

---

## 13. 检查 `/camera/rgbd_image`

查看 RGB-D 同步输出话题：

```bash
ros2 topic list | grep rgbd
```

正常应看到：

```text
/camera/rgbd_image
/rgbd_image/compressed
```

检查同步频率：

```bash
ros2 topic hz /camera/rgbd_image
```

本次 RDK X5 实测：

```text
/camera/rgbd_image 约 28 Hz
```

判断标准：

- 10 Hz 以上：可用。
- 20 Hz 以上：较好。
- 接近 30 Hz：非常好。

本次验证中 `/camera/rgbd_image` 长时间稳定在约 28 Hz，说明 RGB、Depth、CameraInfo 预同步成功。

---

## 14. 可选：录制验证 rosbag

建议保存一段成功数据，方便以后回放和排查：

```bash
mkdir -p ~/bags

ros2 bag record -o ~/bags/astra_s_x5_rgbd_ok \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/depth/image_raw \
  /camera/depth/camera_info \
  /camera/rgbd_image \
  /tf \
  /tf_static
```

录制 20 秒左右后，按 `Ctrl+C` 停止。

查看 bag 信息：

```bash
ros2 bag info ~/bags/astra_s_x5_rgbd_ok
```

---

## 15. 本次验证结论

Astra S 在 RDK X5 上验证通过。

已完成：

- USB 层识别 Astra S。
- `astra_camera` 驱动识别相机。
- RGB 图像稳定输出，约 29 Hz。
- Depth 图像稳定输出，约 29 Hz。
- Color CameraInfo 和 Depth CameraInfo 正常。
- 相机内部 TF 正常，`/tf` 约 10 Hz。
- 临时 `base_footprint -> camera_link` 接入成功。
- `digua_bringup` 可启动 `rgbd_sync`。
- `/camera/rgbd_image` 预同步输出成功，约 28 Hz。

当前 Astra S 已满足后续 RTAB-Map 第一版接入条件。

后续 RTAB-Map 预计输入：

```text
/odometry/filtered
/camera/rgbd_image
/scan
/tf
/tf_static
```

---

## 16. 常见问题

### 16.1 `rosdep update` 提示 pkg_resources deprecated

这只是 Python 警告，可以忽略，不影响编译。

### 16.2 不建议 root 下编译

如果之前 root 操作过工作空间，建议修复权限：

```bash
sudo chown -R sunrise:sunrise /home/sunrise/digua_ws
```

之后用普通用户编译，不要使用：

```bash
sudo colcon build
```

### 16.3 `/tf_static --once` 没输出

不一定是问题。本驱动相机内部 TF 主要在 `/tf` 中以约 10 Hz 发布。优先用：

```bash
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
```

### 16.4 `/camera/rgbd_image` 没有输出

先确认原始话题是否正常：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/color/camera_info --once
```

再确认 `rtabmap_sync` 是否安装：

```bash
ros2 pkg prefix rtabmap_sync
```

如果没有，安装：

```bash
sudo apt install -y ros-humble-rtabmap-sync
```

### 16.5 点云是否需要默认开启

不建议默认开启。

推荐默认：

```bash
enable_point_cloud:=false
enable_ir:=false
```

调试点云时临时开启：

```bash
ros2 launch astra_camera astra.launch.xml \
  enable_ir:=false \
  enable_point_cloud:=true \
  enable_color:=true \
  enable_depth:=true
```

---

## 17. 一键常用命令汇总

启动相机：

```bash
source ~/digua_ws/install/setup.bash

ros2 launch astra_camera astra.launch.xml \
  enable_ir:=false \
  enable_point_cloud:=false \
  enable_color:=true \
  enable_depth:=true
```

检查 RGB / Depth：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

检查 CameraInfo：

```bash
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
```

检查 TF：

```bash
ros2 topic echo /tf --once
ros2 topic hz /tf
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
```

临时发布底盘到相机 TF：

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0.25 0 0 0 \
  base_footprint camera_link
```

启动 RGB-D 预同步：

```bash
ros2 launch digua_bringup rgbd_sync.launch.xml
```

检查 RGB-D 输出：

```bash
ros2 topic list | grep rgbd
ros2 topic hz /camera/rgbd_image
```
