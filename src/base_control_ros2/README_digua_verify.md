# base_control_ros2 地瓜派机器人验证流程

这个包用于 RDK X5 / Ubuntu 22.04 / ROS 2 Humble 通过串口控制 Nano 控制器底盘，默认使用 `/dev/move_base`，发布 `/odom`、`/imu`、`/battery`，订阅 `/cmd_vel`。

## 1. 安装依赖

```bash
sudo apt update
sudo apt install -y python3-serial
```

## 2. 安装 udev 串口别名

```bash
cd ~/digua_ws/src/base_control_ros2/script
bash move_base_udev.sh
# 拔插下位机 USB/串口线后检查
ls -l /dev/move_base
dmesg | grep -E "ttyUSB|ttyACM" | tail -20
```

如果 `/dev/move_base` 不存在，先看真实设备：

```bash
lsusb
dmesg | grep -E "ttyUSB|ttyACM" | tail -20
```

## 3. 直接测试串口协议

```bash
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/move_base', 115200, timeout=1)
s.write(bytes.fromhex('5a 06 01 f1 00 d7'))  # 查询版本号
r = s.read(32)
print('RX:', r.hex(' '))
s.close()
PY
```

期望至少能看到以 `5a` 开头，并包含 `f2` 功能码的回复。

## 4. 编译

```bash
cd ~/digua_ws
colcon build --packages-select base_control_ros2 --symlink-install
source install/setup.bash
```

## 5. 启动

```bash
export BASE_TYPE=NanoCar
ros2 launch base_control_ros2 base_control.launch.py port:=/dev/move_base pub_imu:=true broadcast_odom_tf:=true
```

如果后面使用 `robot_localization` 的 EKF 发布 `odom -> base_footprint`，启动底盘驱动时改为：

```bash
ros2 launch base_control_ros2 base_control.launch.py port:=/dev/move_base pub_imu:=true broadcast_odom_tf:=false
```

## 6. 查看是否正常发布

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/digua_ws/install/setup.bash
ros2 node list
ros2 topic list | grep -E 'cmd_vel|odom|imu|battery|tf'
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic echo /imu --once
ros2 topic echo /battery --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 7. 安全运动测试

先把车轮架空，低速测试。

```bash
# 前进 2 秒，速度 0.05 m/s
timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

阿克曼底盘先使用 `/cmd_vel`，不要优先使用 `ackermann_cmd`。当前下位机源码中 `0x15` 阿克曼专用速度指令解析函数没有真正调用电机计算函数，而 `/cmd_vel` 对应的 `0x01` 指令会进入底盘运动学计算。
