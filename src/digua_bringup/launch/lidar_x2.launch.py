from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 默认使用 ydlidar_ros2_driver 包内的 X2.yaml
    # 当前 X2.yaml 已经在前面验证过：
    # port: /dev/ttyUSB0
    # fixed_resolution: false
    # range_max: 8.0
    # frequency: 6.0 / 实测约 8Hz
    params_file = LaunchConfiguration("params_file")

    # 临时雷达 TF 参数
    # 现在雷达还没装车，所以先假设雷达在 base_link 正上方 2cm
    # 后面装车后，根据真实安装位置修改这里的默认值
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")

    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_roll = LaunchConfiguration("lidar_roll")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ydlidar_ros2_driver"),
                "params",
                "X2.yaml"
            ]),
            description="YDLIDAR X2 parameter file"
        ),

        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Robot base frame. Temporary value before chassis integration."
        ),

        DeclareLaunchArgument(
            "laser_frame",
            default_value="laser_frame",
            description="Laser frame published by YDLIDAR X2 driver."
        ),

        DeclareLaunchArgument(
            "lidar_x",
            default_value="0.0",
            description="Temporary lidar x offset from base_link, meters."
        ),

        DeclareLaunchArgument(
            "lidar_y",
            default_value="0.0",
            description="Temporary lidar y offset from base_link, meters."
        ),

        DeclareLaunchArgument(
            "lidar_z",
            default_value="0.02",
            description="Temporary lidar z offset from base_link, meters."
        ),

        DeclareLaunchArgument(
            "lidar_roll",
            default_value="0.0",
            description="Temporary lidar roll, radians."
        ),

        DeclareLaunchArgument(
            "lidar_pitch",
            default_value="0.0",
            description="Temporary lidar pitch, radians."
        ),

        DeclareLaunchArgument(
            "lidar_yaw",
            default_value="0.0",
            description="Temporary lidar yaw, radians."
        ),

        # YDLIDAR X2 驱动节点
        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            emulate_tty=True,
            parameters=[params_file],
        ),

        # 临时静态 TF：base_link -> laser_frame
        # 后面装车后，需要把 x/y/z/yaw 改成雷达真实安装位置
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_pub_laser",
            output="screen",
            arguments=[
                "--x", lidar_x,
                "--y", lidar_y,
                "--z", lidar_z,
                "--roll", lidar_roll,
                "--pitch", lidar_pitch,
                "--yaw", lidar_yaw,
                "--frame-id", base_frame,
                "--child-frame-id", laser_frame,
            ],
        ),
    ])