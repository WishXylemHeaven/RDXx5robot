from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

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

        # Only start the lidar driver here.
        #
        # Static TF base_link -> laser_frame is now provided by:
        #   digua_description / robot_state_publisher
        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
