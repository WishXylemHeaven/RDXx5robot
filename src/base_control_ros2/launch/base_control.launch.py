#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('baudrate', default_value='115200'),

        # base_frame should match EKF base_link_frame.
        # In this project, EKF uses base_footprint as the robot base frame.
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('imu_frame', default_value='imu'),

        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('battery_topic', default_value='battery'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),

        DeclareLaunchArgument('pub_imu', default_value='true'),

        # Important:
        # In the full robot system, robot_localization EKF publishes:
        #   odom -> base_footprint
        # Therefore base_control should NOT also publish odom TF.
        DeclareLaunchArgument('broadcast_odom_tf', default_value='false'),

        DeclareLaunchArgument('legacy_odom_cmd', default_value='false'),

        Node(
            package='base_control_ros2',
            executable='base_control_node',
            name='base_control',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': ParameterValue(
                    LaunchConfiguration('baudrate'),
                    value_type=int
                ),
                'base_id': LaunchConfiguration('base_frame'),
                'odom_id': LaunchConfiguration('odom_frame'),
                'imu_id': LaunchConfiguration('imu_frame'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'battery_topic': LaunchConfiguration('battery_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'pub_imu': ParameterValue(
                    LaunchConfiguration('pub_imu'),
                    value_type=bool
                ),
                'broadcast_odom_tf': ParameterValue(
                    LaunchConfiguration('broadcast_odom_tf'),
                    value_type=bool
                ),
                'legacy_odom_cmd': ParameterValue(
                    LaunchConfiguration('legacy_odom_cmd'),
                    value_type=bool
                ),
            }]
        ),
    ])
