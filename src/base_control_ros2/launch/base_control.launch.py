#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    try:
        robot_type = os.environ['BASE_TYPE']
    except Exception:
        robot_type = 'NanoCar'
        print(f"\033[91mWarning: BASE_TYPE is not set. Using default: {robot_type}\033[0m")

    imu_static_transform_args = ['0', '0', '0.08', '0', '0', '0', 'base_link', 'imu']
    if robot_type in ['NanoRobot', 'NanoRobot_Pro']:
        imu_static_transform_args = ['0', '0', '0.08', '0', '0', '0', 'base_link', 'imu']
    elif robot_type in ['NanoCar', 'NanoCar_Pro']:
        imu_static_transform_args = ['0.01', '0', '0.08', '0', '0', '0', 'base_link', 'imu']
    elif robot_type == 'NanoOmni':
        imu_static_transform_args = ['-0.03', '0.04', '0.08', '0', '0', '0', 'base_link', 'imu']

    base_footprint_static_transform_args = ['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/move_base'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('imu_frame', default_value='imu'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('battery_topic', default_value='battery'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),
        DeclareLaunchArgument('pub_imu', default_value='true'),
        DeclareLaunchArgument('broadcast_odom_tf', default_value='true'),
        DeclareLaunchArgument('legacy_odom_cmd', default_value='false'),

        Node(
            package='base_control_ros2',
            executable='base_control_node',
            name='base_control',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'base_id': LaunchConfiguration('base_frame'),
                'odom_id': LaunchConfiguration('odom_frame'),
                'imu_id': LaunchConfiguration('imu_frame'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'battery_topic': LaunchConfiguration('battery_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'pub_imu': ParameterValue(LaunchConfiguration('pub_imu'), value_type=bool),
                'broadcast_odom_tf': ParameterValue(LaunchConfiguration('broadcast_odom_tf'), value_type=bool),
                'legacy_odom_cmd': ParameterValue(LaunchConfiguration('legacy_odom_cmd'), value_type=bool),
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=imu_static_transform_args,
            output='log'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_footprint',
            arguments=base_footprint_static_transform_args,
            output='log'
        ),
    ])
