import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('digua_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(
        pkg_nav,
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='digua_navigation',
            executable='pointcloud2_restamp.py',
            name='camera_points_restamp',
            output='screen',
            parameters=[{
                'input_topic': '/camera/depth/points',
                'output_topic': '/camera/depth/points_now',
                'frame_id': '',
                'stamp_mode': 'now',
                'publish_hz': 6.0,
                'cache_max_age': 1.5,
                'stamp_delay': 1.05,
            }],
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 navigation parameters file.'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'navigation_launch_collision.py')
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'collision_monitor.launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(pkg_nav, 'config', 'collision_monitor_params.yaml'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ),
    ])