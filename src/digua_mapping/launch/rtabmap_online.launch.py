import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    home = os.path.expanduser('~')
    workspace_dir = os.path.join(home, 'digua_ws')
    map_root_dir = os.path.join(workspace_dir, 'digua_maps')
    rtabmap_dir = os.path.join(map_root_dir, 'rtabmap')
    nav2_dir = os.path.join(map_root_dir, 'nav2')
    os.makedirs(rtabmap_dir, exist_ok=True)
    os.makedirs(nav2_dir, exist_ok=True)

    map_name = LaunchConfiguration('map_name').perform(context).strip()
    if not map_name:
        map_name = 'digua_online_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    current_map_name_file = os.path.join(map_root_dir, 'current_map_name.txt')

    database_path = LaunchConfiguration('database_path').perform(context).strip()
    if not database_path:
        database_path = os.path.join(rtabmap_dir, f'{map_name}.db')

    rtabmap_args = LaunchConfiguration('rtabmap_args').perform(context).strip()
    if not rtabmap_args:
        rtabmap_args = '--delete_db_on_start --Grid/Sensor 0 --GridGlobal/FullUpdate true --Grid/RangeMax 8.0'

    rtabmap_launch_file = PathJoinSubstitution([
        FindPackageShare('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    ])

    return [
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                (
                    f'echo {map_name} > {current_map_name_file} && '
                    f'echo "[digua_mapping] current map name: {map_name}" && '
                    f'echo "[digua_mapping] RTAB-Map database: {database_path}"'
                )
            ],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments={
                'use_sim_time': 'false',

                'visual_odometry': 'false',
                'odom_topic': '/odometry/filtered',

                'frame_id': 'base_footprint',
                'map_frame_id': 'map',

                'subscribe_rgbd': 'true',
                'rgbd_topic': '/camera/rgbd_image',

                'subscribe_scan': 'true',
                'scan_topic': '/scan',

                'approx_sync': LaunchConfiguration('approx_sync'),
                'wait_for_transform': LaunchConfiguration('wait_for_transform'),

                'rviz': LaunchConfiguration('rviz'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),

                'database_path': database_path,
                'args': rtabmap_args,
            }.items()
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description='Map name. Empty means auto timestamp name.'
        ),

        DeclareLaunchArgument(
            'database_path',
            default_value='',
            description='RTAB-Map database output path. Empty means ~/digua_ws/digua_maps/rtabmap/<map_name>.db'
        ),

        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='',
            description='Extra RTAB-Map args. Empty means default clean mapping args.'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz on this machine. Keep false on RDK X5.'
        ),

        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Start RTAB-Map GUI. Keep false on RDK X5.'
        ),

        DeclareLaunchArgument(
            'wait_for_transform',
            default_value='2.0',
            description='TF wait timeout.'
        ),

        DeclareLaunchArgument(
            'approx_sync',
            default_value='true',
            description='Use approximate sync for RGBD image, scan and odometry.'
        ),

        OpaqueFunction(function=launch_setup),
    ])