import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    home = os.path.expanduser('~')
    workspace_dir = os.path.join(home, 'digua_ws')
    map_root_dir = os.path.join(workspace_dir, 'digua_maps')
    nav2_dir = os.path.join(map_root_dir, 'nav2')
    os.makedirs(nav2_dir, exist_ok=True)

    current_map_name_file = os.path.join(map_root_dir, 'current_map_name.txt')

    map_name = LaunchConfiguration('map_name').perform(context).strip()
    if not map_name:
        if os.path.exists(current_map_name_file):
            with open(current_map_name_file, 'r') as f:
                map_name = f.read().strip()

    if not map_name:
        map_name = 'digua_online_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    map_topic = LaunchConfiguration('map_topic').perform(context).strip()
    output_prefix = LaunchConfiguration('output_prefix').perform(context).strip()
    transient_local = LaunchConfiguration('map_subscribe_transient_local').perform(context).strip()
    timeout = LaunchConfiguration('save_map_timeout').perform(context).strip()

    if not output_prefix:
        output_prefix = os.path.join(nav2_dir, f'{map_name}_map')

    return [
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                (
                    f'echo "[digua_mapping] saving Nav2 map:" && '
                    f'echo "  topic: {map_topic}" && '
                    f'echo "  output: {output_prefix}.yaml / {output_prefix}.pgm" && '
                    f'ros2 run nav2_map_server map_saver_cli '
                    f'-f {output_prefix} '
                    f'-t {map_topic} '
                    f'--ros-args '
                    f'-p save_map_timeout:={timeout} '
                    f'-p map_subscribe_transient_local:={transient_local}'
                )
            ],
            output='screen'
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description='Map name. Empty means read ~/digua_ws/digua_maps/current_map_name.txt'
        ),

        DeclareLaunchArgument(
            'map_topic',
            default_value='/rtabmap/map',
            description='OccupancyGrid topic to save. For this robot, default is /rtabmap/map.'
        ),

        DeclareLaunchArgument(
            'output_prefix',
            default_value='',
            description='Output prefix without .yaml/.pgm. Empty means ~/digua_ws/digua_maps/nav2/<map_name>_map'
        ),

        DeclareLaunchArgument(
            'save_map_timeout',
            default_value='20.0',
            description='Timeout for saving map.'
        ),

        DeclareLaunchArgument(
            'map_subscribe_transient_local',
            default_value='true',
            description='Use transient local QoS when subscribing map.'
        ),

        OpaqueFunction(function=launch_setup),
    ])