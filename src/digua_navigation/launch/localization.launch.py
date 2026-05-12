import glob
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration




def get_default_map_path():
    current_name_file = '/home/sunrise/digua_ws/digua_maps/current_map_name.txt'
    nav2_map_dir = '/home/sunrise/digua_ws/digua_maps/nav2'

    # 优先读取 current_map_name.txt
    try:
        with open(current_name_file, 'r') as f:
            name = f.read().strip()

        if name:
            map_path = os.path.join(nav2_map_dir, name + '_map.yaml')
            if os.path.exists(map_path):
                return map_path

            print('[digua_navigation] WARNING: current_map_name.txt points to missing map:', map_path)

    except Exception as e:
        print('[digua_navigation] WARNING: failed to read current_map_name.txt:', e)

    # 如果 current_map_name.txt 不可用，则自动找 nav2 目录下最新的 2D 地图
    maps = sorted(
        glob.glob(os.path.join(nav2_map_dir, '*_map.yaml')),
        key=os.path.getmtime,
        reverse=True
    )

    if maps:
        print('[digua_navigation] WARNING: using latest map:', maps[0])
        return maps[0]

    # 最后兜底
    fallback = os.path.join(nav2_map_dir, 'map.yaml')
    print('[digua_navigation] WARNING: no map found, fallback to:', fallback)
    return fallback

def generate_launch_description():
    pkg_nav = get_package_share_directory('digua_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(
        pkg_nav,
        'config',
        'localization_params.yaml'
    )

    default_map = get_default_map_path()

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the Nav2 map yaml file.'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 localization parameters file.'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ),
    ])