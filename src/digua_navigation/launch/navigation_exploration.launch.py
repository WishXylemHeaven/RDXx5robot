from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    nav_pkg = FindPackageShare('digua_navigation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='/home/sunrise/digua_ws/src/digua_navigation/config/nav2_exploration_params.yaml',
            description='Nav2 params for online exploration mapping, using /rtabmap/map and Ackermann no-spin BT'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    nav_pkg,
                    'launch',
                    'navigation.launch.py'
                ])
            ),
            launch_arguments={
                'params_file': params_file
            }.items()
        )
    ])
