from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false"
        ),

        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("digua_navigation"),
                "config",
                "collision_monitor_params.yaml"
            ])
        ),

        LifecycleNode(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            namespace="",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time}
            ],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor",
            namespace="",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {"node_names": ["collision_monitor"]},
            ],
        ),
    ])
