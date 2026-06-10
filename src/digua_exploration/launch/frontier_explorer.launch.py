from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_topic', default_value='/rtabmap/map'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('robot_frame', default_value='base_footprint'),
        DeclareLaunchArgument('action_name', default_value='navigate_to_pose'),

        DeclareLaunchArgument('min_frontier_size', default_value='8'),
        DeclareLaunchArgument('min_goal_distance', default_value='0.35'),
        DeclareLaunchArgument('max_goal_distance', default_value='2.5'),
        DeclareLaunchArgument('blacklist_radius', default_value='0.45'),

        DeclareLaunchArgument('frontier_backoff', default_value='0.45'),
        DeclareLaunchArgument('goal_clearance_radius', default_value='0.25'),
        DeclareLaunchArgument('max_heading_error_deg', default_value='70.0'),

        DeclareLaunchArgument('wait_after_goal', default_value='3.0'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('max_goals', default_value='20'),

        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('once', default_value='true'),

        Node(
            package='digua_exploration',
            executable='frontier_explorer_node',
            name='frontier_explorer_node',
            output='screen',
            parameters=[{
                'map_topic': LaunchConfiguration('map_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'action_name': LaunchConfiguration('action_name'),

                'min_frontier_size': ParameterValue(
                    LaunchConfiguration('min_frontier_size'),
                    value_type=int
                ),
                'min_goal_distance': ParameterValue(
                    LaunchConfiguration('min_goal_distance'),
                    value_type=float
                ),
                'max_goal_distance': ParameterValue(
                    LaunchConfiguration('max_goal_distance'),
                    value_type=float
                ),
                'blacklist_radius': ParameterValue(
                    LaunchConfiguration('blacklist_radius'),
                    value_type=float
                ),

                'frontier_backoff': ParameterValue(
                    LaunchConfiguration('frontier_backoff'),
                    value_type=float
                ),
                'goal_clearance_radius': ParameterValue(
                    LaunchConfiguration('goal_clearance_radius'),
                    value_type=float
                ),
                'max_heading_error_deg': ParameterValue(
                    LaunchConfiguration('max_heading_error_deg'),
                    value_type=float
                ),

                'wait_after_goal': ParameterValue(
                    LaunchConfiguration('wait_after_goal'),
                    value_type=float
                ),
                'goal_timeout_sec': ParameterValue(
                    LaunchConfiguration('goal_timeout_sec'),
                    value_type=float
                ),
                'max_goals': ParameterValue(
                    LaunchConfiguration('max_goals'),
                    value_type=int
                ),

                'dry_run': ParameterValue(
                    LaunchConfiguration('dry_run'),
                    value_type=bool
                ),
                'once': ParameterValue(
                    LaunchConfiguration('once'),
                    value_type=bool
                ),
            }]
        )
    ])
