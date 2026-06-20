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

        DeclareLaunchArgument('enable_reverse_recovery', default_value='true'),
        DeclareLaunchArgument('reverse_recovery_distance', default_value='0.25'),
        DeclareLaunchArgument('max_reverse_recovery_count', default_value='1'),
        DeclareLaunchArgument('stop_on_nav_failure', default_value='true'),

        DeclareLaunchArgument('recent_goal_radius', default_value='0.35'),
        DeclareLaunchArgument('recent_frontier_radius', default_value='0.45'),
        DeclareLaunchArgument('recent_history_size', default_value='20'),
        DeclareLaunchArgument('min_progress_to_reset_reverse', default_value='0.55'),
        DeclareLaunchArgument('multiview_angle_step_deg', default_value='30.0'),
        DeclareLaunchArgument('multiview_lateral_offset', default_value='0.20'),
        DeclareLaunchArgument('global_fallback_enabled', default_value='true'),
        DeclareLaunchArgument('global_fallback_max_goal_distance', default_value='4.0'),
        DeclareLaunchArgument('global_fallback_max_heading_error_deg', default_value='85.0'),
        DeclareLaunchArgument('cluster_recent_radius', default_value='0.55'),
        DeclareLaunchArgument('cluster_blacklist_radius', default_value='0.75'),
        DeclareLaunchArgument('ackermann_heading_weight', default_value='0.60'),
        DeclareLaunchArgument('ackermann_final_yaw_weight', default_value='0.20'),
        DeclareLaunchArgument('cluster_size_weight', default_value='0.015'),
        DeclareLaunchArgument('global_fallback_distance_bonus', default_value='0.20'),
        DeclareLaunchArgument('staging_fallback_enabled', default_value='true'),
        DeclareLaunchArgument('staging_sample_stride_cells', default_value='4'),
        DeclareLaunchArgument('staging_clearance_radius', default_value='0.30'),
        DeclareLaunchArgument('staging_min_distance', default_value='0.50'),
        DeclareLaunchArgument('staging_max_distance', default_value='2.20'),
        DeclareLaunchArgument('staging_max_heading_error_deg', default_value='75.0'),
        DeclareLaunchArgument('staging_cluster_distance_weight', default_value='0.55'),
        DeclareLaunchArgument('staging_robot_distance_weight', default_value='0.35'),
        DeclareLaunchArgument('staging_heading_weight', default_value='0.80'),
        DeclareLaunchArgument('staging_cluster_size_weight', default_value='0.020'),

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

                'enable_reverse_recovery': ParameterValue(
                    LaunchConfiguration('enable_reverse_recovery'),
                    value_type=bool
                ),
                'reverse_recovery_distance': ParameterValue(
                    LaunchConfiguration('reverse_recovery_distance'),
                    value_type=float
                ),
                'max_reverse_recovery_count': ParameterValue(
                    LaunchConfiguration('max_reverse_recovery_count'),
                    value_type=int
                ),
                'stop_on_nav_failure': ParameterValue(
                    LaunchConfiguration('stop_on_nav_failure'),
                    value_type=bool
                ),

                'recent_goal_radius': ParameterValue(
                    LaunchConfiguration('recent_goal_radius'),
                    value_type=float
                ),
                'recent_frontier_radius': ParameterValue(
                    LaunchConfiguration('recent_frontier_radius'),
                    value_type=float
                ),
                'recent_history_size': ParameterValue(
                    LaunchConfiguration('recent_history_size'),
                    value_type=int
                ),
                'min_progress_to_reset_reverse': ParameterValue(
                    LaunchConfiguration('min_progress_to_reset_reverse'),
                    value_type=float
                ),
                'multiview_angle_step_deg': ParameterValue(
                    LaunchConfiguration('multiview_angle_step_deg'),
                    value_type=float
                ),
                'multiview_lateral_offset': ParameterValue(
                    LaunchConfiguration('multiview_lateral_offset'),
                    value_type=float
                ),
                'global_fallback_enabled': ParameterValue(
                    LaunchConfiguration('global_fallback_enabled'),
                    value_type=bool
                ),
                'global_fallback_max_goal_distance': ParameterValue(
                    LaunchConfiguration('global_fallback_max_goal_distance'),
                    value_type=float
                ),
                'global_fallback_max_heading_error_deg': ParameterValue(
                    LaunchConfiguration('global_fallback_max_heading_error_deg'),
                    value_type=float
                ),
                'cluster_recent_radius': ParameterValue(
                    LaunchConfiguration('cluster_recent_radius'),
                    value_type=float
                ),
                'cluster_blacklist_radius': ParameterValue(
                    LaunchConfiguration('cluster_blacklist_radius'),
                    value_type=float
                ),
                'ackermann_heading_weight': ParameterValue(
                    LaunchConfiguration('ackermann_heading_weight'),
                    value_type=float
                ),
                'ackermann_final_yaw_weight': ParameterValue(
                    LaunchConfiguration('ackermann_final_yaw_weight'),
                    value_type=float
                ),
                'cluster_size_weight': ParameterValue(
                    LaunchConfiguration('cluster_size_weight'),
                    value_type=float
                ),
                'global_fallback_distance_bonus': ParameterValue(
                    LaunchConfiguration('global_fallback_distance_bonus'),
                    value_type=float
                ),
                'staging_fallback_enabled': ParameterValue(
                    LaunchConfiguration('staging_fallback_enabled'),
                    value_type=bool
                ),
                'staging_sample_stride_cells': ParameterValue(
                    LaunchConfiguration('staging_sample_stride_cells'),
                    value_type=int
                ),
                'staging_clearance_radius': ParameterValue(
                    LaunchConfiguration('staging_clearance_radius'),
                    value_type=float
                ),
                'staging_min_distance': ParameterValue(
                    LaunchConfiguration('staging_min_distance'),
                    value_type=float
                ),
                'staging_max_distance': ParameterValue(
                    LaunchConfiguration('staging_max_distance'),
                    value_type=float
                ),
                'staging_max_heading_error_deg': ParameterValue(
                    LaunchConfiguration('staging_max_heading_error_deg'),
                    value_type=float
                ),
                'staging_cluster_distance_weight': ParameterValue(
                    LaunchConfiguration('staging_cluster_distance_weight'),
                    value_type=float
                ),
                'staging_robot_distance_weight': ParameterValue(
                    LaunchConfiguration('staging_robot_distance_weight'),
                    value_type=float
                ),
                'staging_heading_weight': ParameterValue(
                    LaunchConfiguration('staging_heading_weight'),
                    value_type=float
                ),
                'staging_cluster_size_weight': ParameterValue(
                    LaunchConfiguration('staging_cluster_size_weight'),
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
