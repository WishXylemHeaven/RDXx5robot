from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("digua_semantic_mapping")
    config_file = os.path.join(pkg_share, "config", "semantic_mapping.yaml")

    semantic_observer_node = Node(
        package="digua_semantic_mapping",
        executable="semantic_observer_node",
        name="semantic_observer_node",
        output="screen",
        parameters=[config_file],
    )

    semantic_fusion_node = Node(
        package="digua_semantic_mapping",
        executable="semantic_fusion_node",
        name="semantic_fusion_node",
        output="screen",
        parameters=[config_file],
    )

    semantic_marker_node = Node(
        package="digua_semantic_mapping",
        executable="semantic_marker_node",
        name="semantic_marker_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        semantic_observer_node,
        semantic_fusion_node,
        semantic_marker_node,
    ])
