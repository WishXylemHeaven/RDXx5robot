from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


WS = "/home/sunrise/digua_ws"
CURRENT_MAP_FILE = os.path.join(WS, "digua_maps/current_map_name.txt")
SEMANTIC_SLAM_ROOT = os.path.join(WS, "digua_maps/semantic_slam")


def read_current_map_name():
    if not os.path.exists(CURRENT_MAP_FILE):
        raise RuntimeError(
            f"current_map_name.txt not found: {CURRENT_MAP_FILE}. "
            "Please start rtabmap_online.launch.py first."
        )

    with open(CURRENT_MAP_FILE, "r", encoding="utf-8") as f:
        name = f.read().strip()

    if not name:
        raise RuntimeError(
            f"current_map_name.txt is empty: {CURRENT_MAP_FILE}. "
            "Please start rtabmap_online.launch.py first."
        )

    return name


def generate_launch_description():
    pkg_share = get_package_share_directory("digua_semantic_mapping")
    config_file = os.path.join(pkg_share, "config", "semantic_mapping.yaml")

    map_name = read_current_map_name()
    semantic_dir = os.path.join(SEMANTIC_SLAM_ROOT, map_name)
    semantic_map_path = os.path.join(semantic_dir, "semantic_map.json")

    os.makedirs(semantic_dir, exist_ok=True)

    return LaunchDescription([
        LogInfo(msg=f"[semantic_mapping_current] current map name: {map_name}"),
        LogInfo(msg=f"[semantic_mapping_current] semantic_map_path: {semantic_map_path}"),

        Node(
            package="digua_semantic_mapping",
            executable="semantic_observer_node",
            name="semantic_observer_node",
            output="screen",
            parameters=[config_file],
        ),

        Node(
            package="digua_semantic_mapping",
            executable="semantic_fusion_node",
            name="semantic_fusion_node",
            output="screen",
            parameters=[
                config_file,
                {"semantic_map_path": semantic_map_path},
            ],
        ),

        Node(
            package="digua_semantic_mapping",
            executable="semantic_marker_node",
            name="semantic_marker_node",
            output="screen",
            parameters=[
                config_file,
                {"semantic_map_path": semantic_map_path},
            ],
        ),
    ])
