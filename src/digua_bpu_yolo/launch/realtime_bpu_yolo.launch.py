from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/color/image_raw",
            description="Input RGB image topic.",
        ),
        DeclareLaunchArgument(
            "score_threshold",
            default_value="0.25",
            description="YOLO score threshold.",
        ),
        DeclareLaunchArgument(
            "infer_fps",
            default_value="3.0",
            description="BPU inference FPS limit.",
        ),
        DeclareLaunchArgument(
            "publish_labels",
            default_value="",
            description="Comma-separated label filter.",
        ),
        DeclareLaunchArgument(
            "use_semantic_whitelist",
            default_value="true",
            description="Use semantic_mapping.yaml class_whitelist when publish_labels is empty.",
        ),
        Node(
            package="digua_bpu_yolo",
            executable="realtime_bpu_yolo_node",
            name="realtime_bpu_yolo_node",
            output="screen",
            parameters=[{
                "model_file": "/home/sunrise/digua_ws/models/bpu_yolov8s_oiv7/yolov8s-oiv7_bayese_640x640_nv12.bin",
                "classes_file": "/home/sunrise/digua_ws/src/digua_bpu_yolo/config/oiv7_classes.list",
                "image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": "/semantic/detections_json",
                "frame_id": "camera_color_optical_frame",
                "score_threshold": LaunchConfiguration("score_threshold"),
                "nms_threshold": 0.7,
                "top_k": 50,
                "cls_mode": "auto",
                "infer_fps": LaunchConfiguration("infer_fps"),
                "publish_empty": True,
                "publish_labels": LaunchConfiguration("publish_labels"),
                "use_semantic_whitelist": LaunchConfiguration("use_semantic_whitelist"),
                "semantic_whitelist_yaml": "/home/sunrise/digua_ws/src/digua_semantic_mapping/config/semantic_mapping.yaml",
                "aliases_file": "/home/sunrise/digua_ws/src/digua_bpu_yolo/config/oiv7_aliases.json",
            }],
        ),
    ])
