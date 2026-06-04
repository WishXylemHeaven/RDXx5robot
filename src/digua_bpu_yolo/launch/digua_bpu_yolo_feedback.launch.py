import json
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory("digua_bpu_yolo"))

    # package share:
    # /home/sunrise/digua_ws/install/digua_bpu_yolo/share/digua_bpu_yolo
    # workspace root:
    # /home/sunrise/digua_ws
    try:
        workspace_root = pkg_share.parents[3]
    except Exception:
        workspace_root = Path("/home/sunrise/digua_ws")

    model_file = LaunchConfiguration("model_file").perform(context)
    if model_file == "auto":
        model_file = str(
            workspace_root
            / "models"
            / "bpu_yolov8s_oiv7"
            / "yolov8s-oiv7_bayese_640x640_nv12.bin"
        )

    classes_file = LaunchConfiguration("classes_file").perform(context)
    if classes_file == "auto":
        classes_file = str(pkg_share / "config" / "oiv7_classes.list")

    image_file = LaunchConfiguration("dnn_example_image").perform(context)

    cfg = {
        "model_file": model_file,
        "task_num": 4,
        "dnn_Parser": "yolov8",
        "model_output_count": 6,
        "reg_max": 16,
        "class_num": 601,
        "cls_names_list": classes_file,
        "strides": [8, 16, 32],
        "score_threshold": 0.50,
        "nms_threshold": 0.7,
        "nms_top_k": 300,
        "output_order": [0, 1, 2, 3, 4, 5],
    }

    runtime_dir = Path("/tmp/digua_bpu_yolo")
    runtime_dir.mkdir(parents=True, exist_ok=True)
    runtime_config = runtime_dir / "yolov8_oiv7_workconfig.json"

    with open(runtime_config, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)

    dnn_launch = os.path.join(
        get_package_share_directory("dnn_node_example"),
        "launch",
        "dnn_node_example_feedback.launch.py",
    )

    return [
        LogInfo(msg=f"[digua_bpu_yolo] runtime config: {runtime_config}"),
        LogInfo(msg=f"[digua_bpu_yolo] model_file: {model_file}"),
        LogInfo(msg=f"[digua_bpu_yolo] classes_file: {classes_file}"),
        LogInfo(msg=f"[digua_bpu_yolo] image_file: {image_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dnn_launch),
            launch_arguments={
                "dnn_example_config_file": str(runtime_config),
                "dnn_example_image": image_file,
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_file",
            default_value="auto",
            description="BPU YOLO model file. Use auto for digua_ws/models path.",
        ),
        DeclareLaunchArgument(
            "classes_file",
            default_value="auto",
            description="Class name list. Use auto for package config/oiv7_classes.list.",
        ),
        DeclareLaunchArgument(
            "dnn_example_image",
            default_value="/opt/tros/humble/lib/dnn_node_example/config/test.jpg",
            description="Image used by dnn_node_example feedback launch.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
