from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def get_default_database_path():
    """
    自动选择 RTAB-Map 定位数据库。

    优先级：
    1. /home/sunrise/digua_ws/digua_maps/current_map_name.txt
       -> /home/sunrise/digua_ws/digua_maps/rtabmap/<name>.db
    2. 如果 current_map_name.txt 不存在、为空、或对应 .db 不存在：
       自动选择 rtabmap 目录下最新的 *.db
    3. 如果仍然找不到：
       使用固定兜底 db
    """
    maps_root = Path("/home/sunrise/digua_ws/digua_maps")
    name_file = maps_root / "current_map_name.txt"
    rtabmap_dir = maps_root / "rtabmap"

    fallback = rtabmap_dir / "digua_online_20260511_222109.db"

    try:
        if name_file.exists():
            map_name = name_file.read_text(encoding="utf-8").strip()

            if map_name:
                if map_name.endswith(".db"):
                    db_path = rtabmap_dir / map_name
                else:
                    db_path = rtabmap_dir / f"{map_name}.db"

                if db_path.exists():
                    print(f"[rtabmap_localization_visual_3dof] current_map_name = {map_name}")
                    print(f"[rtabmap_localization_visual_3dof] use database_path = {db_path}")
                    return str(db_path)

                print(f"[rtabmap_localization_visual_3dof] WARNING: current map db does not exist: {db_path}")
            else:
                print(f"[rtabmap_localization_visual_3dof] WARNING: {name_file} is empty")
        else:
            print(f"[rtabmap_localization_visual_3dof] WARNING: {name_file} does not exist")

    except Exception as e:
        print(f"[rtabmap_localization_visual_3dof] WARNING: failed to read {name_file}: {e}")

    try:
        db_files = sorted(
            rtabmap_dir.glob("*.db"),
            key=lambda p: p.stat().st_mtime,
            reverse=True
        )

        if db_files:
            latest_db = db_files[0]
            print(f"[rtabmap_localization_visual_3dof] use latest database_path = {latest_db}")
            return str(latest_db)

        print(f"[rtabmap_localization_visual_3dof] WARNING: no *.db found in {rtabmap_dir}")

    except Exception as e:
        print(f"[rtabmap_localization_visual_3dof] WARNING: failed to search latest db in {rtabmap_dir}: {e}")

    print(f"[rtabmap_localization_visual_3dof] WARNING: use fallback database_path = {fallback}")
    return str(fallback)


def generate_launch_description():
    default_database_path = get_default_database_path()

    database_path = LaunchConfiguration("database_path")
    odom_topic = LaunchConfiguration("odom_topic")
    rviz = LaunchConfiguration("rviz")
    rtabmap_viz = LaunchConfiguration("rtabmap_viz")

    rtabmap_launch_file = PathJoinSubstitution([
        FindPackageShare("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py"
    ])

    # Visual-only + Force3DoF 稳定版：
    # Reg/Strategy=0：只使用视觉配准，不使用 ICP/scan 几何配准来修正位姿
    # Reg/Force3DoF=true：限制为平面机器人 x、y、yaw
    # RGBD/ProximityPathMaxNeighbors=0：关闭基于 scan proximity 的近邻闭环，减少相似场景拉扯
    #
    # 注意：仍然 subscribe_scan=true。
    # /scan 仍可用于 RTAB-Map 输出 2D 栅格地图，但不通过 ICP 直接拉动定位。
    rtabmap_args = (
        "--Mem/IncrementalMemory false "
        "--Mem/InitWMWithAllNodes true "
        "--RGBD/LocalizationSmoothing true "
        "--RGBD/MaxOdomCacheSize 0 "
        "--RGBD/OptimizeMaxError 3 "
        "--Vis/MinInliers 25 "
        "--Grid/Sensor 0 "
        "--Reg/Strategy 0 "
        "--Reg/Force3DoF true "
        "--RGBD/ProximityPathMaxNeighbors 0"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "database_path",
            default_value=default_database_path,
            description="RTAB-Map database path. Default is selected from current_map_name.txt."
        ),

        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odometry/filtered",
            description="Odometry topic for RTAB-Map localization."
        ),

        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),

        GroupAction([
            # 方法 B 固化：
            # RTAB-Map 在 /rtabmap 命名空间下默认发布 /rtabmap/map。
            # Nav2 / RViz 默认订阅 /map。
            # 所以这里把 RTAB-Map 地图输出重映射到标准 /map。
            SetRemap(src="map", dst="/map"),
            SetRemap(src="map_updates", dst="/map_updates"),

            # 保险：兼容已经解析成绝对话题的情况
            SetRemap(src="/rtabmap/map", dst="/map"),
            SetRemap(src="/rtabmap/map_updates", dst="/map_updates"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch_file),
                launch_arguments={
                    "use_sim_time": "false",

                    "localization": "true",
                    "database_path": database_path,

                    "visual_odometry": "false",
                    "odom_topic": odom_topic,

                    "frame_id": "base_footprint",
                    "map_frame_id": "map",

                    "subscribe_rgbd": "true",
                    "rgbd_topic": "/camera/rgbd_image",

                    "subscribe_scan": "true",
                    "scan_topic": "/scan",

                    "approx_sync": "true",
                    "wait_for_transform": "5.0",

                    "rviz": rviz,
                    "rtabmap_viz": rtabmap_viz,

                    "rtabmap_args": rtabmap_args,
                }.items()
            )
        ])
    ])
