from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    database_path = LaunchConfiguration("database_path")
    odom_topic = LaunchConfiguration("odom_topic")
    rviz = LaunchConfiguration("rviz")
    rtabmap_viz = LaunchConfiguration("rtabmap_viz")

    rtabmap_launch_file = PathJoinSubstitution([
        FindPackageShare("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py"
    ])

    # RTAB-Map localization mode 参数：
    # 1. localization=true：加载已有 db 定位，不重新建图
    # 2. Mem/IncrementalMemory=false：定位模式
    # 3. Mem/InitWMWithAllNodes=true：启动时加载已有地图节点用于匹配
    # 4. RGBD/LocalizationSmoothing=true：定位平滑
    # 5. RGBD/MaxOdomCacheSize=0：关闭 odom cache，避免异常 odom 协方差干扰验证
    # 6. Vis/MinInliers=25：提高视觉匹配质量门槛
    # 7. RGBD/OptimizeMaxError=3：保留默认闭环错误保护
    # 8. Grid/Sensor=0：使用 /scan 生成/维护 2D 栅格
    rtabmap_args = (
        "--Mem/IncrementalMemory false "
        "--Mem/InitWMWithAllNodes true "
        "--RGBD/LocalizationSmoothing true "
        "--RGBD/MaxOdomCacheSize 0 "
        "--RGBD/OptimizeMaxError 3 "
        "--Vis/MinInliers 25 "
        "--Grid/Sensor 0"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "database_path",
            default_value="/home/sunrise/digua_ws/digua_maps/rtabmap/digua_online_20260511_222109.db"
        ),

        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odometry/filtered"
        ),

        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),

        # 方法 B 固化：
        # RTAB-Map 在 /rtabmap 命名空间下默认发布 /rtabmap/map。
        # Nav2 和 RViz 默认等待全局 /map。
        # 这里把 RTAB-Map 的 map / map_updates 重映射成全局 /map / /map_updates。
        GroupAction([
            SetRemap(src="map", dst="/map"),
            SetRemap(src="map_updates", dst="/map_updates"),

            # 保险：兼容已经被命名空间解析后的绝对话题名
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
