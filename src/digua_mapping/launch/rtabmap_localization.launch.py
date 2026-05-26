from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # 第一轮质量优化：
    # 1. localization=true：加载已有 db 定位，不重新建图
    # 2. Mem/IncrementalMemory=false：定位模式
    # 3. Mem/InitWMWithAllNodes=true：启动时加载已有地图节点用于匹配
    # 4. RGBD/MaxOdomCacheSize=0：先关闭 odom cache，避免高协方差 odom 反复干扰定位验证
    # 5. Vis/MinInliers=25：提高视觉匹配质量门槛，减少误匹配
    # 6. RGBD/OptimizeMaxError=3：保持默认错误闭环保护，不放宽
    # 7. Grid/Sensor=0：明确使用 /scan 生成/维护 2D 栅格
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

        # 重要：这里默认先改用 /odom，不用 /odometry/filtered
        # 因为你当前 /odometry/filtered 的 x/y pose covariance 已经到 5~9，RTAB-Map 会判定 high variance。
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odometry/filtered"
        ),

        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),

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
