import os from launch import LaunchDescription from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction from launch.launch_description_sources 
import PythonLaunchDescriptionSource from launch.substitutions import LaunchConfiguration, PathJoinSubstitution from launch_ros.substitutions import FindPackageShare def 
launch_setup(context, *args, **kwargs):
    home = os.path.expanduser("~") workspace_dir = os.path.join(home, "digua_ws") map_root_dir = os.path.join(workspace_dir, "digua_maps") rtabmap_dir = os.path.join(map_root_dir, 
    "rtabmap") map_name = LaunchConfiguration("map_name").perform(context).strip() database_path = LaunchConfiguration("database_path").perform(context).strip() if not database_path:
        if not map_name: current_map_name_file = os.path.join(map_root_dir, "current_map_name.txt") if os.path.exists(current_map_name_file): with open(current_map_name_file, "r", 
                encoding="utf-8") as f:
                    map_name = f.read().strip() if not map_name: raise RuntimeError( "没有指定 map_name，也没有找到 current_map_name.txt。" "请使用 map_name:=你的地图名 或 
                database_path:=完整.db路径"
            ) database_path = os.path.join(rtabmap_dir, f"{map_name}.db") if not os.path.exists(database_path): raise RuntimeError(f"RTAB-Map 数据库不存在: {database_path}") 
    rtabmap_launch_file = PathJoinSubstitution([
        FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py" ])
    # 定位模式关键点： 1. 不要 --delete_db_on_start 2. Mem/IncrementalMemory=false 表示不再扩展地图，只在已有地图中定位 3. Mem/InitWMWithAllNodes=true 
    # 表示启动时加载已有地图节点用于匹配
    rtabmap_args = ( "--Mem/IncrementalMemory false " "--Mem/InitWMWithAllNodes true " "--RGBD/LocalizationSmoothing true " "--RGBD/OptimizeFromGraphEnd false" ) return [ 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file), launch_arguments={ "use_sim_time": "false",
                # 你的项目已经有 /odometry/filtered，所以这里继续不用视觉里程计
                "visual_odometry": "false", "odom_topic": "/odometry/filtered",
                # 保持和建图时一致
                "frame_id": "base_footprint", "map_frame_id": "map",
                # 复用现有 RGB-D 和雷达输入
                "subscribe_rgbd": "true", "rgbd_topic": "/camera/rgbd_image", "subscribe_scan": "true", "scan_topic": "/scan", "approx_sync": LaunchConfiguration("approx_sync"), 
                "wait_for_transform": LaunchConfiguration("wait_for_transform"),
                # RDK X5 上默认不开 GUI
                "rviz": LaunchConfiguration("rviz"), "rtabmap_viz": LaunchConfiguration("rtabmap_viz"), "database_path": database_path, "args": rtabmap_args,
            }.items()
        ) ] def generate_launch_description(): return LaunchDescription([ DeclareLaunchArgument( "map_name", default_value="", description="RTAB-Map 地图名，不带 .db。为空时尝试读取 
            ~/digua_ws/digua_maps/current_map_name.txt"
        ), DeclareLaunchArgument( "database_path", default_value="", description="已有 RTAB-Map .db 文件完整路径。优先级高于 map_name" ), DeclareLaunchArgument( "rviz", 
            default_value="false", description="是否启动 RViz"
        ), DeclareLaunchArgument( "rtabmap_viz", default_value="false", description="是否启动 RTAB-Map GUI" ), DeclareLaunchArgument( "wait_for_transform", default_value="2.0", 
            description="TF 等待时间"
        ), DeclareLaunchArgument( "approx_sync", default_value="true", description="RGB-D、scan、odom 是否使用近似同步" ), OpaqueFunction(function=launch_setup),
    ])
