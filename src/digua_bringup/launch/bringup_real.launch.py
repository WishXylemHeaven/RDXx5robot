from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_description = LaunchConfiguration('start_description')
    start_base = LaunchConfiguration('start_base')
    start_ekf = LaunchConfiguration('start_ekf')
    start_lidar = LaunchConfiguration('start_lidar')
    start_camera = LaunchConfiguration('start_camera')
    start_rgbd_sync = LaunchConfiguration('start_rgbd_sync')

    base_port = LaunchConfiguration('base_port')
    base_baudrate = LaunchConfiguration('base_baudrate')

    astra_enable_point_cloud = LaunchConfiguration('astra_enable_point_cloud')
    astra_enable_ir = LaunchConfiguration('astra_enable_ir')
    astra_publish_tf = LaunchConfiguration('astra_publish_tf')

    # -------------------------
    # Launch file paths
    # -------------------------

    description_launch = PathJoinSubstitution([
        FindPackageShare('digua_description'),
        'launch',
        'display.launch.py'
    ])

    base_control_launch = PathJoinSubstitution([
        FindPackageShare('base_control_ros2'),
        'launch',
        'base_control.launch.py'
    ])

    ekf_launch = PathJoinSubstitution([
        FindPackageShare('digua_bringup'),
        'launch',
        'ekf.launch.py'
    ])

    lidar_launch = PathJoinSubstitution([
        FindPackageShare('digua_bringup'),
        'launch',
        'lidar_x2.launch.py'
    ])

    astra_launch = PathJoinSubstitution([
        FindPackageShare('astra_camera'),
        'launch',
        'astra.launch.xml'
    ])

    rgbd_sync_launch = PathJoinSubstitution([
        FindPackageShare('digua_bringup'),
        'launch',
        'rgbd_sync.launch.xml'
    ])

    return LaunchDescription([
        # -------------------------
        # Common arguments
        # -------------------------

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time. Real robot should use false.'
        ),

        DeclareLaunchArgument(
            'start_description',
            default_value='true',
            description='Start robot_state_publisher and publish robot URDF TF.'
        ),

        DeclareLaunchArgument(
            'start_base',
            default_value='true',
            description='Start chassis base controller.'
        ),

        DeclareLaunchArgument(
            'start_ekf',
            default_value='true',
            description='Start robot_localization EKF.'
        ),

        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Start YDLIDAR X2 driver.'
        ),

        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start Astra S camera driver.'
        ),

        DeclareLaunchArgument(
            'start_rgbd_sync',
            default_value='true',
            description='Start RGB-D sync node.'
        ),

        # -------------------------
        # Base controller arguments
        # -------------------------

        DeclareLaunchArgument(
            'base_port',
            default_value='/dev/ttyS1',
            description='Serial port for chassis lower controller.'
        ),

        DeclareLaunchArgument(
            'base_baudrate',
            default_value='115200',
            description='Baudrate for chassis lower controller.'
        ),

        # -------------------------
        # Astra camera arguments
        # -------------------------

        DeclareLaunchArgument(
            'astra_enable_point_cloud',
            default_value='false',
            description='Enable Astra point cloud. Keep false for normal bringup.'
        ),

        DeclareLaunchArgument(
            'astra_enable_ir',
            default_value='false',
            description='Enable Astra IR stream. Keep false for normal bringup.'
        ),

        DeclareLaunchArgument(
            'astra_publish_tf',
            default_value='true',
            description='Let Astra publish internal camera TF frames.'
        ),

        # -------------------------
        # 1. Robot description / static TF
        # -------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=IfCondition(start_description),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'false',
            }.items()
        ),

        # -------------------------
        # 2. Chassis base controller
        # -------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_control_launch),
            condition=IfCondition(start_base),
            launch_arguments={
                'port': base_port,
                'baudrate': base_baudrate,
                'pub_imu': 'true',

                # Important:
                # EKF publishes odom -> base_footprint.
                # base_control should only publish /odom and /imu messages.
                'broadcast_odom_tf': 'false',
            }.items()
        ),

        # -------------------------
        # 3. EKF
        # -------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            condition=IfCondition(start_ekf),
        ),

        # -------------------------
        # 4. YDLIDAR X2
        # -------------------------

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            condition=IfCondition(start_lidar),
        ),

        # -------------------------
        # 5. Astra S camera
        # -------------------------

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch),
            condition=IfCondition(start_camera),
            launch_arguments={
                'enable_point_cloud': astra_enable_point_cloud,
                'enable_ir': astra_enable_ir,
                'publish_tf': astra_publish_tf,
            }.items()
        ),

        # -------------------------
        # 6. RGB-D sync
        # Start a little later to let camera topics appear first.
        # -------------------------

        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(rgbd_sync_launch),
                    condition=IfCondition(start_rgbd_sync),
                )
            ]
        ),
    ])
