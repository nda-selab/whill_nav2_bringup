from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # =========================================================
    # 1. PointCloud to LaserScan
    # =========================================================
    pointcloud_to_laserscan = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'pointcloud_to_laserscan.launch.py',
            ])
        ),
        launch_arguments={
            'pointcloud_to_laserscan_params_file':
                LaunchConfiguration('pointcloud_to_laserscan_params_file'),
        }.items(),
    )

    # =========================================================
    # 2. 速度監視
    # =========================================================
    cmd_vel_watchdog = Node(
        package='whill_nav2_bringup',
        executable='cmd_vel_watchdog.py',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[LaunchConfiguration('cmd_vel_watchdog_params_file')],
    )
    
    # =========================================================
    # 3. Nav2
    # =========================================================
    nav2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'whill_nav2.launch.py',
            ])
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items(),
    )
    
    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([

        DeclareLaunchArgument(
            'pointcloud_to_laserscan_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'pointcloud_to_laserscan_params.yaml',
            ]),
            description='PointCloud to LaserScan parameter YAML file',
        ),
        
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'nav2_params.yaml',
            ]),
            description='Nav2 parameter YAML file',
        ),

        DeclareLaunchArgument(
            'cmd_vel_watchdog_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'cmd_vel_watchdog_params.yaml',
            ]),
            description='CmdVel Watchdog parameter YAML file',
        ),

        pointcloud_to_laserscan,
        cmd_vel_watchdog,
        nav2,
    ])
