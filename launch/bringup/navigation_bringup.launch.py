from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # =========================================================
    # 1. PointCloud to LaserScan
    # =========================================================
    points_to_scan = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'points_to_scan.launch.py',
            ])
        )
    )

    # =========================================================
    # 2. 速度監視
    # =========================================================
    cmd_vel_watchdog_params = PathJoinSubstitution([
        FindPackageShare('whill_nav2_bringup'),
        'config',
        'cmd_vel_watchdog_params.yaml',
    ])

    cmd_vel_watchdog = Node(
        package='whill_nav2_bringup',
        executable='cmd_vel_watchdog.py',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[cmd_vel_watchdog_params],
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
        )
    )
    
    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([
        points_to_scan,
        cmd_vel_watchdog,
        nav2,
    ])
