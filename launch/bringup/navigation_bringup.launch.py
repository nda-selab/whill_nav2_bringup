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
    # 2. Nav2 velocity command -> WHILL
    # =========================================================
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_to_whill_relay',
        output='screen',

        arguments=[
            '/cmd_vel',
            '/whill/controller/cmd_vel',
        ],
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
        cmd_vel_relay,
        nav2,
    ])
