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
    cmd_vel_watchdog = Node(
        package='whill_nav2_bringup',
        executable='whill_cmd_vel_watchdog.py',
        name='whill_cmd_vel_watchdog',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/whill/controller/cmd_vel',
            'command_timeout_sec': 0.4,
            'publish_frequency': 20.0,
            'max_forward_speed': 0.25,
            'max_reverse_speed': 0.15,
            'max_angular_speed': 0.5,
        }],
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
