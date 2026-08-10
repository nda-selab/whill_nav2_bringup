from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # =========================================================
    # 1. Ouster raw point cloud -> LaserScan
    #
    # /ouster/points (sensor frame)
    #        ↓
    # static TF
    #        ↓
    # base_link
    #        ↓
    # pointcloud_to_laserscan
    #        ↓
    # /scan_ouster (frame: base_link)
    # =========================================================
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',

        remappings=[
            ('cloud_in', '/ouster/points'),
            ('scan', '/scan_ouster'),
        ],

        parameters=[{
            # LaserScanをbase_link基準で生成
            'target_frame': 'base_link',

            # TF待ち時間
            'transform_tolerance': 0.05,
            
            # 入力queue
            'queue_size': 1,

            # 使用する点群の高さ範囲
            'min_height': 0.10,
            'max_height': 1.50,

            # 360 deg
            'angle_min': -3.14159,
            'angle_max': 3.14159,

            # 約0.5 deg
            'angle_increment': 0.0087,

            # Ouster 10 Hz
            'scan_time': 0.1,

            # 使用距離
            'range_min': 0.3,
            'range_max': 30.0,

            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    # =========================================================
    # 2. Nav2 velocity command -> WHILL
    #
    # controller_server
    #      ↓
    # /cmd_vel_nav
    #      ↓
    # velocity_smoother
    #      ↓
    # /cmd_vel
    #      ↓
    # relay
    #      ↓
    # /whill/controller/cmd_vel
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
                'whill_nav2.launch.py',
            ])
        )
    )
    
    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([
        pointcloud_to_laserscan,
        cmd_vel_relay,
        nav2,
    ])
