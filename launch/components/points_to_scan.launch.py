from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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

    return LaunchDescription([pointcloud_to_laserscan])
