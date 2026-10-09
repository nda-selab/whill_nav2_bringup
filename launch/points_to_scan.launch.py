from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    urg_params_file = LaunchConfiguration('urg_params_file')

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/ouster_points'),
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

    urg_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        namespace='',
        output='screen',
        parameters=[urg_params_file],
        remappings=[
            ('scan', '/scan_urg'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'urg_params_file',
            default_value=(
                '/home/selab/ros2_ws/src/'
                'urg_node2/config/params_serial.yaml'
            ),
            description='Absolute path to params_serial.yaml for urg_node2',
        ),
        pointcloud_to_laserscan,
        urg_node,
    ])
