from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # =========================================================
    # PointCloud to LaserScan
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
        parameters=[LaunchConfiguration('pointcloud_to_laserscan_params_file')],
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
        
        pointcloud_to_laserscan,
    ])
