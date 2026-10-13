from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # FAST-LIO2
    # =========================================================
    fast_lio = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': 'os2_64.yaml',
            'rviz': 'false',
        }.items(),
    )

    # =========================================================
    # odom -> base_link
    # =========================================================
    odom_to_base_link = Node(
        package='whill_nav2_bringup',
        executable='odom_to_base_link.py',
        name='odom_to_base_link',
        output='screen',
        parameters=[{
            'input_topic': '/Odometry',
            'output_topic': '/odom_nav',
            'base_frame': 'base_link',
        }],
    )

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([        
        fast_lio,
        odom_to_base_link,
    ])
