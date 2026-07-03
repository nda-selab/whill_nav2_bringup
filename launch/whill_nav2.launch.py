from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    ])

    params_file = PathJoinSubstitution([
        FindPackageShare('whill_nav2_bringup'),
        'config',
        'nav2_params.yaml'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'autostart': 'true',
            }.items()
        )
    ])
