from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('prior_map', description='Absolute path to prior 3D PCD'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'localization.yaml',
            ]),
        ),

        Node(
            package='whill_nav2_bringup',
            executable='submap_localizer',
            name='submap_localizer',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'prior_map_path': LaunchConfiguration('prior_map'),
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                },
            ],
        ),
    ])