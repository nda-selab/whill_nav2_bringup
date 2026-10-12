from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # =========================================================
    # 1. Hardware bringup
    # =========================================================
    hardware_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'bringup',
                'hardware_bringup.launch.py',
            ])
        )
    )

    # =========================================================
    # 2. Localization
    # =========================================================
    localization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'localization.launch.py',
            ])
        )
    )

    # =========================================================
    # 3. Navigation bringup
    # =========================================================
    navigation_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'bringup',
                'navigation_bringup.launch.py',
            ])
        )
    )

    # =========================================================
    # 4. Visualization
    # =========================================================
    visualization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'visualization.launch.py',
            ])
        )
    )

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([

        DeclareLaunchArgument(
            'sensor_hostname',
            default_value='os-992029000281.local',
            description='Ouster sensor hostname',
        ),

        DeclareLaunchArgument(
            'urg_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'urg_params.yaml',
            ]),
            description='URG parameter YAML file',
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz',
        ),

        hardware_bringup,
        TimerAction(period=5.0, actions=[localization]),
        TimerAction(period=10.0, actions=[navigation_bringup]),
        TimerAction(period=13.0, actions=[visualization]),
    ])
