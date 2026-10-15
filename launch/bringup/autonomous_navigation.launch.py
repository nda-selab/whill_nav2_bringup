from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution)
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
        ),
        launch_arguments={
            'os2_64_params_file': LaunchConfiguration('os2_64_params_file'),
            'urg_params_file': LaunchConfiguration('urg_params_file'),
        }.items(),
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
        ),
        launch_arguments={
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'cmd_vel_watchdog_params_file': LaunchConfiguration('cmd_vel_watchdog_params_file'),
        }.items(),
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
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([

        DeclareLaunchArgument(
            'os2_64_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'os2_64_params.yaml',
            ]),
            description='OS2-64 parameter YAML file',
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
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'nav2_params.yaml',
            ]),
            description='Nav2 parameter YAML file',
        ),

        DeclareLaunchArgument(
            'cmd_vel_watchdog_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'cmd_vel_watchdog_params.yaml',
            ]),
            description='CmdVel Watchdog parameter YAML file',
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
