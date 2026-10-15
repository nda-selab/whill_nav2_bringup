from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)

from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution)

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =========================================================
    # 1. WHILL driver
    # =========================================================
    whill_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_bringup'),
                'launch',
                'whill_launch.py',
            ])
        )
    )

    # =========================================================
    # 2. Static TF
    #
    # base_link -> os_sensor
    # base_link -> laser
    # =========================================================
    static_tf = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'static_tf.launch.py',
            ])
        )
    )

    # =========================================================
    # 3. Ouster OS2-64
    # =========================================================
    os2_64 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'os2_64.launch.py',
            ])
        ),
        launch_arguments={
            'os2_64_params_file': LaunchConfiguration('os2_64_params_file'),
        }.items(),
    )

    # =========================================================
    # 4. URG
    # =========================================================
    urg = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'urg.launch.py',
            ])
        ),
        launch_arguments={
            'urg_params_file': LaunchConfiguration('urg_params_file'),
        }.items(),
    )

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([

        # -----------------------------------------------------
        # Arguments
        # -----------------------------------------------------
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
        
        whill_bringup,
        static_tf,
        os2_64,
        urg,
    ])
