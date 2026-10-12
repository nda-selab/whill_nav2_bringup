from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)

from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution)

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =========================================================
    # Launch arguments
    # =========================================================
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    urg_params_file = LaunchConfiguration('urg_params_file')

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
    ouster = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch',
                'sensor.launch.xml',
            ])
        ),
        launch_arguments={
            'sensor_hostname': sensor_hostname,
            'point_type': 'original',
            'timestamp_mode': 'TIME_FROM_ROS_TIME',
            'lidar_port': '7502',
            'imu_port': '7503',
            'viz': 'false',
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
            'urg_params_file': urg_params_file,
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
        
        # -----------------------------------------------------
        # Hardware
        # -----------------------------------------------------

        # WHILL
        whill_bringup,
        
        # Static TF
        static_tf,

        # Ouster
        ouster,
        
        # URG
        urg,
    ])
