from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # =========================================================
    # 1. FAST-LIO2
    # =========================================================
    fast_lio2 = IncludeLaunchDescription(
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # =========================================================
    # 2. odom -> base_link
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
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
        }],
    )

    # =========================================================
    # 3. Submap Localization
    # =========================================================
    submap_localization = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'components',
                'submap_localization.launch.py',
            ])
        ),
        launch_arguments={
            'prior_map': LaunchConfiguration('prior_3d_map'),
            'config': LaunchConfiguration('localization_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([

        DeclareLaunchArgument(
            'prior_3d_map',
            description='Absolute path to prior 3D PCD'
        ),

        DeclareLaunchArgument(
            'localization_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'localization.yaml',
            ]),
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),

        fast_lio2,
        odom_to_base_link,
        submap_localization,
    ])
