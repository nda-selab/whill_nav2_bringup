from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    default_map = PathJoinSubstitution([
        FindPackageShare('whill_nav2_bringup'), 'maps', 'map.yaml',
    ])
    default_params = PathJoinSubstitution([
        FindPackageShare('whill_nav2_bringup'), 'config', 'nav2_params.yaml',
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': ParameterValue(
                use_sim_time,
                value_type=bool,
            ),
        }],
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'use_sim_time': ParameterValue(
                use_sim_time,
                value_type=bool,
            ),
            'node_names': ['map_server'],
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('whill_nav2_bringup'),
            'launch',
            'components',
            'whill_navigation.launch.py',
        ])),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),

        map_server,
        map_lifecycle,
        navigation,
    ])
