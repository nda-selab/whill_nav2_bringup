from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'localization_launch.py',
        ])),
        launch_arguments={
            'namespace': '',
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
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
        localization,
        navigation,
    ])
