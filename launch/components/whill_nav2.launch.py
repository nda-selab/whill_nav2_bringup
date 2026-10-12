from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    nav2_bringup_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "bringup_launch.py",
    ])

    default_map_yaml = PathJoinSubstitution([
        FindPackageShare("whill_nav2_bringup"),
        "maps",
        "map.yaml",
    ])

    default_params_file = PathJoinSubstitution([
        FindPackageShare("whill_nav2_bringup"),
        "config",
        "nav2_params.yaml",
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),

        DeclareLaunchArgument(
            "map",
            default_value=default_map_yaml,
            description="Path to the Nav2 occupancy grid map YAML",
        ),

        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the Nav2 parameter file",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map": map_yaml,
                "params_file": params_file,
                "autostart": "true",
                "slam": "False",
            }.items(),
        ),
    ])

