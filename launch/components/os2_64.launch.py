from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    os2_64_params_file = LaunchConfiguration('os2_64_params_file')

    os2_64 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch',
                'driver.launch.py',
            ])
        ),
        launch_arguments={
            'params_file': os2_64_params_file,
            'viz': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'os2_64_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'os2_64_params.yaml',
            ]),
        ),
        os2_64,
    ])