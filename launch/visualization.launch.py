from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    use_rviz = LaunchConfiguration('use_rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz',
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz',
        ),

        rviz,
    ])
