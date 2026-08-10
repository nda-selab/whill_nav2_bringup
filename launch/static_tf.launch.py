from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    base_link_to_os_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_os_sensor',
        arguments=[
            '--x', '-0.12',
            '--y', '0.00',
            '--z', '0.70',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'os_sensor',
        ]
    )
    
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_laser',
        arguments=[
            '--x', '0.70',
            '--y', '0.00',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ]
    )
    
    return LaunchDescription([
        base_link_to_os_sensor,
        base_link_to_laser
    ])
