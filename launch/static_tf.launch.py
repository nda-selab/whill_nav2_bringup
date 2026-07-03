from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    body_to_os_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_to_os_sensor',
        arguments=[
            '--x', '-0.10',
            '--y', '0.00',
            '--z', '1.20',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'body',
            '--child-frame-id', 'os_sensor'
        ]
    )

    body_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_to_base_link',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '3.14159265359',
            '--frame-id', 'body',
            '--child-frame-id', 'base_link'
        ]
    )

    return LaunchDescription([
        body_to_os_sensor,
        body_to_base_link,
    ])
