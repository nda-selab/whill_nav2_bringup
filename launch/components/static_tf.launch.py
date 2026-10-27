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

    # 各URGの取り付け位置（x, y, z, roll, pitch, yaw）
    urg_mounts = {
        'laser_front': (0.70, 0.00, 0.05, 0.0, 0.0, 0.0),
        'laser_left':  (0.40, 0.20, 0.35, 0.0, 1.57079632679, 1.57079632679),
        'laser_right': (0.40, -0.20, 0.35, 0.0, 1.57079632679, -1.57079632679),
    }

    urg_tfs = []
    for frame, (x, y, z, roll, pitch, yaw) in urg_mounts.items():
        urg_tfs.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{frame}',
            arguments=[
                '--x', str(x),
                '--y', str(y),
                '--z', str(z),
                '--roll', str(roll),
                '--pitch', str(pitch),
                '--yaw', str(yaw),
                '--frame-id', 'base_link',
                '--child-frame-id', frame,
            ],
        ))

    return LaunchDescription([
        base_link_to_os_sensor,
        *urg_tfs
    ])
