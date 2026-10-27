from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

URG_DEVICES = {
    'front': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.3:1.0',
    'left': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0',
    'right': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0',
}

def create_urg_actions(position, device, params_file):
    node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=f'urg_{position}',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {
                'serial_port': device,
                'frame_id': f'laser_{position}',
            },
        ],
        remappings=[
            ('scan', f'/urg_{position}/scan'),
        ],
    )

    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
            handle_once=True,
        )
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True,
        )
    )

    return [activate, configure, node]

def generate_launch_description():
    params_file = LaunchConfiguration('urg_params_file')

    actions = [
        DeclareLaunchArgument(
            'urg_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'urg_params.yaml',
            ]),
        ),
    ]

    for position, device in URG_DEVICES.items():
        actions.extend(create_urg_actions(position, device, params_file))

    return LaunchDescription(actions)
