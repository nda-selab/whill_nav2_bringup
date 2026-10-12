from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    urg_params_file = LaunchConfiguration('urg_params_file')

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/ouster/points'),
            ('scan', '/scan_ouster'),
        ],
        parameters=[{
            # LaserScanをbase_link基準で生成
            'target_frame': 'base_link',
            
            # TF待ち時間
            'transform_tolerance': 0.05,
            
            # 使用する点群の高さ範囲
            'min_height': 0.10,
            'max_height': 1.50,
            
            # 360 deg
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            
            # 約0.5 deg
            'angle_increment': 0.0087,
            
            # Ouster 10 Hz
            'scan_time': 0.1,
            
            # 使用距離
            'range_min': 0.3,
            'range_max': 30.0,
            
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    urg_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        namespace='',
        output='screen',
        parameters=[urg_params_file],
        remappings=[
            ('scan', '/scan_urg'),
        ],
    )

    # URGプロセス起動後にconfigure
    configure_urg = RegisterEventHandler(
        OnProcessStart(
            target_action=urg_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(urg_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ],
            handle_once=True,
        )
    )

    # URG: inactive -> active
    activate_urg = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=urg_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(urg_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'urg_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'config',
                'urg_params.yaml'
            ]),
            description='URG parameter YAML file',
        ),
        activate_urg,
        configure_urg,
        pointcloud_to_laserscan,
        urg_node,
    ])
