from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # =========================================================
    # Launch arguments
    # =========================================================
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    urg_params_file = LaunchConfiguration('urg_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # =========================================================
    # 1. WHILL driver
    # =========================================================
    whill_bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_bringup'),
                'launch',
                'whill_launch.py',
            ])
        )
    )

    # =========================================================
    # 2. Static TF
    # =========================================================
    static_tf = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'static_tf.launch.py',
            ])
        )
    )

    # =========================================================
    # 3. Ouster OS2-64
    # =========================================================
    ouster = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch',
                'sensor.launch.xml',
            ])
        ),
        launch_arguments={
            'sensor_hostname': sensor_hostname,
            'point_type': 'original',
            'timestamp_mode': 'TIME_FROM_ROS_TIME',
            'lidar_port': '7502',
            'imu_port': '7503',
            'viz': 'false',
        }.items(),
    )

    # =========================================================
    # 4. FAST-LIO2
    # =========================================================
    fast_lio = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py',
            ])
        )
    )

    # =========================================================
    # 5. pointcloud_to_laserscan + Hokuyo URG
    # =========================================================
    points_to_scan = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'points_to_scan.launch.py',
            ])
        ),
        launch_arguments={
            'urg_params_file': urg_params_file,
        }.items(),
    )
    
    # =========================================================
    # URG Lifecycle
    # =========================================================

    urg_configure = ExecuteProcess(
        cmd=[
            'ros2',
            'lifecycle',
            'set',
            '/urg_node2',
            'configure',
        ],
        output='screen',
    )

    urg_activate = ExecuteProcess(
        cmd=[
            'ros2',
            'lifecycle',
            'set',
            '/urg_node2',
            'activate',
        ],
        output='screen',
    )

    activate_urg_after_configure = RegisterEventHandler(
        OnProcessExit(
            target_action=urg_configure,
            on_exit=[
                urg_activate,
            ],
        )
    )
    
    
    # =========================================================
    # 6. /cmd_vel -> WHILL
    # =========================================================
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_to_whill_relay',
        output='screen',
        arguments=[
            '/cmd_vel',
            '/whill/controller/cmd_vel',
        ],
    )

    # =========================================================
    # 7. Nav2
    # =========================================================
    nav2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('whill_nav2_bringup'),
                'launch',
                'whill_nav2.launch.py',
            ])
        )
    )

    # =========================================================
    # 8. RViz
    # =========================================================
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

    # =========================================================
    # Launch
    # =========================================================
    return LaunchDescription([

        DeclareLaunchArgument(
            'sensor_hostname',
            default_value='os-992029000281.local',
            description='Ouster sensor hostname',
        ),

        DeclareLaunchArgument(
            'urg_params_file',
            default_value=(
                '/home/selab/ros2_ws/src/'
                'urg_node2/config/params_serial.yaml'
            ),
            description='URG parameter YAML file',
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz',
        ),

        # 0 s
        whill_bringup,
        static_tf,

        # 2 s
        TimerAction(
            period=2.0,
            actions=[
                ouster,
            ],
        ),

        # 5 s
        TimerAction(
            period=5.0,
            actions=[
                fast_lio,
            ],
        ),

        # 8 s
        # pointcloud_to_laserscan + URGノード起動
        TimerAction(
            period=8.0,
            actions=[
                points_to_scan,
            ],
        ),

        # configure完了後にactivateを実行するためのイベント
        activate_urg_after_configure,

        # 10 s
        TimerAction(
            period=10.0,
            actions=[
                urg_configure,
            ],
        ),
        
        # 15 s
        TimerAction(
            period=15.0,
            actions=[
                cmd_vel_relay,
            ],
        ),

        # 18 s
        # URGのtime calibration完了後にNav2起動
        TimerAction(
            period=18.0,
            actions=[
                nav2,
            ],
        ),

        # 21 s
        TimerAction(
            period=21.0,
            actions=[
                rviz,
            ],
        ),
    ])
