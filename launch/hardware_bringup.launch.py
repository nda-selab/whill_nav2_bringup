from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =========================================================
    # Launch arguments
    # =========================================================
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    urg_params_file = LaunchConfiguration('urg_params_file')

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
    #
    # base_link -> os_sensor
    # base_link -> laser
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
    # 4. Hokuyo URG
    # =========================================================
    urg_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        namespace='',
        output='screen',
        parameters=[
            urg_params_file,
        ],
        remappings=[
            ('scan', '/scan_urg'),
        ],
    )
    
    # =========================================================
    # URG Lifecycle
    #
    # unconfigured
    #     ↓
    # configure
    #     ↓
    # inactive
    #     ↓
    # activate
    #     ↓
    # active
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
    # Launch
    # =========================================================
    return LaunchDescription([

        # -----------------------------------------------------
        # Arguments
        # -----------------------------------------------------
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
        
        # -----------------------------------------------------
        # Hardware
        # -----------------------------------------------------

        # WHILL
        whill_bringup,
        
        # Static TF
        static_tf,

        # Ouster
        ouster,
        
        # Hokuyo URG
        urg_node,

        # configure終了後にactivate
        activate_urg_after_configure,

        # URG Lifecycleサービス立ち上がり待ち
        TimerAction(
            period=2.0,
            actions=[
                urg_configure,
            ],
        ),
    ])
