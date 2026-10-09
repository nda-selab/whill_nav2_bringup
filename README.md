# WHILL Nav2 Bringup

WHILLをROS 2 Humbleで制御し，Ouster LiDAR，FAST-LIO2，Nav2を組み合わせて自律走行を行うためのパッケージである．

- [WHILL Nav2 Bringup](#whill-nav2-bringup)
  - [Dependencies](#dependencies)
    - [ROS 2 packages](#ros-2-packages)
    - [WHILL ROS 2 driver](#whill-ros-2-driver)
    - [Ouster ROS 2 driver](#ouster-ros-2-driver)
    - [FAST-LIO2 ROS 2](#fast-lio2-ros-2)
  - [Environment](#environment)
  - [System Overview](#system-overview)
  - [TF Configuration](#tf-configuration)
    - [TF Tree without a Prior Map](#tf-tree-without-a-prior-map)
    - [TF Tree with a Prior Map](#tf-tree-with-a-prior-map)
  - [Initial Setup](#initial-setup)
    - [Check user groups](#check-user-groups)
  - [Manual Driving Test](#manual-driving-test)
  - [Sensor and FAST-LIO2 Test](#sensor-and-fast-lio2-test)
    - [Ouster Connection](#ouster-connection)
  - [Navigation without a Prior Map](#navigation-without-a-prior-map)
    - [Recommended Frames](#recommended-frames)
    - [Launch Order](#launch-order)
  - [Navigation Using a Prior Map](#navigation-using-a-prior-map)
    - [Nav2 Frame Parameters](#nav2-frame-parameters)
    - [Launch Order](#launch-order-1)
  - [Topic Check](#topic-check)
  - [TF Check](#tf-check)
  - [Costmap Check](#costmap-check)


## Dependencies

### ROS 2 packages

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-topic-tools \
  ros-humble-joy \
  ros-humble-tf2-tools \
  python3-yaml
```

### WHILL ROS 2 driver

- https://github.com/whill-labs/ros2_whill

### Ouster ROS 2 driver

- https://github.com/ouster-lidar/ouster-ros/tree/ros2

### FAST-LIO2 ROS 2

- https://github.com/Ericsii/FAST_LIO_ROS2

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- WHILL ROS 2 driver
- Ouster ROS 2 driver
- FAST-LIO2 ROS 2
- Nav2
- pointcloud_to_laserscan
- topic_tools
- joy

## System Overview

1. Odometry-based Navigation without a Prior Map

```text
Ouster LiDAR
 ├── /ouster/points
 └── /ouster/imu
        ↓
FAST-LIO2
 ├── /cloud_registered
 ├── /Odometry
 └── TF: odom -> base_link
        ↓
Static TF
 ├── base_link -> os_sensor
 └── base_link -> laser
        ↓
PointCloud to LaserScan
 └── /scan
        ↓
Nav2
 └── /cmd_vel_nav
        ↓
velocity_smoother
 └── /cmd_vel
        ↓
topic_tools relay
 └── /whill/controller/cmd_vel
        ↓
WHILL
```

2. Navigation Using a Prior Map

```text
Prior 3D Map
 └── merged_map.pcd
        ↓
PCD to OccupancyGrid Conversion
 ├── map.pgm
 └── map.yaml
        ↓
Map Server
 └── /map
        ↓
AMCL
 ├── Input: /map
 ├── Input: /scan
 └── TF: map -> odom

Ouster LiDAR
 ├── /ouster/points
 └── /ouster/imu
        ↓
FAST-LIO2
 ├── /cloud_registered
 ├── /Odometry
 └── TF: odom -> base_link
        ↓
Static TF
 ├── base_link -> os_sensor
 └── base_link -> base_link
        ↓
PointCloud to LaserScan
 └── /scan
        ↓
Nav2
 ├── Global Planner
 ├── Controller
 └── /cmd_vel_nav
        ↓
velocity_smoother
 └── /cmd_vel
        ↓
topic_tools relay
 └── /whill/controller/cmd_vel
        ↓
WHILL
```

## TF Configuration

### TF Tree without a Prior Map

```text
odom
 └── base_link
       ├── os_sensor
       │    ├── os_lidar
       │    └── os_imu
       └── laser  
```

### TF Tree with a Prior Map

```text
map
 └── odom
       └── base_link
             ├── os_sensor
             │    ├── os_lidar
             │    └── os_imu
             └── laser           
```

各TFの配信元は以下の通りである．

| TF | Publisher | Description |
|---|---|---|
| `map -> odom` | AMCL | 事前地図に対するFAST-LIO2オドメトリ座標系の補正 |
| `odom -> base_link` | FAST-LIO2 | LiDAR-IMUオドメトリ，Nav2用のWHILL基準座標系 |
| `base_link -> os_sensor` | `static_transform_publisher` | Ouster取り付け位置 |
| `base_link -> laser` | `static_transform_publisher` | URG取り付け位置 |
| `os_sensor -> os_lidar` | Ouster driver | LiDAR座標系 |
| `os_sensor -> os_imu` | Ouster driver | IMU座標系 |

## Initial Setup

### Check user groups

WHILLをUSBシリアル経由で制御する場合，
ユーザが`dialout`グループに所属している必要がある．

```bash
groups
```

`dialout`が含まれていない場合は以下を実行，
ログアウト・再ログインする．

```bash
sudo usermod -aG dialout $USER
```

## Manual Driving Test

- Terminal 1: WHILL

```bash
ros2 launch whill_bringup whill_launch.py
```

- Terminal 2: Joystick

```bash
ros2 run joy joy_node --ros-args \
  -p device_id:=0 \
  -p deadzone:=0.05 \
  -p autorepeat_rate:=20.0 \
  -r /joy:=/whill/controller/joy
```

ジョイスティックでWHILLを操作できることを確認する．

## Sensor and FAST-LIO2 Test

### Ouster Connection

```bash
ping -c 3 169.254.185.67
ip addr
```

- Terminal 1: Ouster

```bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.185.67 \
  point_type:=original \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  lidar_port:=7502 \
  imu_port:=7503 \
  viz:=false
```

- Terminal 2: FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py
```

## Navigation without a Prior Map

### Recommended Frames

```yaml
bt_navigator:
  ros__parameters:
    global_frame: odom
    robot_base_frame: base_link
    odom_topic: /Odometry

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true

behavior_server:
  ros__parameters:
    global_frame: odom
    robot_base_frame: base_link
```

### Launch Order

1. Ouster
2. FAST-LIO2
3. Static TF
4. PointCloud to LaserScan
5. WHILL driver
6. Nav2
7. RViz
8. Relay

- Terminal 1: Ouster

```bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.185.67 \
  point_type:=original \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  lidar_port:=7502 \
  imu_port:=7503 \
  viz:=false
```
  
- Terminal 2: FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py
```

- Terminal 3: Static TF

```bash
ros2 launch whill_nav2_bringup static_tf.launch.py
```

- Terminal 4: PointCloud to LaserScan

```bash
ros2 launch whill_nav2_bringup points_to_scan.launch.py
```

- Terminal 5: WHILL

```bash
ros2 launch whill_bringup whill_launch.py
```

- Terminal 6: Nav2

```bash
ros2 launch whill_nav2_bringup whill_nav2.launch.py
```

- Terminal 7: RViz

```bash
rviz2
```

- Terminal 8: Relay

```bash
ros2 run topic_tools relay \
  /cmd_vel \
  /whill/controller/cmd_vel
```

## Navigation Using a Prior Map

### Nav2 Frame Parameters

```yaml
amcl:
  ros__parameters:
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_link
    scan_topic: scan
    tf_broadcast: true

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /Odometry

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      rolling_window: false
      plugins:
        - static_layer
        - obstacle_layer
        - inflation_layer

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

behavior_server:
  ros__parameters:
    global_frame: odom
    robot_base_frame: base_link
```

### Launch Order

1. Ouster
2. FAST-LIO2
3. Static TF
4. PointCloud to LaserScan
5. Nav2, Map Server, AMCL
6. RViz
7. AMCL初期位置姿勢設定
8. WHILL driver
9. Relay
10. 実行

- Terminal 1: Ouster

```bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.185.67 \
  point_type:=original \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  lidar_port:=7502 \
  imu_port:=7503 \
  viz:=false
```
  
- Terminal 2: FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py
```

- Terminal 3: Static TF

```bash
ros2 launch whill_nav2_bringup static_tf.launch.py
```

- Terminal 4: PointCloud to LaserScan

```bash
ros2 launch whill_nav2_bringup points_to_scan.launch.py
```

- Terminal 5: Nav2

```bash
ros2 launch whill_nav2_bringup whill_nav2.launch.py
```

- Terminal 6: RViz

```bash
rviz2 -d \
  /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

- Terminal 7: AMCL初期位置姿勢設定

```bash
ros2 topic pub --once \
  /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    ]
  }
}"
```

- Terminal 8: WHILL

```bash
ros2 launch whill_bringup whill_launch.py
```

- Terminal 9: Relay

```bash
ros2 run topic_tools relay \
  /cmd_vel \
  /whill/controller/cmd_vel
```

- Terminal 10: 実行

```bash
python3 trajectory_waypoint_navigator.py \
  --trajectory /home/selab/data/bikan_outdoor/odom.csv \
  --map-yaml /home/selab/ros2_ws/src/whill_nav2_bringup/maps/map.yaml \
  --start-index 150 \
  --end-index 350 \
  --waypoint-spacing 0.80 \
  --corner-angle-deg 15.0 \
  --clearance 0.45 \
  --plan-preview \
  --execute
```

## Topic Check

```bash
ros2 topic list | grep ouster

ros2 topic info /cloud_registered -v
ros2 topic info /scan -v
ros2 topic info /map -v

ros2 topic echo /Odometry --once
ros2 topic echo /amcl_pose --once

ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /whill/controller/cmd_vel
```

## TF Check

```bash
ros2 run tf2_tools view_frames

ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

## Costmap Check

```bash
ros2 topic echo /global_costmap/costmap --once --field header
ros2 topic echo /local_costmap/costmap --once --field header
```