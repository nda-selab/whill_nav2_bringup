# WHILL Nav2 Bringup

WHILLをROS 2 Humbleで制御し，Ouster LiDAR，FAST-LIO2，Nav2を組み合わせて自律走行を行うためのパッケージである．

## Environment

| Item | Details |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |

## System Overview

### Offline Map Preparation

```text
Prior 3D Map
 └── merged_map.pcd
        ↓
PCD to OccupancyGrid Conversion
 ├── map.pgm
 └── map.yaml
```

### Map-based Localization

```text
Map Server
 └── /map
        ↓
AMCL
 ├── Input: /map
 ├── Input: /scan_ouster
 └── TF: map -> odom
```

### Real-time Localization

```text
Ouster LiDAR
 ├── /ouster/points
 └── /ouster/imu
        ↓
FAST-LIO2
 ├── /cloud_registered
 ├── /Odometry
 ├── /odom_nav
 └── TF: odom -> base_link
```

### Obstacle Detection

```text
Ouster LiDAR
 └── /ouster/points
        ↓
PointCloud to LaserScan
 └── /scan_ouster
        ↓
AMCL, Local Costmap, Global Costmap
```

```text
Hokuyo URG
        ↓
urg_node2
 └── /scan_urg
        ↓
Local Costmap
```

### Static TF

```text
base_link
 ├── os_sensor
 │     ├── os_lidar
 │     └── os_imu
 └── laser
```

### Navigation and Control

```text
Nav2
 ├── Map / AMCL
 ├── /odom_nav
 ├── TF
 ├── Global Costmap
 └── Local Costmap
        ↓
Global Planner / Controller
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

### Nav2 Frame Parameters

Nav2の詳細な設定パラメータはconfig/nav2_params.yamlを参照する．

```yaml
amcl:
  ros__parameters:
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_link
    scan_topic: /scan_ouster
    tf_broadcast: true

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom_nav

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

### Integrated Bringup

```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py
```

起動後，RVizの「2D Pose Estimate」からロボットの初期位置・姿勢を設定する．
`map → odom`のTFが配信され，Nav2の自己位置が正常に取得できることを確認してから自律走行を開始する．

### Individual Launch Commands (Debugging)

統合起動とは併用しない．

1. Ouster

```bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.185.67 \
  point_type:=original \
  timestamp_mode:=TIME_FROM_ROS_TIME \
  lidar_port:=7502 \
  imu_port:=7503 \
  viz:=false
```

2. URG

```bash
ros2 launch whill_nav2_bringup urg.launch.py
```

3. FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=os2_64.yaml \
  rviz:=false
```

- 標準設定：`os2_64.yaml`
- 屋内環境設定：`os2_64_indoor.yaml`
- 屋外環境設定：`os2_64_outdoor.yaml`

4. Static TF

```bash
ros2 launch whill_nav2_bringup static_tf.launch.py
```

5. PointCloud to LaserScan

```bash
ros2 launch whill_nav2_bringup points_to_scan.launch.py
```

6. Nav2, Map Server, AMCL

```bash
ros2 launch whill_nav2_bringup whill_nav2.launch.py
```

7. RViz

```bash
ros2 launch whill_nav2_bringup visualization.launch.py
```

8. WHILL driver

```bash
ros2 launch whill_bringup whill_launch.py
```

9.  Relay

```bash
ros2 run topic_tools relay \
  /cmd_vel \
  /whill/controller/cmd_vel
```

### Waypoint Navigation

10. 実行

```bash
python3 scripts/trajectory_waypoint_navigator.py \
  --trajectory /home/selab/data/bikan_outdoor/odom.csv \
  --map-yaml /home/selab/ros2_ws/src/whill_nav2_bringup/maps/map.yaml \
  --nav2-params config/nav2_params.yaml \
  --start-index 150 \
  --end-index 350 \
  --waypoint-spacing 0.80 \
  --corner-angle-deg 15.0 \
  --clearance 0.45 \
  --plan-preview
```

安全確認後，`--execute`を指定すると，実走行を開始する．

## Documentation

- [Installation](doc/installation.md)
- [TF Configuration](doc/tf.md)
- [Troubleshooting](doc/troubleshooting.md)
