# Autonomous driving

## WHILL Nav2 Bringup

```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  prior_3d_map:=/home/selab/data/selab/pointcloud_2016-02-12-01-30-04_replay_loop_closed.pcd \
  prior_2d_map:=/home/selab/data/selab/nav2_map_csf/map.yaml
```

## Lidar Localization ROS2

```bash
ros2 run lidar_localization_ros2 lidar_localization_node \
  --ros-args \
  --params-file /home/selab/ros2_ws/src/lidar_localization_ros2/param/localization.yaml \
  -p map_path:=/home/selab/data/selab/pointcloud_2016-02-12-01-30-04_replay_loop_closed.pcd \
  -p use_sim_time:=false \
  -p set_initial_pose:=false \
  -p global_frame_id:=map \
  -p odom_frame_id:=odom \
  -p base_frame_id:=base_link \
  -p enable_map_odom_tf:=true \
  -p use_odom_tf_prediction:=true \
  -p use_imu_preintegration:=false \
  -p use_imu:=false \
  -p use_twist_prediction:=false \
  -p use_odom:=false \
  -p use_continuous_time_deskew:=false \
  -p enable_timer_publishing:=true \
  -p pose_publish_frequency:=20.0 \
  -p publish_bridge_pose_when_lost:=true \
  -r /cloud:=/cloud_registered 
```

## Lifecycle

```bash
ros2 run lidar_localization_ros2 start_lifecycle_node.py lidar_localization
```
