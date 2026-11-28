# WHILL Nav2 Bringup

## 自律移動システムの起動

SLAMベース
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  localization_mode:=slam \
  prior_2d_map:=/path/to/map.yaml \
  prior_3d_map:=/path/to/map.ply \
  cmd_vel_watchdog_params_file:="$(ros2 pkg prefix --share cmd_vel_watchdog)/config/cmd_vel_watchdog_route_params.yaml"
```

EKFベース
```bash
ros2 launch whill_nav2_bringup autonomous_navigation.launch.py \
  localization_mode:=ekf \
  prior_2d_map:=/path/to/map.yaml \
  prior_3d_map:=/path/to/map.ply \
  cmd_vel_watchdog_params_file:="$(ros2 pkg prefix --share cmd_vel_watchdog)/config/cmd_vel_watchdog_route_params.yaml"
```

- 起動後，RVizの「2D Pose Estimate」からロボットの初期位置・姿勢を設定する．
- `map → odom`のTFが配信され，Nav2の自己位置が正常に取得できることを確認してから自律走行を開始する．

## 大域経路の事前確認

```bash
ros2 run nav2_waypoint_tools waypoint_route_runner.py \
  --route "/path/to/waypoints.yaml" \
  --map-yaml "/path/to/map.yaml" \
  --nav2-params "$(ros2 pkg prefix --share whill_nav2_bringup)/config/nav2_params.yaml" \
  --plan-preview
```

## 属性付き経路による走行

SLAMベース
```bash
ros2 run nav2_waypoint_tools waypoint_route_runner.py \
  --route "/path/to/waypoints.yaml" \
  --map-yaml "/path/to/map.yaml" \
  --nav2-params "$(ros2 pkg prefix --share whill_nav2_bringup)/config/nav2_params.yaml" \
  --execute \
  --route-control \
  --localization-mode slam \
  --initial-speed-limit 0.25 \
  --control-point-tolerance 0.25
```

EKFベース
```bash
ros2 run nav2_waypoint_tools waypoint_route_runner.py \
  --route "/path/to/waypoints.yaml" \
  --map-yaml "/path/to/map.yaml" \
  --nav2-params "$(ros2 pkg prefix --share whill_nav2_bringup)/config/nav2_params.yaml" \
  --execute \
  --route-control \
  --localization-mode ekf \
  --ekf-3d \
  --initial-speed-limit 0.25 \
  --control-point-tolerance 0.25
```
