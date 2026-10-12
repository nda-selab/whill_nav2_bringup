# Trouble Shooting

`--nav2-params`には，
Nav2起動時と同じ設定YAMLを指定する．省略時は本リポジトリの
`config/nav2_params.yaml`を使用する．事前チェックではlocal_costmapの
`footprint`と`footprint_padding`（省略時はNav2 Humbleと同じ0.01 m）を読み込み，
各目標のyawで回転させた車体の内側と外周を地図上で確認する．

`--clearance`は**車体の外周から追加で確保する距離**である．従来の
「waypoint中心からの自由半径」とは意味が異なり，`--clearance 0.45`では
車体外周からさらに0.45 mの余裕を要求する．追加余裕を付けずNav2の
footprintとpaddingだけで確認する場合は`--clearance 0.0`を指定する．
障害物セルは対角線の半分の半径を持つ円で覆って判定するため，セルの端や
角の接触も検出する一方，地図解像度に応じて判定が少し保守的になる．
このチェックは各目標姿勢と軌跡開始姿勢に対するものであり，区間全体の
走行可否はPlannerによる経路検査と走行中のNav2の衝突判定で確認する．

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