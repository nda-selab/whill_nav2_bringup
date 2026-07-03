# WHILL Nav2 Bringup

WHILLをROS 2 Humbleで制御し，Ouster LiDAR，FAST-LIO2，Nav2を組み合わせて自律走行を行うための起動手順である．

本構成では，Ouster LiDARから取得した点群をFAST-LIO2に入力して自己位置姿勢推定・地図生成を行い，点群をLaserScanに変換してNav2の障害物回避・経路追従に利用する．
Nav2の`controller_server`から出力される速度指令`/cmd_vel_nav`を`velocity_smoother`で平滑化し，`/cmd_vel`として出力する．
さらに，`topic_tools relay`により`/cmd_vel`をWHILLの制御トピック`/whill/controller/cmd_vel`へ転送することで，WHILLの自律走行を行う．

## Dependencies

### ROS 2 packages

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-topic-tools \
  ros-humble-joy \
  ros-humble-tf2-tools
```

### WHILL ROS 2 driver

- https://github.com/whill-labs/ros2_whill

### Ouster ROS 2 driver

- https://github.com/ouster-lidar/ouster-ros/tree/ros2

### FAST-LIO2 ROS 2

- https://github.com/Ericsii/FAST_LIO_ROS2

## System Overview

```text
Ouster LiDAR
 ├── /ouster/points
 └── /ouster/imu
        ↓
FAST-LIO2
 ├── /cloud_registered
 ├── /Odometry
 └── TF: camera_init -> body
        ↓
Static TF
 ├── body -> os_sensor
 └── body -> base_link
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

## Confirmed Working Configuration

本自律走行では，以下の構成を前提とする．

### TF Tree

```text
camera_init
 └── body
       ├── base_link
       └── os_sensor
             ├── os_lidar
             └── os_imu
```

各フレームの役割は以下の通りである．

| Frame | Description |
|----------|----------|
| `camera_init` | FAST-LIO2の初期座標系．Nav2の`global_frame`として使用する |
| `body` | FAST-LIO2が出力する機体座標系 |
| `base_link` | Nav2で使用するWHILL基準座標系．`body`に対してyaw 180度回転し，+x方向がWHILL前方 |
| `os_sensor` | Ouster LiDAR本体のセンサ座標系 |
| `os_lidar` | OusterのLiDAR点群座標系 |
| `os_imu` | OusterのIMU座標系 |

本環境では，FAST-LIO2が出力する`body`の+x方向がWHILLの後方を向いていた．
そのため，Nav2では`body`を直接`robot_base_frame`として使用せず，`base_link`を追加して使用する．

- body の +x方向 = WHILL後方
- base_link の +x方向 = WHILL前方

Nav2では以下のように設定する．

- global_frame: camera_init
- robot_base_frame: base_link

また，Nav2へ入力するLaserScan `/scan` も `base_link` 座標系で出力する．

```bash
ros2 topic echo /scan --once | grep frame_id
```

期待される出力は以下である．

```text
frame_id: base_link
```

## Initial Setup

### Check user groups

WHILLをUSBシリアル経由で制御する場合，ユーザが`dialout`グループに所属している必要がある．

```bash
groups
```

`dialout`が含まれていない場合は，以下を実行する．

```bash
sudo usermod -aG dialout $USER
```

その後，一度ログアウトして再ログインする．
再ログイン後，以下で`dialout`が含まれていることを確認する．

```bash
groups
```

## Manual Driving Test

まず，WHILLとジョイスティックが正しく動作するか確認する．

### Tab 1: WHILL

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch whill_bringup whill_launch.py
```

### Tab 2: Joystick

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run joy joy_node --ros-args \
  -p device_id:=0 \
  -p deadzone:=0.05 \
  -p autorepeat_rate:=20.0 \
  -r /joy:=/whill/controller/joy
```

この状態で，ジョイスティックによりWHILLを操作できることを確認する．

## Sensor and FAST-LIO2 Test

### Check Ouster Link-local Connection

Ouster LiDARのIPアドレスに対して通信確認を行う．

```bash
ping -c 3 169.254.185.67
```

ネットワークインタフェースは以下で確認する．

```bash
ip addr
```

### Tab 1: Ouster LiDAR

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.185.67 \
  point_type:=xyzi \
  lidar_port:=7502 \
  imu_port:=7503 \
  viz:=false
```

点群トピックを確認する．

```bash
ros2 topic list | grep ouster
ros2 topic echo /ouster/points --once
```

### Tab 2: FAST-LIO2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py
```

FAST-LIO2の出力を確認する．

```bash
ros2 topic list | grep cloud
ros2 topic list | grep -E "Odometry|odom"
ros2 topic echo /cloud_registered --once
```

TFを確認する．

```bash
ros2 run tf2_tools view_frames
```

FAST-LIO2起動後，少なくとも以下のTFが出力される．

```text
camera_init
 └── body
```

## Static TF Configuration

`static_transform_publisher` は実行後に終了せず，TFを配信し続ける．
そのため，表示が止まったように見えても正常であり，自律走行中は起動したままにする．

本構成では，以下の静的TFを追加する．

### `body -> os_sensor`

WHILLに対するOuster LiDARの取り付け位置を設定する．
以下は実機で使用した例であり，実際の取り付け位置に応じて変更する．

```bash
ros2 run tf2_ros static_transform_publisher \
 --x -0.10 \
 --y 0.00 \
 --z 1.20 \
 --roll 0.0 \
 --pitch 0.0 \
 --yaw 0.0 \
 --frame-id body \
 --child-frame-id os_sensor
```

この例では，`body`から見て`os_sensor`が以下の位置にあることを表す．

- x = -0.10 m
- y =  0.00 m
- z =  1.20 m

### `body -> base_link`

FAST-LIO2の`body`座標系に対して，Nav2用の`base_link`を定義する．
本環境では`body`の+x方向がWHILL後方を向いていたため，`base_link`をyaw方向に180度回転させる．

```bash
ros2 run tf2_ros static_transform_publisher \
 --x 0.0 \
 --y 0.0 \
 --z 0.0 \
 --roll 0.0 \
 --pitch 0.0 \
 --yaw 3.14159265359 \
 --frame-id body \
 --child-frame-id base_link
```

確認する．

```bash
ros2 run tf2_ros tf2_echo body base_link
```

期待される出力は以下である．

- Translation: `[0.000, 0.000, 0.000]`
- Rotation in RPY degree: yaw = ±180 deg

## PointCloud to LaserScan Configuration

Nav2には，Ousterの3D点群を2D LaserScanに変換した`/scan`を入力する．
本構成では，`/scan`の`frame_id`を`base_link`とする．

`/scan`が出力されているかを確認する．

```bash
ros2 topic echo /scan --once
```

`frame_id`を確認する．

```bash
ros2 topic echo /scan --once | grep frame_id
```

期待される出力は以下である．

```text
frame_id: base_link
```

## Nav2 Frame Parameters

Nav2の主要フレーム設定は以下のようにする．

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: camera_init
    robot_base_frame: base_link
    odom_topic: /Odometry
    bt_loop_duration: 10
    default_server_timeout: 20

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: camera_init
      robot_base_frame: base_link
      rolling_window: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: camera_init
      robot_base_frame: base_link
      rolling_window: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    global_frame: camera_init
    robot_base_frame: base_link
```

設定が反映されているか確認する．

```bash
ros2 param get /bt_navigator robot_base_frame
ros2 param get /local_costmap/local_costmap robot_base_frame
ros2 param get /global_costmap/global_costmap robot_base_frame
ros2 param get /behavior_server robot_base_frame
```

期待される出力はすべて以下である．

```text
String value is: base_link
```

## Autonomous Driving with Nav2
自律走行時は，以下の順番で起動する．

### Tab 1: Ouster LiDAR

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=169.254.185.67 \
  point_type:=xyzi \
  lidar_port:=7502 \
  imu_port:=7503 \
  viz:=false
```
  
### Tab 2: FAST-LIO2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py
```

### Tab 3: Static TF

`body -> os_sensor`および`body -> base_link`を配信する．

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch whill_nav2_bringup static_tf.launch.py
```

このlaunchでは，以下の2つの静的TFを配信する．

- `body -> os_sensor`
- `body -> base_link`

`static_transform_publisher` は実行後に終了せず，TFを配信し続ける．
そのため，表示が止まったように見えても正常であり，自律走行中は起動したままにする．
これらはlaunchファイル内に記述してもよい．

### Tab 4: PointCloud to LaserScan

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch whill_nav2_bringup points_to_scan.launch.py
```

### Tab 5: WHILL

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch whill_bringup whill_launch.py
```

### Tab 6: Nav2

Nav2は速度指令を`/cmd_vel_nav`に出力し，`velocity_smoother`によって`/cmd_vel`へ変換される．

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch whill_nav2_bringup whill_nav2.launch.py
```

### Tab 7: Relay

`/cmd_vel`をWHILLの制御トピック`/whill/controller/cmd_vel`へ転送する．

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run topic_tools relay /cmd_vel /whill/controller/cmd_vel
```

### Tab 8: RViz

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

rviz2
```

RVizのFixed Frameは以下に設定する．

```text
camera_init
```

表示項目として，少なくとも以下を追加する．

- TF
- LaserScan: `/scan`
- Local Costmap
- Global Costmap
- Path
- Footprint

## Recommended Launch Order

自律走行時の推奨起動順序は以下である．

1. Ouster LiDAR
2. FAST-LIO2
3. Static TF
   - `body -> os_sensor`
   - `body -> base_link`
4. PointCloud to LaserScan
5. WHILL
6. Nav2
7. Relay `/cmd_vel` -> `/whill/controller/cmd_vel`
8. RViz

## Topic Check

### Check Ouster Topics

```bash
ros2 topic list | grep ouster
```

### Check FAST-LIO2 Outputs

```bash
ros2 topic list | grep cloud
ros2 topic list | grep -E "Odometry|odom"

ros2 topic echo /cloud_registered --once
ros2 topic echo /Odometry --once
```

### Check LaserScan

Nav2に入力するLaserScanが出力されているか確認する．

```bash
ros2 topic echo /scan --once
```

### Check Nav2 Velocity Command

Nav2が速度指令を出力しているかを確認する．

```bash
ros2 topic echo /cmd_vel_nav
```

`velocity_smoother`後の速度指令を確認する．

```bash
ros2 topic echo /cmd_vel
```

### Check WHILL Velocity Command

WHILL側に速度指令が転送されているかを確認する．

```bash
ros2 topic echo /whill/controller/cmd_vel
```

### Check Topic Connections

```bash
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel -v
ros2 topic info /whill/controller/cmd_vel -v
```

正常時の速度指令の流れは以下である．

```text
controller_server
 ↓
/cmd_vel_nav
 ↓
velocity_smoother
 ↓
/cmd_vel
 ↓
relay
 ↓
/whill/controller/cmd_vel
 ↓
WHILL
```

## TF Check

Nav2ではTFの接続が重要である．
以下でTFツリーを確認する．

```bash
ros2 run tf2_tools view_frames
```

期待されるTF構成は以下である．

```text
camera_init
 └── body
       ├── base_link
       └── os_sensor
             ├── os_lidar
             └── os_imu
```

個別のTF確認は以下で行う．

```bash
ros2 run tf2_ros tf2_echo camera_init base_link
ros2 run tf2_ros tf2_echo body base_link
ros2 run tf2_ros tf2_echo base_link os_lidar
```

`body -> base_link`ではyawが約180度になっていることを確認する．

```text
Rotation in RPY degree: yaw = ±180 deg
```

## Costmap Check

Local costmapが生成されているか確認する．

```bash
ros2 topic echo /local_costmap/costmap --once
```

`frame_id`は`camera_init`でよい．
これはNav2の`global_frame`が`camera_init`であるためである．

更新周期を確認する．

```bash
ros2 topic hz /local_costmap/costmap
```

正常な場合，数Hz程度でcostmapが更新される．

## RViz Operation

RVizのFixed Frameを以下に設定する．

```text
camera_init
```

ゴール指定は，上部ツールバーの`2D Goal Pose`を使用する．

1. `2D Goal Pose`を選択する
2. ロボット近傍の空き領域をクリックする
3. ドラッグして到達時の向きを指定する
4. マウスを離してゴールを送信する

初回は安全のため，0.5 m程度の近距離目標から確認する．

## Cancel Navigation

RVizにNavigationパネルがある場合は，`Cancel`または`Cancel Task`を押す．

Navigationパネルは以下から追加できる．

```text
Panels -> Add New Panel -> Navigation 2
```

## Safety Stop

自律走行を停止する場合は，まずNav2のcontrollerを停止する．

```bash
ros2 lifecycle set /controller_server deactivate
```

再開する場合は以下を実行する．

```bash
ros2 lifecycle set /controller_server activate
```

WHILLへ直接ゼロ速度を送る場合は以下を実行する．

```bash
ros2 topic pub /whill/controller/cmd_vel geometry_msgs/msg/Twist \
 "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
 -r 10
```

実機試験では，必ず非常停止できる状態で行うこと．

## Notes

本構成では，FAST-LIO2の出力フレーム名である`camera_init`と`body`を利用している．
一般的なNav2構成では以下のようなTFが用いられる．

```text
map
 └── odom
       └── base_link
```

しかし，本実機構成では，以下を使用する．

```text
camera_init
  └── body
        └── base_link
```

将来的には，FAST-LIO2側または中継ノード側でフレーム名を整理し，以下のような標準的な構成へ統一することが望ましい．

```text
odom
  └── base_link
        └── os_sensor
              ├── os_lidar
              └── os_imu
```

ただし，現在の確認済み構成では，`camera_init`をNav2の`global_frame`，`base_link`を`robot_base_frame`として使用することで，WHILLの自律走行が可能である．

## Confirmed Result

本構成により，以下を確認した．

- Ouster LiDAR点群の取得
- FAST-LIO2による自己位置姿勢推定
- 点群からLaserScanへの変換
- Nav2による経路生成・障害物回避
- `/cmd_vel`から`/whill/controller/cmd_vel`への速度指令転送
- WHILLの自律走行

以上により，ROS 2 Humble上でWHILL，Ouster LiDAR，FAST-LIO2，Nav2を統合した自律走行を確認した．
