# lidar_localization_ros2による事前地図を用いた自己位置姿勢推定

## WHILL Driver

```bash
ros2 launch whill_bringup whill_launch.py
```

## Ouster Driver

```bash
ros2 launch whill_nav2_bringup os2_64.launch.py 
```

## lidar_localization_ros2の起動

```bash
ros2 run lidar_localization_ros2 quickstart.py \
--map '/home/selab/data/344/PCD/merged_map.pcd' \
--cloud-topic /ouster/points \
--lidar-frame os_lidar \
--imu-topic /ouster/imu \
--imu-frame os_imu \
--base-frame os_sensor
```
