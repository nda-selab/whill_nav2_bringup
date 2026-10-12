# TF Configuration

## TF Tree without a Prior Map

```text
odom
 └── base_link
       ├── os_sensor
       │    ├── os_lidar
       │    └── os_imu
       └── laser  
```

## TF Tree with a Prior Map

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