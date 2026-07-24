## 10_14

## 12_07

## 12_08

- `acl_jackal`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="acl_jackal"
```

- `acl_jackal2`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="acl_jackal2"
```

- `sparkal1`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="sparkal1"
```

#### Exceptional cases where needs more information (e.g., `lidar_frame`, `imu_frame`)

- `apis`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="apis" lidar_frame:="ouster_link" imu_frame:="camera_imu_optical_frame"
```

- `hathor`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="hathor" lidar_frame:=velodyne
```

- `sobek`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="sobek" lidar_frame:=ouster_link
```

- `sparkal2`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="sparkal2" lidar_frame:="velodyne"
```

- `thoth`

```
ros2 launch spark_fast_lio mapping_kimera_multi.launch.yaml scene_id:="thoth" lidar_frame:="ouster_link" imu_frame:="camera_imu_optical_frame"
```
