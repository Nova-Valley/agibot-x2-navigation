<div align="center">
    <h1>SPARK-FAST-LIO</h1>
    <a href="https://github.com/MIT-SPARK/spark-fast-lio2"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/MIT-SPARK/spark-fast-lio2"><img src="https://img.shields.io/badge/ROS2-Jazzy-blue" /></a>
    <a href="https://github.com/MIT-SPARK/spark-fast-lio2"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2409.15615"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
  <br />
  <p align="center"><img src="https://github.com/user-attachments/assets/7eeb6c1d-1191-46b8-9547-ba9ad342dd2d" alt="KISS Matcher" width="85%"/></p>
  <p><strong><em>LiDAR mapping = SPARK-FAST-LIO + <a href="https://github.com/MIT-SPARK/KISS-Matcher/tree/main/ros">KISS-Matcher-SAM</a></em></strong></p>
</div>

______________________________________________________________________

## :package: How to Install

Put the code in your workspace/src folder

```shell
cd ${YOUR_COLCON_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/spark-fast-lio.git
colcon build --packages-up-to spark_fast_lio
```

## 🚀 How to Run

We provide **two out-of-the-box ROS2** examples using pre-processed ROS2 bag data (because the original data are only available in ROS1).
All pre-processed ROS2 bag files can be found [here](https://www.dropbox.com/scl/fo/i56kucdzxpzq1mr5jula7/ALJpdqvOZT1hTaQXEePCvyI?rlkey=y5bvslyazf09erko7gl0aylll&st=dh91zyho&dl=0).

### 🇺🇸 LIO on the MIT campus

1. Download `10_14_acl_jackal` and `10_14_hathor` (from the [Kimer-Multi dataset](https://github.com/MIT-SPARK/Kimera-Multi-Data))

1. Run `spark_fast_lio` using the following command:

```
ros2 launch spark_fast_lio mapping_mit_campus.launch.yaml scene_id:=acl_jackal
```

3. In another terminal, run ROS2 bag file as follows:

```
ros2 bag play 10_14_acl_jackal
```

### 🏟️ LIO on the Colosseum

1. Download `colosse_train0` (from the [VBR dataset](https://github.com/rvp-group/vbr-devkit))

1. Run `spark_fast_lio` using the following command:

```
ros2 launch spark_fast_lio mapping_vbr_colosseo.launch.yaml
```

3. In another terminal, run ROS2 bag file as follows:

```
ros2 bag play colosseo_train0
```

### 🔧 How to run `spark-fast-lio2` using your own ROS2 bag?

1. Copy `config/velodyne_mit.yaml` or `config/ouster_vbr.yaml` to `config/${YOUR_CONFIG}.yaml`, and set the appropriate values for:
   - `lidar_type`, `scan_line`, `timestamp_unit`, and `filter_size_map` depending on your sensor type
   - `extrinsic_T` and `extrinsic_R` (it's LiDAR w.r.t. IMU, i.e., `extrinsic * cloud w.r.t. LiDAR ->  cloud w.r.t. IMU`)
1. Configure your launch file and remap the lidar and imu topic names to match your setup.
   - Also set an appropriate rviz setup
1. Run your launch file, for example: `ros2 launch spark_fast_lio ${YOUR_LAUNCH}.launch.yaml`

### How to run with [KISS-Matcher-SAM](https://github.com/MIT-SPARK/KISS-Matcher/tree/main/ros)?

Please carefully read [README.md](https://github.com/MIT-SPARK/KISS-Matcher/blob/main/ros/README.md) of KISS-Matcher-SAM before running the command.

1. To install `kiss_matcher_ros` in your colcon workspace, run:

```bash
cd ${YOUR_ROS2_WORKSPACE}/src
git clone https://github.com/MIT-SPARK/KISS-Matcher.git
cd ..
colcon build --packages-select kiss_matcher_ros
```

2. Then, run the command below:

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml
```

3. By default, this setup is compatible with the two examples above (i.e., the topics are already remapped to support them).
   However, if you want to run it on your own dataset, make sure to set the `/cloud` and `/odom` topics appropriately using:

```
ros2 launch kiss_matcher_ros run_kiss_matcher_sam.launch.yaml \
  odom_topic:=<YOUR_TOPIC> scan_topic:=<YOUR_TOPIC>
```

______________________________________________________________________

## What's New? Key Features and Updates

1. Complete code refactoring for better structure and readability

1. Visualization frame can now be configured via launch file

1. Added support for gravity alignment

______________________________________________________________________

## Acknowledgement

Thanks to [HKU MaRS Lab](https://mars.hku.hk/) guys. The original code is from FAST-LIO2, which can be found [here](https://github.com/hku-mars/FAST_LIO).
