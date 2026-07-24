#!/usr/bin/env python3
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TsCheckSync(Node):
    def __init__(self):
        super().__init__("ts_check_sync")

        # ===== 固定配置：不需要终端参数 =====
        self.lidar_topic = "/lidar/points_rectified"
        self.imu_topic   = "/imu/rectified"   # 或者你原始IMU：/aima/hal/sensor/lidar_chest_front/imu
        self.field_name  = "time"
        self.print_every = 10
        # FAST-LIO 源码里不 sync 的阈值是 10s（你 grep 看到的那段）
        self.unsync_threshold_sec = 10.0
        # ===================================

        self._cnt = 0
        self.last_imu_t = None

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.cb_imu, qos_profile_sensor_data)
        self.sub_lid = self.create_subscription(PointCloud2, self.lidar_topic, self.cb_lidar, qos_profile_sensor_data)

        self.get_logger().info(f"✅ TsCheckSync LiDAR: {self.lidar_topic} (field={self.field_name})")
        self.get_logger().info(f"✅ TsCheckSync IMU:   {self.imu_topic}")

    def cb_imu(self, msg: Imu):
        self.last_imu_t = stamp_to_sec(msg.header.stamp)

    def cb_lidar(self, msg: PointCloud2):
        self._cnt += 1
        if self._cnt % self.print_every != 0:
            return

        # ---- lidar header time ----
        t_lid = stamp_to_sec(msg.header.stamp)

        # ---- compute frame-internal time span from field 'time' ----
        time_off = None
        time_dt = None
        for f in msg.fields:
            if f.name == self.field_name:
                time_off = f.offset
                time_dt = f.datatype
                break
        if time_off is None:
            self.get_logger().error(f"❌ field '{self.field_name}' not found in cloud")
            return
        if time_dt not in (7, 8):
            self.get_logger().warn(f"⚠️ field '{self.field_name}' datatype={time_dt} (expected float32/64)")

        npts = msg.width * msg.height
        step = msg.point_step
        data = msg.data

        sample_n = min(npts, 2000)
        idxs = np.linspace(0, npts - 1, sample_n, dtype=np.int64)

        if time_dt == 8:
            unpack = struct.Struct("<d").unpack_from
        else:
            unpack = struct.Struct("<f").unpack_from

        ts = np.empty(sample_n, dtype=np.float64)
        for i, pi in enumerate(idxs):
            base = int(pi) * step + time_off
            ts[i] = float(unpack(data, base)[0])

        tmin = float(np.nanmin(ts))
        tmax = float(np.nanmax(ts))
        span = tmax - tmin

        # ---- sync check: lidar header vs latest imu header ----
        if self.last_imu_t is None:
            self.get_logger().warn(f"[frame#{self._cnt}] LiDAR header={t_lid:.6f}s span={span:.6f}s | IMU not received yet")
            return

        dt = self.last_imu_t - t_lid
        verdict = "✅ SYNC-ish" if abs(dt) <= self.unsync_threshold_sec else "🚨 UNSYNC"

        self.get_logger().info(
            f"[frame#{self._cnt}] LiDAR header={t_lid:.6f}s  IMU(last)={self.last_imu_t:.6f}s  "
            f"dt(imu-lidar)={dt:.6f}s  {verdict} | time_span={span:.6f}s"
        )


def main():
    rclpy.init()
    node = TsCheckSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
