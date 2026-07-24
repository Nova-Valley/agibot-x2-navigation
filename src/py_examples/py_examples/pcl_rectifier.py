#!/usr/bin/env python3
import math
import numpy as np
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)
from sensor_msgs.msg import PointCloud2, PointField


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]], dtype=np.float64)
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]], dtype=np.float64)
    Rx = np.array([[1,  0,   0],
                   [0, cr, -sr],
                   [0, sr,  cr]], dtype=np.float64)
    return (Rz @ Ry @ Rx)


def make_pointfield(name, offset, datatype, count=1):
    f = PointField()
    f.name = name
    f.offset = offset
    f.datatype = datatype
    f.count = count
    return f


class PclRectifier(Node):
    def __init__(self):
        super().__init__("pcl_rectifier")

        # ===== 你只改这里（固定配置，无需终端参数）=====
        self.input_topic  = "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
        self.output_topic = "/lidar/points_rectified"
        self.output_frame = "lidar_chest_front"

        # 点云扶正（按你之前选好的）
        self.roll_deg  = -90.0
        self.pitch_deg = 0.0
        self.yaw_deg   = 0.0

        # timestamp 单位：你的 span≈9.48e7 对应 0.0948s → 强烈像 ns
        self.timestamp_scale = 1e-9   # raw timestamp * 1e-9 = seconds
        # =================================================

        self.R = rpy_to_R(math.radians(self.roll_deg),
                          math.radians(self.pitch_deg),
                          math.radians(self.yaw_deg))
                          
        # ✅ 输入：匹配传感器（BEST_EFFORT）
        qos_in = qos_profile_sensor_data

        # ✅ 输出：给下游用（RELIABLE）
        qos_out = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
                         

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_out)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb, qos_in)
        
        self.get_logger().info(f"Rectifier started: {self.input_topic} -> {self.output_topic}")
        self.get_logger().info(f"RPY(deg): {self.roll_deg}, {self.pitch_deg}, {self.yaw_deg}")
        self.get_logger().info(f"timestamp_scale={self.timestamp_scale} (raw->sec), output adds field 'time' (float32, rel)")

    def cb(self, msg: PointCloud2):
        # find offsets
        off = {}
        dt = {}
        for f in msg.fields:
            off[f.name] = f.offset
            dt[f.name] = f.datatype

        need = ["x", "y", "z", "timestamp"]
        for k in need:
            if k not in off:
                self.get_logger().error(f"Missing field '{k}' in input cloud.")
                return

        x_off, y_off, z_off, ts_off = off["x"], off["y"], off["z"], off["timestamp"]
        step_in = msg.point_step
        npts = msg.width * msg.height
        data_in = msg.data

        # unpackers
        unpack_f = struct.Struct("<f").unpack_from
        pack_f   = struct.Struct("<f").pack_into
        unpack_d = struct.Struct("<d").unpack_from

        # 1) sample timestamps to estimate t0 quickly (full scan still cheap: 25k points)
        t0 = float("inf")
        for i in range(npts):
            base = i * step_in + ts_off
            t = float(unpack_d(data_in, base)[0])
            if t < t0:
                t0 = t

        # 2) build new fields: keep all original fields, append 'time' at the end
        fields_out = list(msg.fields)
        # compute new offsets: append 4 bytes
        time_off = msg.point_step
        fields_out.append(make_pointfield("time", time_off, PointField.FLOAT32, 1))

        step_out = step_in + 4
        data_out = bytearray(npts * step_out)

        # 3) copy point-by-point raw bytes, rotate xyz, write time_rel
        R = self.R
        for i in range(npts):
            base_in = i * step_in
            base_out = i * step_out

            # copy original bytes
            data_out[base_out:base_out + step_in] = data_in[base_in:base_in + step_in]

            # rotate xyz
            x = unpack_f(data_out, base_out + x_off)[0]
            y = unpack_f(data_out, base_out + y_off)[0]
            z = unpack_f(data_out, base_out + z_off)[0]
            xyz2 = R @ np.array([x, y, z], dtype=np.float64)
            pack_f(data_out, base_out + x_off, float(xyz2[0]))
            pack_f(data_out, base_out + y_off, float(xyz2[1]))
            pack_f(data_out, base_out + z_off, float(xyz2[2]))

            # time_rel in seconds
            t = float(unpack_d(data_out, base_out + ts_off)[0])
            time_rel = (t - t0) * self.timestamp_scale
            pack_f(data_out, base_out + time_off, float(time_rel))

        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.output_frame

        out.height = msg.height
        out.width = msg.width
        out.fields = fields_out
        out.is_bigendian = msg.is_bigendian
        out.point_step = step_out
        out.row_step = step_out * out.width
        out.is_dense = msg.is_dense
        out.data = bytes(data_out)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = PclRectifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
