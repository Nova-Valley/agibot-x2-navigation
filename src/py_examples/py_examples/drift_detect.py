#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import numpy as np

class PC2Inspector(Node):
    def __init__(self):
        super().__init__('pc2_inspector')
        self.sub = self.create_subscription(
            PointCloud2, '/lidar/points_rectified', self.cb, 10
        )
        self.done = False

    def cb(self, msg: PointCloud2):
        if self.done:
            return
        # locate offsets
        off = {f.name: f.offset for f in msg.fields}
        if 'ring' not in off or 'timestamp' not in off:
            self.get_logger().error(f"fields={list(off.keys())}")
            self.done = True
            rclpy.shutdown()
            return

        step = msg.point_step
        n = msg.width * msg.height
        buf = msg.data

        rings = np.empty(n, dtype=np.uint16)
        times = np.empty(n, dtype=np.float64)

        for i in range(n):
            base = i * step
            rings[i] = struct.unpack_from('<H', buf, base + off['ring'])[0]   # uint16
            times[i] = struct.unpack_from('<d', buf, base + off['timestamp'])[0]  # float64

        uniq_rings = np.unique(rings)
        tmin, tmax = float(np.min(times)), float(np.max(times))
        self.get_logger().info(f"points={n}")
        self.get_logger().info(f"ring: unique={len(uniq_rings)} min={int(uniq_rings[0])} max={int(uniq_rings[-1])}")
        self.get_logger().info(f"timestamp: min={tmin:.9f} max={tmax:.9f} span={tmax-tmin:.9f}")
        self.get_logger().info(f"stamp(sec)={msg.header.stamp.sec} stamp(nsec)={msg.header.stamp.nanosec}")
        self.done = True
        rclpy.shutdown()

def main():
    rclpy.init()
    node = PC2Inspector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
