#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu


class ImuPrinterPy(Node):
    def __init__(self):
        super().__init__('imu_printer')
        self.declare_parameter('imu_topic', '/aima/hal/imu/chest/state')
        # Can be replaced with other IMU topic names.
        # Waist IMU: /aima/hal/imu/torse/state
        # LiDAR IMU: /aima/hal/sensor/lidar_chest_front/imu
        self.topic = self.get_parameter(
            'imu_topic').get_parameter_value().string_value

        self._last_recv = None
        self.sub = self.create_subscription(
            Imu, self.topic, self.cb, qos_profile_sensor_data
        )
        self.get_logger().info(f'Subscribing IMU: {self.topic}')

    def cb(self, msg: Imu):
        now = time.perf_counter()
        dt_ms = 0.0 if self._last_recv is None else (
            now - self._last_recv) * 1000.0
        self._last_recv = now

        t_sec = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        q = msg.orientation
        w = msg.angular_velocity
        a = msg.linear_acceleration
        oc = msg.orientation_covariance
        wc = msg.angular_velocity_covariance
        ac = msg.linear_acceleration_covariance

        self.get_logger().info(
            (
                f"stamp={t_sec:.6f}s  frame={msg.header.frame_id}  recv_dt={dt_ms:.3f}ms\n"
                f"  orientation (x,y,z,w): [{q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}]\n"
                f"  angular_velocity (rad/s): [{w.x:.6f}, {w.y:.6f}, {w.z:.6f}]\n"
                f"  linear_accel (m/s^2): [{a.x:.6f}, {a.y:.6f}, {a.z:.6f}]\n"
                f"  cov_diag(ori, ang, acc): "
                f"[{oc[0]:.6f},{oc[4]:.6f},{oc[8]:.6f} | "
                f"{wc[0]:.6f},{wc[4]:.6f},{wc[8]:.6f} | "
                f"{ac[0]:.6f},{ac[4]:.6f},{ac[8]:.6f}]"
            )
        )


def main():
    rclpy.init()
    node = ImuPrinterPy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
