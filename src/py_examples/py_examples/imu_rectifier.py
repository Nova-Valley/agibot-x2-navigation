#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)
from sensor_msgs.msg import Imu


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


class ImuRectifier(Node):
    def __init__(self):
        super().__init__("imu_rectifier")

        # ========= 你只改这里（先试 pitch=-90）=========
        self.in_topic  = "/aima/hal/sensor/lidar_chest_front/imu"
        self.out_topic = "/imu/rectified"
        self.out_frame = "lidar_imu_chest_front"

        self.roll_deg  = 0.0
        self.pitch_deg = -90.0   # 关键：把 g 从 x 转到 z（先试 -90）
        self.yaw_deg   = 0.0
        # ==============================================

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
        
        self.pub = self.create_publisher(Imu, self.out_topic, qos_out)
        self.sub = self.create_subscription(Imu, self.in_topic, self.cb, qos_in)

        self.get_logger().info(f"IMU rectifier: {self.in_topic} -> {self.out_topic}")
        self.get_logger().info(f"RPY(deg): roll={self.roll_deg}, pitch={self.pitch_deg}, yaw={self.yaw_deg}")

    def cb(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.out_frame

        # rotate angular_velocity and linear_acceleration
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float64)
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=np.float64)

        w2 = self.R @ w
        a2 = self.R @ a

        out.angular_velocity.x = float(w2[0])
        out.angular_velocity.y = float(w2[1])
        out.angular_velocity.z = float(w2[2])

        out.linear_acceleration.x = float(a2[0])
        out.linear_acceleration.y = float(a2[1])
        out.linear_acceleration.z = float(a2[2])

        # copy covariances (keep as-is)
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.orientation_covariance = msg.orientation_covariance

        # keep orientation identity (FAST-LIO typically ignores it)
        out.orientation = msg.orientation

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuRectifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
