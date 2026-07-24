#!/usr/bin/env python3
import os
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PCDToCloudPublisher(Node):
    def __init__(self):
        super().__init__('pcd_to_cloud_map_publisher')

        self.declare_parameter('pcd_path', '')
        self.declare_parameter('topic_name', '/cloud_pcd')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('use_latched_qos', False)

        self.pcd_path = self.get_parameter('pcd_path').value
        self.topic_name = self.get_parameter('topic_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        if not self.pcd_path:
            raise ValueError('Parameter "pcd_path" is empty.')

        if not os.path.exists(self.pcd_path):
            raise FileNotFoundError(f'PCD file does not exist: {self.pcd_path}')

        self.points_xyz = self.load_pcd_xyz(self.pcd_path)

        self.pub = self.create_publisher(PointCloud2, self.topic_name, 10)

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_once)

        self.get_logger().info(
            f'Loaded {self.points_xyz.shape[0]} xyz points from {self.pcd_path}'
        )
        self.get_logger().info(
            f'Publishing to {self.topic_name} with frame_id="{self.frame_id}" '
            f'at {self.publish_rate:.2f} Hz'
        )

    def load_pcd_xyz(self, pcd_path: str) -> np.ndarray:
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points, dtype=np.float32)

        if points.size == 0:
            raise ValueError(f'No valid points found in PCD: {pcd_path}')

        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]

        if points.size == 0:
            raise ValueError(f'All points are invalid after filtering: {pcd_path}')

        return points

    def publish_once(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        cloud_msg = point_cloud2.create_cloud_xyz32(
            header,
            self.points_xyz.tolist()
        )

        self.pub.publish(cloud_msg)

        self.get_logger().info(
            f'Published {self.points_xyz.shape[0]} points on {self.topic_name} '
            f'in frame {self.frame_id}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PCDToCloudPublisher()
        rclpy.spin(node)
    except Exception as e:
        print(f'[ERROR] {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
