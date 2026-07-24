#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data  # ✅ match camera QoS


def bgr8_to_ros_image(img_bgr: np.ndarray, header) -> Image:
    """OpenCV BGR uint8 -> ROS2 sensor_msgs/Image (no cv_bridge)."""
    if img_bgr is None:
        raise ValueError("img_bgr is None")
    if img_bgr.dtype != np.uint8:
        raise ValueError(f"Expected uint8 image, got {img_bgr.dtype}")
    if img_bgr.ndim != 3 or img_bgr.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 BGR image, got shape={img_bgr.shape}")

    h, w = img_bgr.shape[:2]
    msg = Image()
    msg.header = header
    msg.height = int(h)
    msg.width = int(w)
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(w * 3)
    msg.data = img_bgr.tobytes()
    return msg


class C2INode(Node):
    def __init__(self, in_topic: str):
        super().__init__("compressed_to_image")

        out_topic = in_topic.replace("compressed", "raw_modified")
        self.pub = self.create_publisher(Image, out_topic, 10)

        # ✅ use sensor QoS to match camera publishers
        self.sub = self.create_subscription(
            CompressedImage,
            in_topic,
            self.callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f"✅ Sub: {in_topic}")
        self.get_logger().info(f"✅ Pub: {out_topic}")

    def callback(self, msg: CompressedImage):
        # msg.format often like: "jpeg" / "png" / "jpeg compressed bgr8"
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # returns BGR uint8
        if img is None:
            self.get_logger().warn("cv2.imdecode returned None")
            return

        try:
            img_msg = bgr8_to_ros_image(img, msg.header)
        except Exception as e:
            self.get_logger().error(f"convert to Image failed: {e}")
            return

        self.pub.publish(img_msg)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 compressed_to_raw.py /your/compressed/topic")
        return

    in_topic = sys.argv[1]

    rclpy.init()
    node = C2INode(in_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
