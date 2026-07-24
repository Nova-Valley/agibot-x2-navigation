#!/usr/bin/env python3
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class SaveOneRawPy(Node):
    def __init__(self):
        super().__init__('save_one_image')

        # parameter: image topic
        self.declare_parameter(
            'image_topic', '/aima/hal/sensor/stereo_head_front_left/rgb_image'
        )
        self.topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value

        # save directory
        self.save_dir = Path('images').resolve()
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # state
        self._saved = False
        self._bridge = CvBridge()

        # subscriber (sensor QoS)
        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.image_cb,
            qos_profile_sensor_data
        )
        self.get_logger().info(f'Subscribing to raw image: {self.topic}')
        self.get_logger().info(f'Images will be saved to: {self.save_dir}')

    def image_cb(self, msg: Image):
        # already saved one, ignore later frames
        if self._saved:
            return

        try:
            enc = msg.encoding.lower()
            self.get_logger().info(f'Received image with encoding: {enc}')

            # convert from ROS Image to cv2
            img = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')

            # normalize to BGR for saving
            if enc == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif enc == 'mono8':
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            # if it's bgr8 or other 8-bit bgr that cv2 can save, we just use it

            ts_ms = int(time.time() * 1000)
            out_path = self.save_dir / f'frame_{ts_ms}.png'

            ok = cv2.imwrite(str(out_path), img)
            if ok:
                self.get_logger().info(
                    f'Saved image: {out_path}  ({img.shape[1]}x{img.shape[0]})'
                )
                self._saved = True
                # shut down once we got exactly one frame
                # destroy node first, then shutdown rclpy
                self.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            else:
                self.get_logger().error(f'cv2.imwrite failed: {out_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to decode / save image: {e}')


def main():
    rclpy.init()
    node = SaveOneRawPy()
    rclpy.spin(node)
    # in case the node was already destroyed in the callback
    if rclpy.ok():
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
