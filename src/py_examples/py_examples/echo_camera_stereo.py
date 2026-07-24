#!/usr/bin/env python3
"""
Head stereo camera multi-topic subscription example

Supports selecting the topic type to subscribe via startup parameter --ros-args -p topic_type:=<type>:
  - left_rgb_image: Left camera RGB image (sensor_msgs/Image)
  - left_rgb_image_compressed: Left camera RGB compressed image (sensor_msgs/CompressedImage)
  - left_camera_info: Left camera intrinsic parameters (sensor_msgs/CameraInfo)
  - right_rgb_image: Right camera RGB image (sensor_msgs/Image)
  - right_rgb_image_compressed: Right camera RGB compressed image (sensor_msgs/CompressedImage)
  - right_camera_info: Right camera intrinsic parameters (sensor_msgs/CameraInfo)

Example:
  ros2 run py_examples echo_camera_stereo --ros-args -p topic_type:=left_rgb_image
  ros2 run py_examples echo_camera_stereo --ros-args -p topic_type:=right_rgb_image
  ros2 run py_examples echo_camera_stereo --ros-args -p topic_type:=left_camera_info

Default topic_type is left_rgb_image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from collections import deque
import time


class StereoCameraTopicEcho(Node):
    def __init__(self):
        super().__init__('stereo_camera_topic_echo')

        # Select the topic type to subscribe
        self.declare_parameter('topic_type', 'left_rgb_image')
        self.declare_parameter('dump_video_path', '')

        self.topic_type = self.get_parameter('topic_type').value
        self.dump_video_path = self.get_parameter('dump_video_path').value

        # Set QoS parameters - use sensor data QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Create different subscribers based on topic_type
        if self.topic_type == "left_rgb_image":
            self.topic_name = "/aima/hal/sensor/stereo_head_front_left/rgb_image"
            self.sub_image = self.create_subscription(
                Image, self.topic_name, self.cb_image, qos)
            self.get_logger().info(
                f"✅ Subscribing Left RGB Image: {self.topic_name}")
            if self.dump_video_path:
                self.get_logger().info(
                    f"📝 Will dump received images to video: {self.dump_video_path}")

        elif self.topic_type == "left_rgb_image_compressed":
            self.topic_name = "/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed"
            self.sub_compressed = self.create_subscription(
                CompressedImage, self.topic_name, self.cb_compressed, qos)
            self.get_logger().info(
                f"✅ Subscribing Left CompressedImage: {self.topic_name}")

        elif self.topic_type == "left_camera_info":
            self.topic_name = "/aima/hal/sensor/stereo_head_front_left/camera_info"
            # CameraInfo subscription must use reliable + transient_local QoS to receive historical messages (even if only one frame is published)
            camera_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
            self.sub_camerainfo = self.create_subscription(
                CameraInfo, self.topic_name, self.cb_camerainfo, camera_qos)
            self.get_logger().info(
                f"✅ Subscribing Left CameraInfo (with transient_local): {self.topic_name}")

        elif self.topic_type == "right_rgb_image":
            self.topic_name = "/aima/hal/sensor/stereo_head_front_right/rgb_image"
            self.sub_image = self.create_subscription(
                Image, self.topic_name, self.cb_image, qos)
            self.get_logger().info(
                f"✅ Subscribing Right RGB Image: {self.topic_name}")
            if self.dump_video_path:
                self.get_logger().info(
                    f"📝 Will dump received images to video: {self.dump_video_path}")

        elif self.topic_type == "right_rgb_image_compressed":
            self.topic_name = "/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed"
            self.sub_compressed = self.create_subscription(
                CompressedImage, self.topic_name, self.cb_compressed, qos)
            self.get_logger().info(
                f"✅ Subscribing Right CompressedImage: {self.topic_name}")

        elif self.topic_type == "right_camera_info":
            self.topic_name = "/aima/hal/sensor/stereo_head_front_right/camera_info"
            # CameraInfo subscription must use reliable + transient_local QoS to receive historical messages (even if only one frame is published)
            camera_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
            self.sub_camerainfo = self.create_subscription(
                CameraInfo, self.topic_name, self.cb_camerainfo, camera_qos)
            self.get_logger().info(
                f"✅ Subscribing Right CameraInfo (with transient_local): {self.topic_name}")

        else:
            self.get_logger().error(f"Unknown topic_type: {self.topic_type}")
            raise ValueError("Unknown topic_type")

        # Internal state
        self.last_print = self.get_clock().now()
        self.arrivals = deque()

    def update_arrivals(self):
        """Calculate received FPS"""
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

    def get_fps(self):
        """Get FPS"""
        return len(self.arrivals)

    def should_print(self):
        """Control print frequency"""
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds * 1e-9 >= 1.0:
            self.last_print = now
            return True
        return False

    def cb_image(self, msg: Image):
        """Image callback (Left/Right camera RGB image)"""
        self.update_arrivals()

        if self.should_print():
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"📸 {self.topic_type} received\n"
                f"  • frame_id:        {msg.header.frame_id}\n"
                f"  • stamp (sec):     {stamp_sec:.6f}\n"
                f"  • encoding:        {msg.encoding}\n"
                f"  • size (WxH):      {msg.width} x {msg.height}\n"
                f"  • step (bytes/row):{msg.step}\n"
                f"  • is_bigendian:    {msg.is_bigendian}\n"
                f"  • recv FPS (1s):   {self.get_fps():.1f}"
            )

        # Only RGB images support video dump
        if (self.topic_type in ["left_rgb_image", "right_rgb_image"]) and self.dump_video_path:
            self.dump_image_to_video(msg)

    def cb_compressed(self, msg: CompressedImage):
        """CompressedImage callback (Left/Right camera RGB compressed image)"""
        self.update_arrivals()

        if self.should_print():
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"🗜️  {self.topic_type} received\n"
                f"  • frame_id:        {msg.header.frame_id}\n"
                f"  • stamp (sec):     {stamp_sec:.6f}\n"
                f"  • format:          {msg.format}\n"
                f"  • data size:       {len(msg.data)}\n"
                f"  • recv FPS (1s):   {self.get_fps():.1f}"
            )

    def cb_camerainfo(self, msg: CameraInfo):
        """CameraInfo callback (Left/Right camera intrinsic parameters)"""
        # Camera info will only receive one frame, print it directly
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Format D array
        d_str = ", ".join([f"{d:.6f}" for d in msg.d])

        # Format K matrix
        k_str = ", ".join([f"{k:.6f}" for k in msg.k])

        # Format P matrix
        p_str = ", ".join([f"{p:.6f}" for p in msg.p])

        self.get_logger().info(
            f"📷 {self.topic_type} received\n"
            f"  • frame_id:        {msg.header.frame_id}\n"
            f"  • stamp (sec):     {stamp_sec:.6f}\n"
            f"  • width x height:  {msg.width} x {msg.height}\n"
            f"  • distortion_model:{msg.distortion_model}\n"
            f"  • D: [{d_str}]\n"
            f"  • K: [{k_str}]\n"
            f"  • P: [{p_str}]\n"
            f"  • binning_x: {msg.binning_x}\n"
            f"  • binning_y: {msg.binning_y}\n"
            f"  • roi: {{ x_offset: {msg.roi.x_offset}, y_offset: {msg.roi.y_offset}, height: {msg.roi.height}, width: {msg.roi.width}, do_rectify: {msg.roi.do_rectify} }}"
        )

    def dump_image_to_video(self, msg: Image):
        """Video dump is only supported for RGB images"""
        # You can add video recording functionality here
        # Simplified in the Python version, only logs instead
        if self.should_print():
            self.get_logger().info(f"📝 Video dump not implemented in Python version")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StereoCameraTopicEcho()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
