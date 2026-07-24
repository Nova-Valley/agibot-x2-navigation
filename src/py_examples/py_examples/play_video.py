#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import PlayVideo


class PlayVideoClient(Node):
    def __init__(self):
        super().__init__('play_video_client')
        self.client = self.create_client(
            PlayVideo, '/face_ui_proxy/play_video')
        self.get_logger().info('✅ PlayVideo client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, video_path, mode, priority):
        req = PlayVideo.Request()

        req.video_path = video_path
        req.mode = mode
        req.priority = priority

        # async call
        self.get_logger().info(
            f'📨 Sending request to play video: mode={mode} video={video_path}')
        for i in range(8):
            req.header.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f'trying ... [{i}]')

        resp = future.result()
        if resp is None:
            self.get_logger().error('❌ Service call not completed or timed out.')
            return False

        if resp.success:
            self.get_logger().info(
                f'✅ Request to play video recorded successfully: {resp.message}')
            return True
        else:
            self.get_logger().error(
                f'❌ Failed to record play-video request: {resp.message}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        # video path and priority can be customized
        video_path = "/agibot/data/home/agi/zhiyuan.mp4"
        priority = 5
        # input play mode
        mode = int(input("Enter video play mode (1: play once, 2: loop): "))
        if mode not in (1, 2):
            raise ValueError(f'invalid mode {mode}')

        node = PlayVideoClient()
        node.send_request(video_path, mode, priority)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'Program exited with exception: {e}')

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
