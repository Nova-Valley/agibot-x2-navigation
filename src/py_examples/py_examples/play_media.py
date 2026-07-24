#!/usr/bin/env python3

import sys
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import PlayMediaFile
from aimdk_msgs.msg import TtsPriorityLevel


class PlayMediaClient(Node):
    def __init__(self):
        super().__init__('play_media_client')
        self.client = self.create_client(
            PlayMediaFile, '/aimdk_5Fmsgs/srv/PlayMediaFile')
        self.get_logger().info('✅ PlayMedia client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, media_path):
        req = PlayMediaFile.Request()

        req.media_file_req.file_name = media_path
        req.media_file_req.domain = 'demo_client'       # required: caller domain
        req.media_file_req.trace_id = 'demo'            # optional
        req.media_file_req.is_interrupted = True        # interrupt same-priority
        req.media_file_req.priority_weight = 0          # optional: 0~99
        req.media_file_req.priority_level.value = TtsPriorityLevel.INTERACTION_L6

        self.get_logger().info(
            f'📨 Sending request to play media: {media_path}')
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

        if resp.tts_resp.is_success:
            self.get_logger().info('✅ Request to play media file recorded successfully.')
            return True
        else:
            self.get_logger().error('❌ Failed to record play-media request.')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None

    default_media = '/agibot/data/var/interaction/tts_cache/normal/demo.wav'
    try:
        if len(sys.argv) > 1:
            media_path = sys.argv[1]
        else:
            media_path = input(
                f'Enter media file path to play (default: {default_media}): ').strip()
            if not media_path:
                media_path = default_media

        node = PlayMediaClient()
        node.send_request(media_path)
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
