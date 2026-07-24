#!/usr/bin/env python3

import sys
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import PlayTts
from aimdk_msgs.msg import TtsPriorityLevel


class PlayTTSClient(Node):
    def __init__(self):
        super().__init__('play_tts_client')

        # fill in the actual service name
        self.client = self.create_client(PlayTts, '/aimdk_5Fmsgs/srv/PlayTts')
        self.get_logger().info('✅ PlayTts client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, text):
        req = PlayTts.Request()

        req.tts_req.text = text
        req.tts_req.domain = 'demo_client'   # required: caller domain
        req.tts_req.trace_id = 'demo'        # optional: request id
        req.tts_req.is_interrupted = True    # required: interrupt same-priority
        req.tts_req.priority_weight = 0
        req.tts_req.priority_level.value = 6

        self.get_logger().info(f'📨 Sending request to play tts: text={text}')
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
            self.get_logger().info('✅ TTS sent successfully.')
            return True
        else:
            self.get_logger().error('❌ Failed to send TTS.')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        # get text to speak
        if len(sys.argv) > 1:
            text = sys.argv[1]
        else:
            text = input('Enter text to speak: ')
            if not text:
                text = 'Hello, I am AgiBot X2.'

        node = PlayTTSClient()
        node.send_request(text)
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
