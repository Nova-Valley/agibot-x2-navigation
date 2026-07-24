#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import PlayEmoji


class PlayEmojiClient(Node):
    def __init__(self):
        super().__init__('play_emoji_client')
        self.client = self.create_client(
            PlayEmoji, '/face_ui_proxy/play_emoji')
        self.get_logger().info('✅ PlayEmoji client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, emoji: int, mode: int, priority: int):
        req = PlayEmoji.Request()

        req.emotion_id = int(emoji)
        req.mode = int(mode)
        req.priority = int(priority)

        self.get_logger().info(
            f'📨 Sending request to play emoji: id={req.emotion_id}, mode={req.mode}, priority={req.priority}')
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
                f'✅ Emoji played successfully: {resp.message}')
            return True
        else:
            self.get_logger().error(f'❌ Failed to play emoji: {resp.message}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None

    # Interactive input, same as the original C++ version
    try:
        emotion = int(
            input("Enter emoji ID: 1-blink, 60-bored, 70-abnormal, 80-sleeping, 90-happy ... 190-double angry, 200-adore: "))
        mode = int(input("Enter play mode (1: play once, 2: loop): "))
        if mode not in (1, 2):
            raise ValueError("invalid mode")
        priority = 10  # default priority

        node = PlayEmojiClient()
        node.send_request(emotion, mode, priority)
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
