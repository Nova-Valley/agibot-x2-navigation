#!/usr/bin/env python3

import sys
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import CommonRequest
from aimdk_msgs.srv import LedStripCommand


class PlayLightsClient(Node):
    def __init__(self):
        super().__init__('play_lights_client')

        # create service client
        self.client = self.create_client(
            LedStripCommand, '/aimdk_5Fmsgs/srv/LedStripCommand')

        self.get_logger().info('✅ PlayLights client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, led_mode, r, g, b):
        """Send LED control request"""
        # create request
        request = LedStripCommand.Request()
        request.led_strip_mode = led_mode
        request.r = r
        request.g = g
        request.b = b

        # send request
        # Note: LED strip is slow to response (up to ~5s)
        self.get_logger().info(
            f'📨 Sending request to control led strip: mode={led_mode}, RGB=({r}, {g}, {b})')
        for i in range(4):
            request.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5)

            if future.done():
                break

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f'trying ... [{i}]')

        response = future.result()
        if response is None:
            self.get_logger().error('❌ Service call not completed or timed out.')
            return False

        if response.status_code == 0:
            self.get_logger().info('✅ LED strip command sent successfully.')
            return True
        else:
            self.get_logger().error(
                f'❌ LED strip command failed with status: {response.status_code}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        # get command line args
        if len(sys.argv) > 4:
            # use CLI args
            led_mode = int(sys.argv[1])
            if led_mode not in (0, 1, 2, 3):
                raise ValueError("invalid mode")
            r = int(sys.argv[2])
            if r < 0 or r > 255:
                raise ValueError("invalid R value")
            g = int(sys.argv[3])
            if g < 0 or g > 255:
                raise ValueError("invalid G value")
            b = int(sys.argv[4])
            if b < 0 or b > 255:
                raise ValueError("invalid B value")
        else:
            # interactive input
            print("=== LED strip control example ===")
            print("Select LED strip mode:")
            print("0 - Steady on")
            print("1 - Breathing (4s cycle, sine brightness)")
            print("2 - Blinking (1s cycle, 0.5s on, 0.5s off)")
            print("3 - Flowing (2s cycle, light up from left to right)")

            led_mode = int(input("Enter mode (0-3): "))
            if led_mode not in (0, 1, 2, 3):
                raise ValueError("invalid mode")

            print("\nSet RGB color values (0-255):")
            r = int(input("Red (R): "))
            if r < 0 or r > 255:
                raise ValueError("invalid R value")
            g = int(input("Green (G): "))
            if g < 0 or g > 255:
                raise ValueError("invalid G value")
            b = int(input("Blue (B): "))
            if b < 0 or b > 255:
                raise ValueError("invalid B value")

        node = PlayLightsClient()
        node.send_request(led_mode, r, g, b)
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
