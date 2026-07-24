#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import GetMcAction
from aimdk_msgs.msg import CommonRequest


class GetMcActionClient(Node):
    def __init__(self):
        super().__init__('get_mc_action_client')
        self.client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
        self.get_logger().info('✅ GetMcAction client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self):
        request = GetMcAction.Request()
        request.request = CommonRequest()

        self.get_logger().info('📨 Sending request to get robot mode')
        for i in range(8):
            request.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f'trying ... [{i}]')

        response = future.result()
        if response is None:
            self.get_logger().error('❌ Service call failed or timed out.')
            return

        self.get_logger().info('✅ Robot mode get successfully.')
        self.get_logger().info(f'Mode name: {response.info.action_desc}')
        self.get_logger().info(f'Mode status: {response.info.status.value}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GetMcActionClient()
        node.send_request()
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
