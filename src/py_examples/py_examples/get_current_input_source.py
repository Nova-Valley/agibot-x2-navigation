#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import GetCurrentInputSource
from aimdk_msgs.msg import CommonRequest


class GetCurrentInputSourceClient(Node):
    def __init__(self):
        super().__init__('get_current_input_source_client')
        self.client = self.create_client(
            GetCurrentInputSource,
            '/aimdk_5Fmsgs/srv/GetCurrentInputSource'
        )

        self.get_logger().info('✅ GetCurrentInputSource client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self):
        # Create request
        req = GetCurrentInputSource.Request()
        req.request = CommonRequest()

        # Send request and wait for response
        self.get_logger().info('📨 Sending request to get current input source')
        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f'trying ... [{i}]')

        if not future.done():
            self.get_logger().error('❌ Service call failed or timed out.')
            return False

        response = future.result()
        ret_code = response.response.header.code
        if ret_code == 0:
            self.get_logger().info(
                f'✅ Current input source get successfully:')
            self.get_logger().info(
                f'Name: {response.input_source.name}')
            self.get_logger().info(
                f'Priority: {response.input_source.priority}')
            self.get_logger().info(
                f'Timeout: {response.input_source.timeout}')
            return True
        else:
            self.get_logger().error(
                f'❌ Current input source get failed, return code: {ret_code}')
            return False


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = GetCurrentInputSourceClient()
        success = node.send_request()
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
