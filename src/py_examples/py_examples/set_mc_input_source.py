#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import SetMcInputSource
from aimdk_msgs.msg import RequestHeader, McInputAction


class McInputClient(Node):
    def __init__(self):
        super().__init__('set_mc_input_source_client')
        self.client = self.create_client(
            SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource'
        )

        self.get_logger().info('✅ SetMcInputSource client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self):
        req = SetMcInputSource.Request()

        # header
        req.request.header = RequestHeader()

        # action (e.g. 1001 = register)
        req.action = McInputAction()
        req.action.value = 1001

        # input source info
        req.input_source.name = 'node'
        req.input_source.priority = 40
        req.input_source.timeout = 1000  # ms

        # Send request and wait for response
        self.get_logger().info(
            f'📨 Sending input source request: action_id={req.action.value}, '
            f'name={req.input_source.name}, priority={req.input_source.priority}'
        )
        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=0.25)

            if future.done():
                break

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f'trying ... [{i}]')

        if not future.done():
            self.get_logger().error('❌ Service call failed or timed out.')
            return False

        response = future.result()
        ret_code = response.response.header.code
        task_id = response.response.task_id

        if ret_code == 0:
            self.get_logger().info(
                f'✅ Input source set successfully. task_id={task_id}'
            )
            return True
        else:
            self.get_logger().error(
                f'❌ Input source set failed. ret_code={ret_code}, task_id={task_id} (duplicated ADD? or MODIFY/ENABLE/DISABLE for unknown source?)'
            )
            return False


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = McInputClient()
        ok = node.send_request()
        if not ok:
            node.get_logger().error('Input source request failed.')
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
