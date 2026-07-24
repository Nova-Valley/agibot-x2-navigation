#!/usr/bin/env python3

import sys
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class CmdVelToAimaVelocity(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_aima_velocity')

        # ---------------- parameters ----------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/aima/mc/locomotion/velocity')
        self.declare_parameter('source_name', 'nav2')
        self.declare_parameter('publish_rate', 50.0)   # Hz
        self.declare_parameter('watchdog_timeout', 0.5)  # sec, /cmd_vel 超时后自动清零

        # scale / sign
        self.declare_parameter('forward_scale', 1.0)
        self.declare_parameter('lateral_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)

        # limits from your sample
        self.declare_parameter('max_forward_speed', 1.0)
        self.declare_parameter('min_forward_speed', 0.2)

        self.declare_parameter('max_lateral_speed', 1.0)
        self.declare_parameter('min_lateral_speed', 0.2)

        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_angular_speed', 0.1)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.source_name = self.get_parameter('source_name').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)

        self.forward_scale = float(self.get_parameter('forward_scale').value)
        self.lateral_scale = float(self.get_parameter('lateral_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)

        self.max_forward_speed = float(self.get_parameter('max_forward_speed').value)
        self.min_forward_speed = float(self.get_parameter('min_forward_speed').value)

        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.min_lateral_speed = float(self.get_parameter('min_lateral_speed').value)

        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_angular_speed = float(self.get_parameter('min_angular_speed').value)

        # ---------------- state ----------------
        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_cmd_time = self.get_clock().now()

        # ---------------- ROS interfaces ----------------
        qos = QoSProfile(depth=10)

        self.publisher = self.create_publisher(
            McLocomotionVelocity,
            self.output_topic,
            qos
        )

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos
        )

        self.client = self.create_client(
            SetMcInputSource,
            '/aimdk_5Fmsgs/srv/SetMcInputSource'
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)

        self.get_logger().info('CmdVelToAimaVelocity node started')
        self.get_logger().info(f'cmd_vel_topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'output_topic:  {self.output_topic}')
        self.get_logger().info(f'source_name:   {self.source_name}')

    # ---------------- helper ----------------
    @staticmethod
    def clamp(value: float, max_abs: float) -> float:
        if value > max_abs:
            return max_abs
        if value < -max_abs:
            return -max_abs
        return value

    @staticmethod
    def apply_deadband_and_min(value: float, min_abs: float) -> float:
        """
        与你示例一致：
        abs(value) < 0.005 认为是 0；
        其余若非 0，则至少达到起动阈值 min_abs。
        """
        if abs(value) < 0.005:
            return 0.0
        if 0.0 < abs(value) < min_abs:
            return min_abs if value > 0 else -min_abs
        return value

    def clear_velocity(self):
        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0

    # ---------------- service ----------------
    def register_input_source(self) -> bool:
        self.get_logger().info('Registering input source...')

        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error('Waiting for service timed out')
                return False
            self.get_logger().info('Waiting for input source service...')

        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = self.source_name
        req.input_source.priority = 40
        req.input_source.timeout = 1000

        future = None
        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            self.get_logger().info(f'trying to register input source... [{i}]')

        if future is not None and future.done():
            try:
                response = future.result()
                state = response.response.state.value
                self.get_logger().info(
                    f'Input source set successfully: state={state}, '
                    f'task_id={response.response.task_id}'
                )
                return True
            except Exception as e:
                self.get_logger().error(f'Service call exception: {str(e)}')
                return False
        else:
            self.get_logger().error('Service call failed or timed out')
            return False

    # ---------------- cmd_vel callback ----------------
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        fwd = msg.linear.x * self.forward_scale
        lat = msg.linear.y * self.lateral_scale
        ang = msg.angular.z * self.angular_scale

        # clamp first
        fwd = self.clamp(fwd, self.max_forward_speed)
        lat = self.clamp(lat, self.max_lateral_speed)
        ang = self.clamp(ang, self.max_angular_speed)

        # then apply startup threshold rule
        fwd = self.apply_deadband_and_min(fwd, self.min_forward_speed)
        lat = self.apply_deadband_and_min(lat, self.min_lateral_speed)
        ang = self.apply_deadband_and_min(ang, self.min_angular_speed)

        self.forward_velocity = fwd
        self.lateral_velocity = lat
        self.angular_velocity = ang

    # ---------------- publisher timer ----------------
    def publish_velocity(self):
        # watchdog: 如果 /cmd_vel 太久没更新，自动停
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.watchdog_timeout:
            self.clear_velocity()

        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = now.to_msg()
        msg.source = self.source_name
        msg.forward_velocity = self.forward_velocity
        msg.lateral_velocity = self.lateral_velocity
        msg.angular_velocity = self.angular_velocity

        self.publisher.publish(msg)


global_node = None


def signal_handler(sig, frame):
    global global_node
    if global_node is not None:
        global_node.clear_velocity()
        global_node.publish_velocity()
        global_node.get_logger().info(
            f'Received signal {sig}, clearing velocity and shutting down'
        )
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    global global_node

    rclpy.init(args=args)
    node = CmdVelToAimaVelocity()
    global_node = node

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not node.register_input_source():
        node.get_logger().error('Input source registration failed, exiting')
        rclpy.shutdown()
        return

    node.get_logger().info('Bridge is running: /cmd_vel -> /aima/mc/locomotion/velocity')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
