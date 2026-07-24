#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class NavGoalClient(Node):
    def __init__(self):
        super().__init__('nav_goal_client')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(float(yaw))
        goal_msg.pose = pose

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.client.wait_for_server()

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')
        future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"distance_remaining={fb.distance_remaining:.3f}, "
            f"navigation_time={fb.navigation_time.sec}.{fb.navigation_time.nanosec}"
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = NavGoalClient()
    x = 2.0
    y = 1.0
    yaw = 0.0
    node.send_goal(x, y, yaw)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
