#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource
import curses
import time
from functools import partial


class KeyboardVelocityController(Node):
    def __init__(self, stdscr):
        super().__init__('keyboard_velocity_controller')
        self.stdscr = stdscr
        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0
        self.step = 0.2
        self.angular_step = 0.1

        self.publisher = self.create_publisher(
            McLocomotionVelocity, '/aima/mc/locomotion/velocity', 10)
        self.client = self.create_client(
            SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')

        if not self.register_input_source():
            self.get_logger().error("Input source registration failed, exiting")
            raise RuntimeError("Failed to register input source")

        # Configure curses
        curses.cbreak()
        curses.noecho()
        self.stdscr.keypad(True)
        self.stdscr.nodelay(True)

        self.get_logger().info(
            "Control started: W/S forward/backward, A/D strafe, Q/E turn, Space stop, Esc exit")

        # Timer: check keyboard every 50 ms
        self.timer = self.create_timer(0.05, self.check_key_and_publish)

    def register_input_source(self):
        self.get_logger().info("Registering input source...")

        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error("Waiting for service timed out")
                return False
            self.get_logger().info("Waiting for input source service...")

        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = "node"
        req.input_source.priority = 40
        req.input_source.timeout = 1000

        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f"trying to register input source... [{i}]")

        if future.done():
            try:
                resp = future.result()
                state = resp.response.state.value
                self.get_logger().info(
                    f"Input source set successfully: state={state}, task_id={resp.response.task_id}")
                return True
            except Exception as e:
                self.get_logger().error(f"Service exception: {str(e)}")
                return False
        else:
            self.get_logger().error("Service call failed or timed out")
            return False

    def check_key_and_publish(self):
        try:
            ch = self.stdscr.getch()
        except Exception:
            ch = -1

        if ch != -1:
            if ch == ord(' '):
                self.forward_velocity = 0.0
                self.lateral_velocity = 0.0
                self.angular_velocity = 0.0
            elif ch == ord('w'):
                self.forward_velocity = min(
                    self.forward_velocity + self.step, 1.0)
            elif ch == ord('s'):
                self.forward_velocity = max(
                    self.forward_velocity - self.step, -1.0)
            elif ch == ord('a'):
                self.lateral_velocity = min(
                    self.lateral_velocity + self.step, 1.0)
            elif ch == ord('d'):
                self.lateral_velocity = max(
                    self.lateral_velocity - self.step, -1.0)
            elif ch == ord('q'):
                self.angular_velocity = min(
                    self.angular_velocity + self.angular_step, 1.0)
            elif ch == ord('e'):
                self.angular_velocity = max(
                    self.angular_velocity - self.angular_step, -1.0)
            elif ch == 27:  # ESC
                self.get_logger().info("Exiting control")
                rclpy.shutdown()
                return

        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = self.forward_velocity
        msg.lateral_velocity = self.lateral_velocity
        msg.angular_velocity = self.angular_velocity

        self.publisher.publish(msg)

        # Update UI
        self.stdscr.clear()
        self.stdscr.addstr(
            0, 0, "W/S: Forward/Backward | A/D: Strafe | Q/E: Turn | Space: Stop | ESC: Exit")
        self.stdscr.addstr(2, 0,
                           f"Speed Status: Forward: {self.forward_velocity:.2f} m/s | "
                           f"Lateral: {self.lateral_velocity:.2f} m/s | "
                           f"Angular: {self.angular_velocity:.2f} rad/s")
        self.stdscr.refresh()


def curses_main(stdscr):
    rclpy.init()
    try:
        node = KeyboardVelocityController(stdscr)
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger("main").fatal(
            f"Program exited with exception: {e}")
    finally:
        curses.endwin()
        rclpy.shutdown()


def main():
    curses.wrapper(curses_main)


if __name__ == '__main__':
    main()
