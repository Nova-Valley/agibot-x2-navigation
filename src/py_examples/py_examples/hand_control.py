import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import HandCommandArray, HandCommand, HandType, MessageHeader
import time


class HandControl(Node):
    def __init__(self):
        super().__init__('hand_control')

        # Preset parameter list: [(left hand, right hand), ...]
        self.position_pairs = [
            (1.0, 1.0),   # fully open
            (0.0, 0.0),   # fully closed
            (0.5, 0.5),   # half open
            (0.2, 0.8),   # left slightly closed, right more open
            (0.7, 0.3)    # left more open, right slightly closed
        ]
        self.current_index = 0
        self.last_switch_time = self.get_clock().now().nanoseconds / 1e9  # seconds

        # Create publisher
        self.publisher_ = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )

        # 50 Hz timer
        self.timer_ = self.create_timer(
            0.02,  # 20 ms = 50 Hz
            self.publish_hand_commands
        )

        self.get_logger().info("Hand control node started!")

    def publish_hand_commands(self):
        # Check time to decide whether to switch to the next preset
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_switch_time >= 2.0:
            self.current_index = (self.current_index +
                                  1) % len(self.position_pairs)
            self.last_switch_time = now_sec
            self.get_logger().info(
                f"Switched to preset: {self.current_index}, left={self.position_pairs[self.current_index][0]:.2f}, right={self.position_pairs[self.current_index][1]:.2f}"
            )

        # Use current preset
        left_position, right_position = self.position_pairs[self.current_index]

        msg = HandCommandArray()
        msg.header = MessageHeader()

        # Configure left hand
        left_hand = HandCommand()
        left_hand.name = "left_hand"
        left_hand.position = float(left_position)
        left_hand.velocity = 1.0
        left_hand.acceleration = 1.0
        left_hand.deceleration = 1.0
        left_hand.effort = 1.0

        # Configure right hand
        right_hand = HandCommand()
        right_hand.name = "right_hand"
        right_hand.position = float(right_position)
        right_hand.velocity = 1.0
        right_hand.acceleration = 1.0
        right_hand.deceleration = 1.0
        right_hand.effort = 1.0

        msg.left_hand_type = HandType(value=2)  # gripper mode
        msg.right_hand_type = HandType(value=2)
        msg.left_hands = [left_hand]
        msg.right_hands = [right_hand]

        # Publish message
        self.publisher_.publish(msg)
        # We only log when switching presets to avoid too much log output


def main(args=None):
    rclpy.init(args=args)
    hand_control_node = HandControl()

    try:
        rclpy.spin(hand_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        hand_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
