#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import HandCommand, HandCommandArray, HandType, MessageHeader


class OmnihandControl(Node):
    def __init__(self):
        super().__init__('omnihand_control')

        self.num_fingers = 5
        self.joints_per_finger = 2
        self.total_joints = self.num_fingers * self.joints_per_finger  # 10

        # ===== Define 10 gesture sets directly as arrays =====
        # Each set has 20 joint params (left 10 + right 10)
        # 1 = bent, 0 = straight
        self.gesture_sequence = [
            # bend 1 finger
            [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # bend 2 fingers
            [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # bend 3 fingers
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            # bend 4 fingers
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0],
            # bend 5 fingers
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            # release 1 finger
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0],
            # release 2 fingers
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            # release 3 fingers
            [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # release 4 fingers
            [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # all straight
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]

        self.current_index = 0
        self.last_switch_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )

        # Switch gesture every second
        self.timer = self.create_timer(
            1.0,
            self.publish_hand_commands
        )

        self.get_logger().info("Omnihand demo control node started!")

    def publish_hand_commands(self):
        now_time = self.get_clock().now()
        if (now_time - self.last_switch_time).nanoseconds / 1e9 >= 1.0:
            self.current_index = (self.current_index +
                                  1) % len(self.gesture_sequence)
            self.last_switch_time = now_time
            self.get_logger().info(
                f'Switched to gesture {self.current_index + 1}/10')

        joint_array = self.gesture_sequence[self.current_index]

        msg = HandCommandArray()
        msg.header = MessageHeader()

        # Left hand
        for i in range(self.total_joints):
            cmd = HandCommand()
            cmd.name = f"left_hand_joint{i}"
            cmd.position = joint_array[i]
            cmd.velocity = 1.0
            cmd.acceleration = 1.0
            cmd.deceleration = 1.0
            cmd.effort = 1.0
            msg.left_hands.append(cmd)

        # Right hand
        for i in range(self.total_joints):
            cmd = HandCommand()
            cmd.name = f"right_hand_joint{i}"
            cmd.position = joint_array[self.total_joints + i]
            cmd.velocity = 1.0
            cmd.acceleration = 1.0
            cmd.deceleration = 1.0
            cmd.effort = 1.0
            msg.right_hands.append(cmd)

        msg.left_hand_type = HandType()
        msg.left_hand_type.value = 1  # dexterous hand mode
        msg.right_hand_type = HandType()
        msg.right_hand_type.value = 1

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OmnihandControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
