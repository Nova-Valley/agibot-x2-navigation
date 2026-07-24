#!/usr/bin/env python3
"""
Robot joint control example
This script implements a ROS2-based robot joint control system, using the Ruckig trajectory
planner to achieve smooth joint motion control.

Main features:
1. Supports controlling multiple joint areas (head, arm, waist, leg)
2. Uses Ruckig for trajectory planning to ensure smooth motion
3. Supports real-time control of joint position, velocity, and acceleration
4. Provides joint limit and PID (stiffness/damping) parameter configuration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from aimdk_msgs.msg import JointCommandArray, JointStateArray, JointCommand
from std_msgs.msg import Header
import ruckig
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict
from threading import Lock

# QoS config: define ROS2 Quality of Service parameters
# Subscriber QoS: best-effort reliability, keep last 10 messages
subscriber_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

# Publisher QoS: reliable transport, keep last 10 messages
publisher_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)


class JointArea(Enum):
    HEAD = 'HEAD'    # Head joints
    ARM = 'ARM'      # Arm joints
    WAIST = 'WAIST'  # Waist joints
    LEG = 'LEG'      # Leg joints


@dataclass
class JointInfo:
    # Joint information data class
    name: str           # Joint name
    lower_limit: float  # Joint lower angle limit
    upper_limit: float  # Joint upper angle limit
    kp: float           # Position control proportional gain
    kd: float           # Velocity control derivative gain


# Robot model configuration: define all joint parameters
robot_model: Dict[JointArea, List[JointInfo]] = {
    # Leg joint configuration
    JointArea.LEG: [
        # Left leg joints
        JointInfo("left_hip_pitch_joint", -2.4871,
                  2.4871, 40.0, 4.0),    # left hip pitch
        JointInfo("left_hip_roll_joint", -0.12217,
                  2.9059, 40.0, 4.0),    # left hip roll
        JointInfo("left_hip_yaw_joint", -1.6842,
                  3.4296, 30.0, 3.0),    # left hip yaw
        JointInfo("left_knee_joint", 0.026179,
                  2.1206, 80.0, 8.0),    # left knee
        JointInfo("left_ankle_pitch_joint", -0.80285,
                  0.45378, 40.0, 4.0),   # left ankle pitch
        JointInfo("left_ankle_roll_joint", -0.2618,
                  0.2618, 20.0, 2.0),    # left ankle roll
        # Right leg joints
        JointInfo("right_hip_pitch_joint", -2.4871,
                  2.4871, 40.0, 4.0),    # right hip pitch
        JointInfo("right_hip_roll_joint", -2.9059,
                  0.12217, 40.0, 4.0),   # right hip roll
        JointInfo("right_hip_yaw_joint", -3.4296,
                  1.6842, 30.0, 3.0),    # right hip yaw
        JointInfo("right_knee_joint", 0.026179,
                  2.1206, 80.0, 8.0),    # right knee
        JointInfo("right_ankle_pitch_joint", -0.80285,
                  0.45378, 40.0, 4.0),   # right ankle pitch
        JointInfo("right_ankle_roll_joint", -0.2618,
                  0.2618, 20.0, 2.0),    # right ankle roll
    ],
    # Waist joint configuration
    JointArea.WAIST: [
        JointInfo("waist_yaw_joint", -3.4296,
                  2.3824, 20.0, 4.0),     # waist yaw
        JointInfo("waist_pitch_joint", -0.17453,
                  0.17453, 20.0, 4.0),  # waist pitch
        JointInfo("waist_roll_joint", -0.48869,
                  0.48869, 20.0, 4.0),   # waist roll
    ],
    # Arm joint configuration
    JointArea.ARM: [
        # Left arm
        JointInfo("left_shoulder_pitch_joint", -
                  2.5569, 2.5569, 20.0, 2.0),  # left shoulder pitch
        JointInfo("left_shoulder_roll_joint", -
                  0.06108, 3.3598, 20.0, 2.0),  # left shoulder roll
        JointInfo("left_shoulder_yaw_joint", -
                  2.5569, 2.5569, 20.0, 2.0),   # left shoulder yaw
        JointInfo("left_elbow_pitch_joint", -2.4435,
                  0.0, 20.0, 2.0),              # left elbow pitch
        JointInfo("left_elbow_roll_joint", -2.5569,
                  2.5569, 20.0, 2.0),           # left elbow roll
        JointInfo("left_wrist_pitch_joint", -0.5585,
                  0.5585, 20.0, 2.0),           # left wrist pitch
        JointInfo("left_wrist_yaw_joint", -1.5446,
                  1.5446, 20.0, 2.0),           # left wrist yaw
        # Right arm
        JointInfo("right_shoulder_pitch_joint", -
                  2.5569, 2.5569, 20.0, 2.0),   # right shoulder pitch
        JointInfo("right_shoulder_roll_joint", -
                  3.3598, 0.06108, 20.0, 2.0),  # right shoulder roll
        JointInfo("right_shoulder_yaw_joint", -
                  2.5569, 2.5569, 20.0, 2.0),   # right shoulder yaw
        JointInfo("right_elbow_pitch_joint", 0.0,
                  2.4435, 20.0, 2.0),           # right elbow pitch
        JointInfo("right_elbow_roll_joint", -2.5569,
                  2.5569, 20.0, 2.0),           # right elbow roll
        JointInfo("right_wrist_pitch_joint", -
                  0.5585, 0.5585, 20.0, 2.0),   # right wrist pitch
        JointInfo("right_wrist_yaw_joint", -1.5446,
                  1.5446, 20.0, 2.0),           # right wrist yaw
    ],
    # Head joint configuration
    JointArea.HEAD: [
        JointInfo("head_pitch_joint", -0.61087,
                  0.61087, 20.0, 2.0),  # head pitch
        JointInfo("head_yaw_joint", -0.61087,
                  0.61087, 20.0, 2.0),    # head yaw
    ],
}


class JointControllerNode(Node):
    """
    Joint controller node
    Responsible for receiving joint states, using Ruckig for trajectory planning,
    and publishing joint commands.
    """

    def __init__(self, node_name: str, sub_topic: str, pub_topic: str, area: JointArea, dofs: int):
        """
        Initialize joint controller
        Args:
            node_name: node name
            sub_topic: topic name to subscribe (joint states)
            pub_topic: topic name to publish (joint commands)
            area: joint area (head/arm/waist/leg)
            dofs: number of DOFs
        """
        super().__init__(node_name)
        self.lock = Lock()
        self.joint_info = robot_model[area]
        self.dofs = dofs
        self.ruckig = ruckig.Ruckig(dofs, 0.002)  # 2 ms control period
        self.input = ruckig.InputParameter(dofs)
        self.output = ruckig.OutputParameter(dofs)
        self.ruckig_initialized = False

        # Initialize trajectory parameters
        self.input.current_position = [0.0] * dofs
        self.input.current_velocity = [0.0] * dofs
        self.input.current_acceleration = [0.0] * dofs

        # Motion limits
        self.input.max_velocity = [1.0] * dofs
        self.input.max_acceleration = [1.0] * dofs
        self.input.max_jerk = [25.0] * dofs

        # ROS2 subscriber and publisher
        self.sub = self.create_subscription(
            JointStateArray,
            sub_topic,
            self.joint_state_callback,
            subscriber_qos
        )
        self.pub = self.create_publisher(
            JointCommandArray,
            pub_topic,
            publisher_qos
        )

    def joint_state_callback(self, msg: JointStateArray):
        """
        Joint state callback
        Receives and processes joint state messages
        """
        self.ruckig_initialized = True

    def control_callback(self, joint_idx):
        """
        Control callback
        Uses Ruckig for trajectory planning and publishes control commands
        Args:
            joint_idx: target joint index
        """
        # Run Ruckig until the target is reached
        while self.ruckig.update(self.input, self.output) in [ruckig.Result.Working, ruckig.Result.Finished]:
            # Update current state
            self.input.current_position = self.output.new_position
            self.input.current_velocity = self.output.new_velocity
            self.input.current_acceleration = self.output.new_acceleration

            # Check if target is reached
            tolerance = 1e-6
            current_p = self.output.new_position[joint_idx]
            if abs(current_p - self.input.target_position[joint_idx]) < tolerance:
                break

            # Create and publish command
            cmd = JointCommandArray()
            for i, joint in enumerate(self.joint_info):
                j = JointCommand()
                j.name = joint.name
                j.position = self.output.new_position[i]
                j.velocity = self.output.new_velocity[i]
                j.effort = 0.0
                j.stiffness = joint.kp
                j.damping = joint.kd
                cmd.joints.append(j)

            self.pub.publish(cmd)

    def set_target_position(self, joint_name, position):
        """
        Set target joint position
        Args:
            joint_name: joint name
            position: target position
        """
        p_s = [0.0] * self.dofs
        joint_idx = 0
        for i, joint in enumerate(self.joint_info):
            if joint.name == joint_name:
                p_s[i] = position
                joint_idx = i
        self.input.target_position = p_s
        self.input.target_velocity = [0.0] * self.dofs
        self.input.target_acceleration = [0.0] * self.dofs
        self.control_callback(joint_idx)


def main(args=None):
    """
    Main function
    Initialize ROS2 node and start joint controller
    """
    rclpy.init(args=args)

    # Create leg controller node
    #leg_node = JointControllerNode(
    #    "leg_node",
    #    "/aima/hal/joint/leg/state",
    #    "/aima/hal/joint/leg/command",
    #    JointArea.LEG,
    #    12
    #)

    # waist_node = JointControllerNode(
    #     "waist_node",
    #     "/aima/hal/joint/waist/state",
    #     "/aima/hal/joint/waist/command",
    #     JointArea.WAIST,
    #     3
    # )

    arm_node = JointControllerNode(
         "arm_node",
         "/aima/hal/joint/arm/state",
         "/aima/hal/joint/arm/command",
         JointArea.ARM,
         14
    )


    position = 0.5

    # Only control the left leg joint. If you want to control a specific joint, assign it directly.
    def timer_callback():
        """
        Timer callback
        Periodically change target position to achieve oscillating motion
        """
        nonlocal position
        position = -0.1
        #position = 1.0 + position
        arm_node.set_target_position("left_shoulder_pitch_joint", position)

    arm_node.create_timer(3.0, timer_callback)

    # Multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(arm_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        arm_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
