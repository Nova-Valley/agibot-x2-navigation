#!/usr/bin/env python3
"""
Single-joint override on top of a balance controller that publishes the SAME command topic.

Design:
- Subscribe to /aima/hal/joint/leg/command as baseline (balance node output)
- When override is active, publish a merged JointCommandArray to the SAME topic:
    * All joints follow baseline EXCEPT one joint, which follows a Ruckig trajectory
    * Hold at target for hold_sec seconds
- When override is inactive, this node publishes nothing (balance node fully controls)

Important:
- Because we publish to the same topic we subscribe, we filter out our own echoed messages
  by checking whether the override joint position is nearly equal to our last published value.
"""

import time
from dataclasses import dataclass
from enum import Enum
from threading import Lock
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import ruckig
from aimdk_msgs.msg import JointCommandArray, JointCommand


# ---------------- QoS ----------------
subscriber_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

publisher_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)


# ---------------- Robot model (LEG only here; extend if you want) ----------------
class JointArea(Enum):
    LEG = "LEG"


@dataclass
class JointInfo:
    name: str
    lower_limit: float
    upper_limit: float
    kp: float
    kd: float


robot_model: Dict[JointArea, List[JointInfo]] = {
    JointArea.LEG: [
        JointInfo("left_hip_pitch_joint",  -2.4871,   2.4871,  40.0, 4.0),
        JointInfo("left_hip_roll_joint",   -0.12217,  2.9059,  40.0, 4.0),
        JointInfo("left_hip_yaw_joint",    -1.6842,   3.4296,  30.0, 3.0),
        JointInfo("left_knee_joint",        0.026179, 2.1206,  80.0, 8.0),
        JointInfo("left_ankle_pitch_joint",-0.80285,  0.45378, 40.0, 4.0),
        JointInfo("left_ankle_roll_joint", -0.2618,   0.2618,  20.0, 2.0),

        JointInfo("right_hip_pitch_joint",  -2.4871,   2.4871,  40.0, 4.0),
        JointInfo("right_hip_roll_joint",   -2.9059,   0.12217, 40.0, 4.0),
        JointInfo("right_hip_yaw_joint",    -3.4296,   1.6842,  30.0, 3.0),
        JointInfo("right_knee_joint",        0.026179, 2.1206,  80.0, 8.0),
        JointInfo("right_ankle_pitch_joint",-0.80285,  0.45378, 40.0, 4.0),
        JointInfo("right_ankle_roll_joint", -0.2618,   0.2618,  20.0, 2.0),
    ]
}


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def copy_joint_command(src: JointCommand) -> JointCommand:
    dst = JointCommand()
    dst.name = src.name
    dst.position = float(src.position)
    dst.velocity = float(src.velocity)
    dst.effort = float(src.effort)
    dst.stiffness = float(src.stiffness)
    dst.damping = float(src.damping)
    return dst


class SingleJointOverrideNode(Node):
    """
    Subscribe baseline command; override one joint with Ruckig; publish merged command.
    """

    def __init__(
        self,
        node_name: str = "single_joint_override",
        command_topic: str = "/aima/hal/joint/leg/command",
        area: JointArea = JointArea.LEG,
        control_dt: float = 0.002
    ):
        super().__init__(node_name)

        self.lock = Lock()
        self.command_topic = command_topic
        self.dt = control_dt

        self.joint_info_list = robot_model[area]
        self.joint_info_by_name: Dict[str, JointInfo] = {j.name: j for j in self.joint_info_list}

        # Baseline command cache (latest from balance node)
        self.baseline_cmd: Optional[JointCommandArray] = None
        self.baseline_map: Dict[str, JointCommand] = {}

        # For self-echo filtering
        self.last_pub_override: Optional[Tuple[str, float]] = None  # (joint_name, pos)
        self.self_echo_tol = 1e-8

        # Override state
        self.override_active = False
        self.override_joint: Optional[str] = None
        self.override_target: float = 0.0
        self.hold_sec: float = 0.0
        self.reach_tol: float = 1e-6

        self.reached = False
        self.hold_start: Optional[float] = None

        # 1-DOF Ruckig for the overridden joint
        self.ruckig = ruckig.Ruckig(1, self.dt)
        self.inp = ruckig.InputParameter(1)
        self.out = ruckig.OutputParameter(1)

        # Motion limits (tune if needed)
        self.inp.max_velocity = [1.0]
        self.inp.max_acceleration = [1.0]
        self.inp.max_jerk = [25.0]

        # ROS2 IO
        self.sub_cmd = self.create_subscription(
            JointCommandArray, self.command_topic, self._baseline_callback, subscriber_qos
        )
        self.pub_cmd = self.create_publisher(
            JointCommandArray, self.command_topic, publisher_qos
        )

        # High-rate control loop: only publishes while override is active
        self.ctrl_timer = self.create_timer(self.dt, self._control_tick)

    # ---------- Baseline subscription ----------
    def _baseline_callback(self, msg: JointCommandArray):
        with self.lock:
            # If we're overriding, we might receive our own published message back.
            # Filter by checking override joint position against our last published override pos.
            if self.override_active and self.last_pub_override is not None:
                jn, last_pos = self.last_pub_override
                # find that joint in msg
                for jc in msg.joints:
                    if jc.name == jn:
                        if abs(float(jc.position) - float(last_pos)) <= self.self_echo_tol:
                            # treat as self-echo; ignore baseline update
                            return
                        break  # not echo, accept baseline

            # Accept this as baseline (likely from balance node)
            self.baseline_cmd = msg
            self.baseline_map = {jc.name: jc for jc in msg.joints}

    # ---------- Public API ----------
    def move_joint_hold(
        self,
        joint_name: str,
        target_position: float,
        hold_sec: float,
        max_vel: float = 1.0,
        max_acc: float = 1.0,
        max_jerk: float = 25.0
    ):
        """
        Start overriding one joint: move to target and hold for hold_sec.
        Other joints always follow latest baseline.
        """
        with self.lock:
            if self.baseline_cmd is None or len(self.baseline_map) == 0:
                self.get_logger().warn("No baseline command received yet; cannot start override.")
                return
            if joint_name not in self.baseline_map:
                self.get_logger().warn(f"Joint {joint_name} not found in baseline command; cannot override.")
                return
            if joint_name not in self.joint_info_by_name:
                self.get_logger().warn(f"Joint {joint_name} not in robot_model; cannot clamp/stiffness config.")
                return

            jinfo = self.joint_info_by_name[joint_name]
            tgt = clamp(float(target_position), jinfo.lower_limit, jinfo.upper_limit)

            # Init Ruckig from current baseline state of that joint
            cur_pos = float(self.baseline_map[joint_name].position)
            cur_vel = float(self.baseline_map[joint_name].velocity)

            self.inp.current_position = [cur_pos]
            self.inp.current_velocity = [cur_vel]
            self.inp.current_acceleration = [0.0]

            self.inp.target_position = [tgt]
            self.inp.target_velocity = [0.0]
            self.inp.target_acceleration = [0.0]

            self.inp.max_velocity = [float(max_vel)]
            self.inp.max_acceleration = [float(max_acc)]
            self.inp.max_jerk = [float(max_jerk)]

            self.override_active = True
            self.override_joint = joint_name
            self.override_target = tgt
            self.hold_sec = float(hold_sec)

            self.reached = False
            self.hold_start = None

            # reset self-echo marker
            self.last_pub_override = None

            self.get_logger().info(f"Override start: {joint_name} -> {tgt:.6f}, hold {self.hold_sec:.3f}s")

    def stop_override(self):
        with self.lock:
            if self.override_active:
                self.get_logger().info("Override stop.")
            self.override_active = False
            self.override_joint = None
            self.last_pub_override = None
            self.reached = False
            self.hold_start = None

    # ---------- Control loop ----------
    def _control_tick(self):
        with self.lock:
            if not self.override_active:
                return
            if self.baseline_cmd is None or self.override_joint is None:
                return
            if self.override_joint not in self.baseline_map:
                # baseline not containing the joint temporarily; skip
                return

            joint_name = self.override_joint
            jinfo = self.joint_info_by_name[joint_name]

            # Update Ruckig one step (unless already reached and holding)
            if not self.reached:
                res = self.ruckig.update(self.inp, self.out)
                if res not in [ruckig.Result.Working, ruckig.Result.Finished]:
                    self.get_logger().warn(f"Ruckig update failed: {res}; stopping override.")
                    self.stop_override()
                    return

                # feedback new state
                self.inp.current_position = self.out.new_position
                self.inp.current_velocity = self.out.new_velocity
                self.inp.current_acceleration = self.out.new_acceleration

                override_pos = float(self.out.new_position[0])
                override_vel = float(self.out.new_velocity[0])

                # reach check
                if abs(override_pos - self.override_target) <= self.reach_tol:
                    self.reached = True
                    self.hold_start = time.monotonic()
            else:
                # Holding: command exactly target (velocity 0)
                override_pos = float(self.override_target)
                override_vel = 0.0

                if self.hold_sec <= 0.0:
                    self.stop_override()
                    return
                if self.hold_start is not None and (time.monotonic() - self.hold_start) >= self.hold_sec:
                    self.stop_override()
                    return

            # Build merged command: baseline for all joints, override one joint
            merged = JointCommandArray()
            merged_joints: List[JointCommand] = []

            # Copy baseline joints
            for base_j in self.baseline_cmd.joints:
                merged_joints.append(copy_joint_command(base_j))

            # Override the specified joint fields
            for j in merged_joints:
                if j.name == joint_name:
                    j.position = override_pos
                    j.velocity = override_vel
                    j.effort = 0.0
                    j.stiffness = float(jinfo.kp)
                    j.damping = float(jinfo.kd)
                    break

            merged.joints = merged_joints

            # Publish merged
            self.pub_cmd.publish(merged)

            # update self-echo marker
            self.last_pub_override = (joint_name, override_pos)


def main(args=None):
    rclpy.init(args=args)

    node = SingleJointOverrideNode(
        node_name="single_joint_override_leg",
        command_topic="/aima/hal/joint/leg/command",
        area=JointArea.LEG,
        control_dt=0.002
    )

    # Example: every 6 seconds, move left_knee_joint to two positions, holding 2 seconds each time
    state = {"toggle": False}

    def demo_timer():
        # 你可以改这里实现“到某位置 + hold N 秒”
        if not state["toggle"]:
            node.move_joint_hold("left_knee_joint", 1.6, hold_sec=2.0, max_vel=1.0, max_acc=1.0, max_jerk=25.0)
        else:
            node.move_joint_hold("left_knee_joint", 0.6, hold_sec=2.0, max_vel=1.0, max_acc=1.0, max_jerk=25.0)
        state["toggle"] = not state["toggle"]

    node.create_timer(6.0, demo_timer)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
