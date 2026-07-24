#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <atomic>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>
#include <signal.h>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Global variables and signal handling
 */
// Global variables to control program state
std::atomic<bool> g_running(true);
std::atomic<bool> g_emergency_stop(false);

// Signal handler function
void signal_handler(int) {
  g_running = false;
  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Received termination signal, shutting down...");
}

/**
 * @brief Robot model definition
 */
enum class JointArea {
  HEAD,  // Head joints
  ARM,   // Arm joints
  WAIST, // Waist joints
  LEG,   // Leg joints
};

/**
 * @brief Joint information structure
 */
struct JointInfo {
  std::string name;   // Joint name
  double lower_limit; // Joint angle lower limit
  double upper_limit; // Joint angle upper limit
  double kp;          // Position control gain
  double kd;          // Velocity control gain
};

/**
 * @brief Robot model configuration
 * Contains parameters for all joints, enabling or disabling specific joints as
 * needed
 */
std::map<JointArea, std::vector<JointInfo>> robot_model = {
    {JointArea::LEG,
     {
         // Left leg joint configuration
         {"left_hip_pitch_joint", -2.4871, 2.4871, 40.0, 4.0}, // Left hip pitch
         {"left_hip_roll_joint", -0.12217, 2.9059, 40.0, 4.0}, // Left hip roll
         {"left_hip_yaw_joint", -1.6842, 3.4296, 30.0, 3.0},   // Left hip yaw
         {"left_knee_joint", 0.026179, 2.1206, 80.0, 8.0},     // Left knee
         {"left_ankle_pitch_joint", -0.80285, 0.45378, 40.0,
          4.0}, // Left ankle pitch
         {"left_ankle_roll_joint", -0.2618, 0.2618, 20.0,
          2.0}, // Left ankle roll
                // Right leg joint configuration
         {"right_hip_pitch_joint", -2.4871, 2.4871, 40.0,
          4.0}, // Right hip pitch
         {"right_hip_roll_joint", -2.9059, 0.12217, 40.0,
          4.0},                                               // Right hip roll
         {"right_hip_yaw_joint", -3.4296, 1.6842, 30.0, 3.0}, // Right hip yaw
         {"right_knee_joint", 0.026179, 2.1206, 80.0, 8.0},   // Right knee
         {"right_ankle_pitch_joint", -0.80285, 0.45378, 40.0,
          4.0}, // Right ankle pitch
         {"right_ankle_roll_joint", -0.2618, 0.2618, 20.0,
          2.0}, // Right ankle roll
     }},

    {JointArea::WAIST,
     {
         // Waist joint configuration
         {"waist_yaw_joint", -3.4296, 2.3824, 20.0, 4.0},     // Waist yaw
         {"waist_pitch_joint", -0.17453, 0.17453, 20.0, 4.0}, // Waist pitch
         {"waist_roll_joint", -0.48869, 0.48869, 20.0, 4.0},  // Waist roll
     }},
    {JointArea::ARM,
     {
         // Left arm joint configuration
         {"left_shoulder_pitch_joint", -2.5569, 2.5569, 20.0,
          2.0}, // Left shoulder pitch
         {"left_shoulder_roll_joint", -0.06108, 3.3598, 20.0,
          2.0}, // Left shoulder roll
         {"left_shoulder_yaw_joint", -2.5569, 2.5569, 20.0,
          2.0}, // Left shoulder yaw
         {"left_elbow_pitch_joint", -2.4435, 0.0, 20.0,
          2.0}, // Left elbow pitch
         {"left_elbow_roll_joint", -2.5569, 2.5569, 20.0,
          2.0}, // Left elbow roll
         {"left_wrist_pitch_joint", -0.5585, 0.5585, 20.0,
          2.0}, // Left wrist pitch
         {"left_wrist_yaw_joint", -1.5446, 1.5446, 20.0,
          2.0}, // Left wrist yaw
                // Right arm joint configuration
         {"right_shoulder_pitch_joint", -2.5569, 2.5569, 20.0,
          2.0}, // Right shoulder pitch
         {"right_shoulder_roll_joint", -3.3598, 0.06108, 20.0,
          2.0}, // Right shoulder roll
         {"right_shoulder_yaw_joint", -2.5569, 2.5569, 20.0,
          2.0}, // Right shoulder yaw
         {"right_elbow_pitch_joint", 0.0, 2.4435, 20.0,
          2.0}, // Right elbow pitch
         {"right_elbow_roll_joint", -2.5569, 2.5569, 20.0,
          2.0}, // Right elbow roll
         {"right_wrist_pitch_joint", -0.5585, 0.5585, 20.0,
          2.0}, // Right wrist pitch
         {"right_wrist_yaw_joint", -1.5446, 1.5446, 20.0,
          2.0}, // Right wrist yaw
     }},
    {JointArea::HEAD,
     {
         // Head joint configuration
         {"head_pitch_joint", -0.61087, 0.61087, 20.0, 2.0}, // Head pitch
         {"head_yaw_joint", -0.61087, 0.61087, 20.0, 2.0},   // Head yaw
     }},
};

/**
 * @brief Joint controller node class
 * @tparam DOFs Degrees of freedom
 * @tparam Area Joint area
 */
template <int DOFs, JointArea Area>
class JointControllerNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   * @param node_name Node name
   * @param sub_topic Subscription topic name
   * @param pub_topic Publication topic name
   * @param qos QoS configuration
   */
  JointControllerNode(std::string node_name, std::string sub_topic,
                      std::string pub_topic,
                      rclcpp::QoS qos = rclcpp::SensorDataQoS())
      : Node(node_name), ruckig(0.002) {
    joint_info_ = robot_model[Area];
    if (joint_info_.size() != DOFs) {
      RCLCPP_ERROR(this->get_logger(), "Joint count mismatch.");
      exit(1);
    }

    // Set motion constraints for Ruckig trajectory planner
    for (int i = 0; i < DOFs; ++i) {
      input.max_velocity[i] = 1.0;     // Max velocity limit
      input.max_acceleration[i] = 1.0; // Max acceleration limit
      input.max_jerk[i] = 25.0; // Max jerk (change of acceleration) limit
    }

    // Create joint state subscriber
    sub_ = this->create_subscription<aimdk_msgs::msg::JointStateArray>(
        sub_topic, qos,
        std::bind(&JointControllerNode::JointStateCallback, this,
                  std::placeholders::_1));

    // Create joint command publisher
    pub_ = this->create_publisher<aimdk_msgs::msg::JointCommandArray>(pub_topic,
                                                                      qos);
  }

private:
  // Ruckig trajectory planner variables
  ruckig::Ruckig<DOFs> ruckig;          // Trajectory planner instance
  ruckig::InputParameter<DOFs> input;   // Input parameters
  ruckig::OutputParameter<DOFs> output; // Output parameters
  bool ruckig_initialized_ = false;   // Trajectory planner initialization flag
  std::vector<JointInfo> joint_info_; // Joint information list

  // ROS communication variables
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr
      sub_; // State subscriber
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr
      pub_; // Command publisher

  /**
   * @brief Joint state callback function
   * @param msg Joint state message
   */
  void
  JointStateCallback(const aimdk_msgs::msg::JointStateArray::SharedPtr msg) {
    // Initialize trajectory planner on first state reception
    if (!ruckig_initialized_) {
      for (int i = 0; i < DOFs; ++i) {
        input.current_position[i] = msg->joints[i].position;
        input.current_velocity[i] = msg->joints[i].velocity;
        input.current_acceleration[i] = 0.0;
      }
      ruckig_initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Ruckig trajectory planner initialization complete");
    }
  }

public:
  /**
   * @brief Set target joint position
   * @param joint_name Joint name
   * @param target_position Target position
   * @return Whether the target position was successfully set
   */
  bool SetTargetPosition(std::string joint_name, double target_position) {
    if (!ruckig_initialized_) {
      RCLCPP_WARN(this->get_logger(),
                  "Ruckig trajectory planner not initialized");
      return false;
    }

    // Find target joint and set its position
    int target_joint = -1;
    for (int i = 0; i < DOFs; ++i) {
      if (joint_info_[i].name == joint_name) {
        // Check if target position is within limits
        if (target_position < joint_info_[i].lower_limit ||
            target_position > joint_info_[i].upper_limit) {
          RCLCPP_ERROR(
              this->get_logger(),
              "Target position %.3f exceeds limit for joint %s [%.3f, %.3f]",
              target_position, joint_name.c_str(), joint_info_[i].lower_limit,
              joint_info_[i].upper_limit);
          return false;
        }
        input.target_position[i] = target_position;
        input.target_velocity[i] = 0.0;
        input.target_acceleration[i] = 0.0;
        target_joint = i;
      } else {
        input.target_position[i] = input.current_position[i];
        input.target_velocity[i] = 0.0;
        input.target_acceleration[i] = 0.0;
      }
    }

    if (target_joint == -1) {
      RCLCPP_ERROR(this->get_logger(), "Joint %s not found",
                   joint_name.c_str());
      return false;
    }

    // Perform trajectory planning and send command using Ruckig
    const double tolerance = 1e-6;
    while (g_running && rclcpp::ok() && !g_emergency_stop) {
      auto result = ruckig.update(input, output);
      if (result != ruckig::Result::Working &&
          result != ruckig::Result::Finished) {
        RCLCPP_WARN(this->get_logger(), "Trajectory planning failed");
        break;
      }

      // Update current state
      for (int i = 0; i < DOFs; ++i) {
        input.current_position[i] = output.new_position[i];
        input.current_velocity[i] = output.new_velocity[i];
        input.current_acceleration[i] = output.new_acceleration[i];
      }

      // Check if target position is reached
      if (std::abs(output.new_position[target_joint] - target_position) <
          tolerance) {
        RCLCPP_INFO(this->get_logger(), "Joint %s reached target position",
                    joint_name.c_str());
        break;
      }

      // Create and send joint command
      aimdk_msgs::msg::JointCommandArray cmd;
      cmd.joints.resize(DOFs);
      for (int i = 0; i < DOFs; ++i) {
        auto &joint = joint_info_[i];
        cmd.joints[i].name = joint.name;
        cmd.joints[i].position = output.new_position[i];
        cmd.joints[i].velocity = output.new_velocity[i];
        cmd.joints[i].stiffness = joint.kp;
        cmd.joints[i].damping = joint.kd;
      }
      pub_->publish(cmd);

      // Short delay to avoid excessive CPU usage
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return true;
  }

  /**
   * @brief Safely stop all joints
   */
  void safe_stop() {
    if (!ruckig_initialized_) {
      RCLCPP_WARN(this->get_logger(), "Ruckig trajectory planner not "
                                      "initialized, cannot perform safe stop");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Performing safe stop...");

    // Set all joint target positions to current positions
    for (int i = 0; i < DOFs; ++i) {
      input.target_position[i] = input.current_position[i];
      input.target_velocity[i] = 0.0;
      input.target_acceleration[i] = 0.0;
    }

    // Send final command to ensure joints stop
    aimdk_msgs::msg::JointCommandArray cmd;
    cmd.joints.resize(DOFs);
    for (int i = 0; i < DOFs; ++i) {
      auto &joint = joint_info_[i];
      cmd.joints[i].name = joint.name;
      cmd.joints[i].position = input.current_position[i];
      cmd.joints[i].velocity = 0.0;
      cmd.joints[i].stiffness = joint.kp;
      cmd.joints[i].damping = joint.kd;
    }
    pub_->publish(cmd);

    RCLCPP_INFO(this->get_logger(), "Safe stop complete");
  }

  /**
   * @brief Emergency stop for all joints
   */
  void emergency_stop() {
    g_emergency_stop = true;
    safe_stop();
    RCLCPP_ERROR(this->get_logger(), "Emergency stop triggered");
  }
};

/**
 * @brief Main function
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Set up signal handling
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    // Create leg controller node
    auto leg_node = std::make_shared<JointControllerNode<12, JointArea::LEG>>(
        "leg_node", "/aima/hal/joint/leg/state", "/aima/hal/joint/leg/command");

    // Create timer node
    rclcpp::Node::SharedPtr timer_node =
        rclcpp::Node::make_shared("timer_node");
    double position = 0.8;

    // Create timer callback function
    auto timer = timer_node->create_wall_timer(std::chrono::seconds(3), [&]() {
      if (!g_running || g_emergency_stop)
        return; // If the program is shutting down or emergency stopped, do not
                // execute new actions
      position = -position;
      position = 1.3 + position;
      if (!leg_node->SetTargetPosition("left_knee_joint", position)) {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Failed to set target position");
      }
    });

    // Create executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(leg_node);
    executor.add_node(timer_node);

    // Main loop
    while (g_running && rclcpp::ok() && !g_emergency_stop) {
      executor.spin_once(std::chrono::milliseconds(100));
    }

    // Safely stop all joints
    RCLCPP_INFO(rclcpp::get_logger("main"), "Safely stopping all joints...");
    leg_node->safe_stop();

    // Wait a short time to ensure command transmission is complete
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Clean up resources
    RCLCPP_INFO(rclcpp::get_logger("main"), "Cleaning up resources...");
    leg_node.reset();
    timer_node.reset();

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception occurred: %s",
                 e.what());
    g_emergency_stop = true;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown exception occurred");
    g_emergency_stop = true;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "Program exited safely");
  rclcpp::shutdown();
  return 0;
}
