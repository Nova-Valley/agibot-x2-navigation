#include "aimdk_msgs/msg/hand_command.hpp"
#include "aimdk_msgs/msg/hand_command_array.hpp"
#include "aimdk_msgs/msg/hand_type.hpp"
#include "aimdk_msgs/msg/message_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>

/**
 * @brief Omnihand control node
 */
class OmnihandControl : public rclcpp::Node {
public:
  OmnihandControl()
      : Node("omnihand_control"), position_pairs_({
                                      {1.0, 1.0}, // Hands fully open
                                      {0.0, 0.0}, // Hands fully closed
                                      {0.5, 0.5}, // Hands half open
                                      {0.3, 0.7}, // Left closed right open
                                      {0.8, 0.2}  // Left open right closed
                                  }),
        current_index_(0) {
    publisher_ = this->create_publisher<aimdk_msgs::msg::HandCommandArray>(
        "/aima/hal/joint/hand/command", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50Hz
        std::bind(&OmnihandControl::publish_hand_commands, this));

    last_switch_time_ = now();
    RCLCPP_INFO(this->get_logger(), "Omnihand control node has been started!");
  }

  /**
   * @brief Publish hand commands
   */
  void publish_hand_commands() {
    // 1. Switches parameter set every 5 seconds
    auto now_time = this->now();
    if ((now_time - last_switch_time_).seconds() >= 5.0) {
      current_index_ = (current_index_ + 1) % position_pairs_.size();
      last_switch_time_ = now_time;
      RCLCPP_INFO(
          this->get_logger(),
          "Switched to parameter set %zu, left hand = %.2f, right hand = %.2f",
          current_index_, position_pairs_[current_index_].first,
          position_pairs_[current_index_].second);
    }

    auto msg = std::make_unique<aimdk_msgs::msg::HandCommandArray>();
    msg->header = aimdk_msgs::msg::MessageHeader();

    float left_position = position_pairs_[current_index_].first;
    float right_position = position_pairs_[current_index_].second;

    // Left hand 10 joints
    for (int i = 0; i < 10; ++i) {
      aimdk_msgs::msg::HandCommand left_hands;
      left_hands.name = "left_hand_joint" + std::to_string(i);
      left_hands.position = left_position;
      left_hands.velocity = 1.0;
      left_hands.acceleration = 1.0;
      left_hands.deceleration = 1.0;
      left_hands.effort = 1.0;
      msg->left_hands.push_back(left_hands);
    }
    // Right hand 10 joints
    for (int i = 0; i < 10; ++i) {
      aimdk_msgs::msg::HandCommand right_hands;
      right_hands.name = "right_hand_joint" + std::to_string(i);
      right_hands.position = right_position;
      right_hands.velocity = 1.0;
      right_hands.acceleration = 1.0;
      right_hands.deceleration = 1.0;
      right_hands.effort = 1.0;
      msg->right_hands.push_back(right_hands);
    }

    // Type 1: Dexterous hand mode
    msg->left_hand_type.value = 1;
    msg->right_hand_type.value = 1;

    publisher_->publish(std::move(msg));
  }

private:
  // Member variables
  rclcpp::Publisher<aimdk_msgs::msg::HandCommandArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<float, float>> position_pairs_;
  size_t current_index_;
  rclcpp::Time last_switch_time_;
};

/**
 * @brief Main function
 */
// Initializes ROS, creates node, and starts spinning
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto omnihand_control_node = std::make_shared<OmnihandControl>();
  rclcpp::spin(omnihand_control_node);
  rclcpp::shutdown();
  return 0;
}
