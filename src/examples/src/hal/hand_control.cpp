#include "aimdk_msgs/msg/hand_command.hpp"
#include "aimdk_msgs/msg/hand_command_array.hpp"
#include "aimdk_msgs/msg/hand_type.hpp"
#include "aimdk_msgs/msg/message_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>

class HandControl : public rclcpp::Node {
public:
  HandControl()
      : Node("hand_control"), position_pairs_({
                                  {1.0, 1.0},
                                  {0.0, 0.0},
                                  {0.5, 0.5},
                                  {0.2, 0.8},
                                  {0.7, 0.3},
                              }),
        current_index_(0) {
    publisher_ = this->create_publisher<aimdk_msgs::msg::HandCommandArray>(
        "/aima/hal/joint/hand/command", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50Hz
        std::bind(&HandControl::publish_hand_commands, this));

    last_switch_time_ = now();
    RCLCPP_INFO(this->get_logger(), "The hand control node has been started!");
  }

  void publish_hand_commands() {
    // 1. Determine if it's time to switch parameters.
    auto now_time = this->now();
    if ((now_time - last_switch_time_).seconds() >= 2.0) {
      current_index_ = (current_index_ + 1) % position_pairs_.size();
      last_switch_time_ = now_time;
      RCLCPP_INFO(this->get_logger(),
                  "已切换到下一个参数组，索引=%zu (left=%.2f, right=%.2f)",
                  current_index_, position_pairs_[current_index_].first,
                  position_pairs_[current_index_].second);
    }

    auto msg = std::make_unique<aimdk_msgs::msg::HandCommandArray>();
    msg->header = aimdk_msgs::msg::MessageHeader();

    float left_position = position_pairs_[current_index_].first;
    float right_position = position_pairs_[current_index_].second;

    aimdk_msgs::msg::HandCommand left_hands;
    left_hands.name = "left_hand";
    left_hands.position = left_position;
    left_hands.velocity = 1.0;
    left_hands.acceleration = 1.0;
    left_hands.deceleration = 1.0;
    left_hands.effort = 1.0;

    aimdk_msgs::msg::HandCommand right_hands;
    right_hands.name = "right_hand";
    right_hands.position = right_position;
    right_hands.velocity = 1.0;
    right_hands.acceleration = 1.0;
    right_hands.deceleration = 1.0;
    right_hands.effort = 1.0;

    msg->left_hands.push_back(left_hands);
    msg->right_hands.push_back(right_hands);
    msg->left_hand_type.value = 2;
    msg->right_hand_type.value = 2;

    publisher_->publish(std::move(msg));
  }

private:
  rclcpp::Publisher<aimdk_msgs::msg::HandCommandArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<float, float>> position_pairs_;
  size_t current_index_;

  rclcpp::Time last_switch_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto hand_control_node = std::make_shared<HandControl>();
  rclcpp::spin(hand_control_node);
  rclcpp::shutdown();
  return 0;
}
