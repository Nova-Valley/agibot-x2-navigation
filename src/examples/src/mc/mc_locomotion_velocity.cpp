#include "aimdk_msgs/msg/mc_locomotion_velocity.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/common_task_response.hpp"
#include "aimdk_msgs/msg/mc_input_action.hpp"
#include "aimdk_msgs/msg/message_header.hpp"
#include "aimdk_msgs/srv/set_mc_input_source.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <signal.h>
#include <thread>

class DirectVelocityControl : public rclcpp::Node {
public:
  DirectVelocityControl() : Node("direct_velocity_control") {
    // Create publisher
    publisher_ = this->create_publisher<aimdk_msgs::msg::McLocomotionVelocity>(
        "/aima/mc/locomotion/velocity", 10);
    // Create service client
    client_ = this->create_client<aimdk_msgs::srv::SetMcInputSource>(
        "/aimdk_5Fmsgs/srv/SetMcInputSource");

    // Maximum speed limits
    max_forward_speed_ = 1.0; // m/s
    max_lateral_speed_ = 1.0; // m/s
    max_angular_speed_ = 1.0; // rad/s
    // Minimum speed limits (0 is also OK)
    min_forward_speed_ = 0.2; // m/s
    min_lateral_speed_ = 0.2; // m/s
    min_angular_speed_ = 0.1; // rad/s

    RCLCPP_INFO(this->get_logger(), "Direct velocity control node started.");
  }

  void start_publish() {
    if (timer_ != nullptr) {
      return;
    }
    // Set timer to periodically publish velocity messages (50Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&DirectVelocityControl::publish_velocity, this));
  }

  bool register_input_source() {
    const std::chrono::seconds timeout(8);
    auto start_time = std::chrono::steady_clock::now();
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        RCLCPP_ERROR(this->get_logger(), "Waiting for service timed out");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for input source service...");
    }

    auto request =
        std::make_shared<aimdk_msgs::srv::SetMcInputSource::Request>();
    request->action.value = 1001;
    request->input_source.name = "node";
    request->input_source.priority = 40;
    request->input_source.timeout = 1000;

    auto timeout2 = std::chrono::milliseconds(250);

    for (int i = 0; i < 8; i++) {
      request->request.header.stamp = this->now();
      auto future = client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          this->shared_from_this(), future, timeout2);
      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        // retry as remote peer is NOT handled well by ROS
        RCLCPP_INFO(this->get_logger(),
                    "trying to register input source... [%d]", i);
        continue;
      }
      // future.done
      auto response = future.get();
      int state = response->response.state.value;
      RCLCPP_INFO(this->get_logger(),
                  "Set input source succeeded: state=%d, task_id=%lu", state,
                  response->response.task_id);
      return true;
    }
    RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
    return false;
  }

  void publish_velocity() {
    auto msg = std::make_unique<aimdk_msgs::msg::McLocomotionVelocity>();
    msg->header = aimdk_msgs::msg::MessageHeader();
    msg->header.stamp = this->now();
    msg->source = "node"; // Set message source
    msg->forward_velocity = forward_velocity_;
    msg->lateral_velocity = lateral_velocity_;
    msg->angular_velocity = angular_velocity_;

    publisher_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(),
                "Published velocity: Forward %.2f m/s, Lateral %.2f m/s, "
                "Angular %.2f rad/s",
                forward_velocity_, lateral_velocity_, angular_velocity_);
  }

  void clear_velocity() {
    forward_velocity_ = 0.0;
    lateral_velocity_ = 0.0;
    angular_velocity_ = 0.0;
  }

  bool set_forward(double forward) {
    if (abs(forward) < 0.005) {
      forward_velocity_ = 0.0;
      return true;
    } else if ((abs(forward) > max_forward_speed_) ||
               (abs(forward) < min_forward_speed_)) {
      RCLCPP_ERROR(this->get_logger(), "input value out of range, exiting");
      return false;
    } else {
      forward_velocity_ = forward;
      return true;
    }
  }

  bool set_lateral(double lateral) {
    if (abs(lateral) < 0.005) {
      lateral_velocity_ = 0.0;
      return true;
    } else if ((abs(lateral) > max_lateral_speed_) ||
               (abs(lateral) < min_lateral_speed_)) {
      RCLCPP_ERROR(this->get_logger(), "input value out of range, exiting");
      return false;
    } else {
      lateral_velocity_ = lateral;
      return true;
    }
  }

  bool set_angular(double angular) {
    if (abs(angular) < 0.005) {
      angular_velocity_ = 0.0;
      return true;
    } else if ((abs(angular) > max_angular_speed_) ||
               (abs(angular) < min_angular_speed_)) {
      RCLCPP_ERROR(this->get_logger(), "input value out of range, exiting");
      return false;
    } else {
      angular_velocity_ = angular;
      return true;
    }
  }

private:
  rclcpp::Publisher<aimdk_msgs::msg::McLocomotionVelocity>::SharedPtr
      publisher_;
  rclcpp::Client<aimdk_msgs::srv::SetMcInputSource>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  double forward_velocity_;
  double lateral_velocity_;
  double angular_velocity_;

  double max_forward_speed_;
  double max_lateral_speed_;
  double max_angular_speed_;

  double min_forward_speed_;
  double min_lateral_speed_;
  double min_angular_speed_;
};

// Signal Processing
std::shared_ptr<DirectVelocityControl> global_node = nullptr;
void signal_handler(int sig) {
  if (global_node) {
    global_node->clear_velocity();
    RCLCPP_INFO(global_node->get_logger(),
                "Received signal %d: clearing velocity and shutting down", sig);
  }
  rclcpp::shutdown();
  exit(sig);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  global_node = std::make_shared<DirectVelocityControl>();
  auto node = global_node;

  if (!node->register_input_source()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Input source registration failed, exiting");
    global_node.reset();
    rclcpp::shutdown();
    return 1;
  }

  // get and check control values
  // notice that mc has thresholds to start movement
  double forward, lateral, angular;
  std::cout << "Enter forward speed 0 or ±(0.2 ~ 1.0) m/s: ";
  std::cin >> forward;
  if (!node->set_forward(forward)) {
    return 2;
  }
  std::cout << "Enter lateral speed 0 or ±(0.2 ~ 1.0) m/s: ";
  std::cin >> lateral;
  if (!node->set_lateral(lateral)) {
    return 2;
  }
  std::cout << "Enter angular speed 0 or ±(0.1 ~ 1.0) rad/s: ";
  std::cin >> angular;
  if (!node->set_angular(angular)) {
    return 2;
  }

  RCLCPP_INFO(node->get_logger(), "Setting velocity; moving for 5 seconds");

  node->start_publish();

  auto start_time = node->now();
  while ((node->now() - start_time).seconds() < 5.0) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  node->clear_velocity();
  RCLCPP_INFO(node->get_logger(), "5 seconds elapsed; robot stopped");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
