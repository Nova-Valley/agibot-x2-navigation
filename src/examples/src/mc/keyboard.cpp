#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/common_task_response.hpp"
#include "aimdk_msgs/msg/mc_input_action.hpp"
#include "aimdk_msgs/msg/mc_locomotion_velocity.hpp"
#include "aimdk_msgs/msg/message_header.hpp"
#include "aimdk_msgs/srv/set_mc_input_source.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <curses.h>
#include <rclcpp/rclcpp.hpp>

using aimdk_msgs::msg::McLocomotionVelocity;
using std::placeholders::_1;

class KeyboardVelocityController : public rclcpp::Node {
public:
  KeyboardVelocityController()
      : Node("keyboard_velocity_controller"), forward_velocity_(0.0),
        lateral_velocity_(0.0), angular_velocity_(0.0), step_(0.2),
        angular_step_(0.1) {
    pub_ = this->create_publisher<McLocomotionVelocity>(
        "/aima/mc/locomotion/velocity", 10);
    client_ = this->create_client<aimdk_msgs::srv::SetMcInputSource>(
        "/aimdk_5Fmsgs/srv/SetMcInputSource");
    // Register input source
    if (!register_input_source()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Input source registration failed, exiting");
      throw std::runtime_error("Input source registration failed");
    }
    // Initialize ncurses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&KeyboardVelocityController::checkKeyAndPublish, this));

    RCLCPP_INFO(this->get_logger(),
                "Control started: W/S Forward/Backward | A/D Strafe Left/Right "
                "| Q/E Turn Left/Right | Space Stop | ESC Exit");
  }

  ~KeyboardVelocityController() {
    endwin(); // Restore terminal
  }

private:
  rclcpp::Publisher<McLocomotionVelocity>::SharedPtr pub_;
  rclcpp::Client<aimdk_msgs::srv::SetMcInputSource>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  float forward_velocity_, lateral_velocity_, angular_velocity_;
  const float step_, angular_step_;

  bool register_input_source() {
    const std::chrono::seconds srv_timeout(8);
    auto start_time = std::chrono::steady_clock::now();
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (std::chrono::steady_clock::now() - start_time > srv_timeout) {
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

    auto timeout = std::chrono::milliseconds(250);

    for (int i = 0; i < 8; i++) {
      request->request.header.stamp = this->now();
      auto future = client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), future, timeout);
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

  void checkKeyAndPublish() {
    int ch = getch(); // non-blocking read

    switch (ch) {
    case ' ': // Space key
      forward_velocity_ = 0.0;
      lateral_velocity_ = 0.0;
      angular_velocity_ = 0.0;
      break;
    case 'w':
      forward_velocity_ = std::min(forward_velocity_ + step_, 1.0f);
      break;
    case 's':
      forward_velocity_ = std::max(forward_velocity_ - step_, -1.0f);
      break;
    case 'a':
      lateral_velocity_ = std::min(lateral_velocity_ + step_, 1.0f);
      break;
    case 'd':
      lateral_velocity_ = std::max(lateral_velocity_ - step_, -1.0f);
      break;
    case 'q':
      angular_velocity_ = std::min(angular_velocity_ + angular_step_, 1.0f);
      break;
    case 'e':
      angular_velocity_ = std::max(angular_velocity_ - angular_step_, -1.0f);
      break;
    case 27: // ESC Key
      RCLCPP_INFO(this->get_logger(), "Exiting control");
      rclcpp::shutdown();
      return;
    }

    auto msg = std::make_unique<McLocomotionVelocity>();
    msg->header = aimdk_msgs::msg::MessageHeader();
    msg->header.stamp = this->now();
    msg->source = "node";
    msg->forward_velocity = forward_velocity_;
    msg->lateral_velocity = lateral_velocity_;
    msg->angular_velocity = angular_velocity_;

    float fwd = forward_velocity_;
    float lat = lateral_velocity_;
    float ang = angular_velocity_;

    pub_->publish(std::move(msg));

    // Screen Output
    clear();
    mvprintw(0, 0,
             "W/S: Forward/Backward | A/D: Left/Right Strafe | Q/E: Turn "
             "Left/Right | Space: Stop | ESC: Exit");
    mvprintw(2, 0,
             "Speed Status: Forward: %.2f m/s | Lateral: %.2f m/s | Angular: "
             "%.2f rad/s",
             fwd, lat, ang);
    refresh();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<KeyboardVelocityController>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
