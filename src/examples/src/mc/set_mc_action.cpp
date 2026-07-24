#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_action.hpp"
#include "aimdk_msgs/msg/mc_action_command.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iomanip>
#include <memory>
#include <signal.h>
#include <unordered_map>
#include <vector>

// Global variable used for signal handling
std::shared_ptr<rclcpp::Node> g_node = nullptr;

// Signal handler function
void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class SetMcActionClient : public rclcpp::Node {
public:
  SetMcActionClient() : Node("set_mc_action_client") {

    client_ = this->create_client<aimdk_msgs::srv::SetMcAction>(
        "/aimdk_5Fmsgs/srv/SetMcAction");
    RCLCPP_INFO(this->get_logger(), "✅ SetMcAction client node created.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool send_request(std::string &action_name) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
      request->header = aimdk_msgs::msg::RequestHeader();

      // Set robot mode
      aimdk_msgs::msg::McActionCommand command;
      command.action_desc = action_name;
      request->command = command;

      RCLCPP_INFO(this->get_logger(), "📨 Sending request to set robot mode: %s",
                  action_name.c_str());

      // Set Service Call Timeout
      const std::chrono::milliseconds timeout(250);
      for (int i = 0; i < 8; i++) {
        request->header.stamp = this->now();
        auto future = client_->async_send_request(request);
        auto retcode = rclcpp::spin_until_future_complete(shared_from_this(),
                                                          future, timeout);
        if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
          // retry as remote peer is NOT handled well by ROS
          RCLCPP_INFO(this->get_logger(), "trying ... [%d]", i);
          continue;
        }
        // future.done
        auto response = future.get();
        if (response->response.status.value ==
            aimdk_msgs::msg::CommonState::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "✅ Robot mode set successfully.");
          return true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "❌ Failed to set robot mode: %s",
                       response->response.message.c_str());
          return false;
        }
      }
      RCLCPP_ERROR(this->get_logger(), "❌ Service call failed or timed out.");
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  rclcpp::Client<aimdk_msgs::srv::SetMcAction>::SharedPtr client_;
};

static std::unordered_map<std::string, std::vector<std::string>> g_action_info =
    {
        {"PASSIVE_DEFAULT", {"PD", "joints with zero torque"}},
        {"DAMPING_DEFAULT", {"DD", "joints in damping mode"}},
        {"JOINT_DEFAULT", {"JD", "Position Control Stand (joints locked)"}},
        {"STAND_DEFAULT", {"SD", "Stable Stand (auto-balance)"}},
        {"LOCOMOTION_DEFAULT", {"LD", "locomotion mode (walk or run)"}},
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);

    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Create node
    g_node = std::make_shared<SetMcActionClient>();
    auto client = std::dynamic_pointer_cast<SetMcActionClient>(g_node);

    if (client) {
      std::unordered_map<std::string, std::string> choices;
      std::string motion;

      // Prefer command-line argument; otherwise prompt user
      if (argc > 1) {
        motion = argv[1];
        RCLCPP_INFO(g_node->get_logger(),
                    "Using abbr of motion mode from cmdline: %s", argv[1]);
      } else {
        std::cout << std::left << std::setw(4) << "abbr"
                  << " - " << std::setw(20) << "robot mode"
                  << " : "
                  << "description" << std::endl;
        for (auto &it : g_action_info) {
          std::cout << std::left << std::setw(4) << it.second[0] << " - "
                    << std::setw(20) << it.first << " : " << it.second[1]
                    << std::endl;
        }
        std::cout << "Enter abbr of motion mode:";
        std::cin >> motion;
      }
      for (auto &it : g_action_info) {
        choices[it.second[0]] = it.first;
      }

      auto m = choices.find(motion);
      if (m != choices.end()) {
        auto &action_name = m->second;
        client->send_request(action_name);
      } else {
        RCLCPP_ERROR(g_node->get_logger(), "Invalid abbr of robot mode: %s",
                     motion.c_str());
      }
    }

    // Clean up resources
    g_node.reset();
    rclcpp::shutdown();

    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
