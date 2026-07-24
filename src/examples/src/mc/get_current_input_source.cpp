#include "aimdk_msgs/srv/get_current_input_source.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/response_header.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <signal.h>

// Global node object
std::shared_ptr<rclcpp::Node> g_node = nullptr;

// Signal handler
void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

// Client Class
class GetCurrentInputSourceClient : public rclcpp::Node {
public:
  GetCurrentInputSourceClient() : Node("get_current_input_source_client") {

    client_ = this->create_client<aimdk_msgs::srv::GetCurrentInputSource>(
        "/aimdk_5Fmsgs/srv/GetCurrentInputSource");

    RCLCPP_INFO(this->get_logger(),
                "✅ GetCurrentInputSource client node created.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  void send_request() {
    try {
      auto request =
          std::make_shared<aimdk_msgs::srv::GetCurrentInputSource::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();

      RCLCPP_INFO(this->get_logger(),
                  "📨 Sending request to get current input source");

      auto timeout = std::chrono::milliseconds(250);

      for (int i = 0; i < 8; i++) {
        request->request.header.stamp = this->now();
        auto future = client_->async_send_request(request);
        auto retcode = rclcpp::spin_until_future_complete(
            this->shared_from_this(), future, timeout);
        if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
          // retry as remote peer is NOT handled well by ROS
          RCLCPP_INFO(this->get_logger(), "trying ... [%d]", i);
          continue;
        }
        // future.done
        auto response = future.get();
        if (response->response.header.code == 0) {
          RCLCPP_INFO(this->get_logger(),
                      "✅ Current input source get successfully:");
          RCLCPP_INFO(this->get_logger(), "Name: %s",
                      response->input_source.name.c_str());
          RCLCPP_INFO(this->get_logger(), "Priority: %d",
                      response->input_source.priority);
          RCLCPP_INFO(this->get_logger(), "Timeout: %d",
                      response->input_source.timeout);
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "❌ Current input source get failed, return code: %ld",
                      response->response.header.code);
        }
        return;
      }
      RCLCPP_ERROR(this->get_logger(), "❌ Service call failed or timed out.");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
    }
  }

private:
  rclcpp::Client<aimdk_msgs::srv::GetCurrentInputSource>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<GetCurrentInputSourceClient>();
    auto client =
        std::dynamic_pointer_cast<GetCurrentInputSourceClient>(g_node);

    if (client) {
      client->send_request();
    }

    g_node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
