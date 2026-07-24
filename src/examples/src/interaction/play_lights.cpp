#include <aimdk_msgs/msg/common_request.hpp>
#include <aimdk_msgs/srv/led_strip_command.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class PlayLightsClient : public rclcpp::Node {
public:
  PlayLightsClient() : Node("play_lights_client") {
    client_ = this->create_client<aimdk_msgs::srv::LedStripCommand>(
        "/aimdk_5Fmsgs/srv/LedStripCommand");
    RCLCPP_INFO(this->get_logger(), "✅ PlayLights client node started.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool send_request(uint8_t led_mode, uint8_t r, uint8_t g, uint8_t b) {
    try {
      auto request =
          std::make_shared<aimdk_msgs::srv::LedStripCommand::Request>();

      request->led_strip_mode = led_mode;
      request->r = r;
      request->g = g;
      request->b = b;

      RCLCPP_INFO(this->get_logger(),
                  "📨 Sending request to control led strip: mode=%hhu, "
                  "RGB=(%hhu, %hhu, %hhu)",
                  led_mode, r, g, b);

      // LED strip is slow to response (up to ~5s)
      const std::chrono::milliseconds timeout(5000);
      for (int i = 0; i < 4; i++) {
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
        if (response->status_code == 0) {
          RCLCPP_INFO(this->get_logger(),
                      "✅ LED strip command sent successfully.");
          return true;
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "❌ LED strip command failed with status: %d",
                       response->status_code);
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
  rclcpp::Client<aimdk_msgs::srv::LedStripCommand>::SharedPtr client_;
};

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PlayLightsClient>();
    auto client_node = std::dynamic_pointer_cast<PlayLightsClient>(g_node);

    int led_mode = 0;          // LED Strip Mode
    int r = 255, g = 0, b = 0; // RGB values

    std::cout << "=== LED Strip Control Example ===" << std::endl;
    std::cout << "Select LED strip mode:" << std::endl;
    std::cout << "0 - Steady On" << std::endl;
    std::cout << "1 - Breathing (4s period, sinusoidal brightness)"
              << std::endl;
    std::cout << "2 - Blinking (1s period, 0.5s on, 0.5s off)" << std::endl;
    std::cout << "3 - Flow (2s period, lights turn on left to right)"
              << std::endl;
    std::cout << "Enter mode (0-3): ";
    std::cin >> led_mode;

    std::cout << "\nSet RGB color values (0-255):" << std::endl;
    std::cout << "Red component (R): ";
    std::cin >> r;
    std::cout << "Green component (G): ";
    std::cin >> g;
    std::cout << "Blue component (B): ";
    std::cin >> b;

    // clamp mode to range 0-3
    led_mode = std::max(0, std::min(3, led_mode));
    // clamp r/g/b to range 0-255
    r = std::max(0, std::min(255, r));
    g = std::max(0, std::min(255, g));
    b = std::max(0, std::min(255, b));

    if (client_node) {
      client_node->send_request(led_mode, r, g, b);
    }

    g_node.reset();
    rclcpp::shutdown();

    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program terminated with exception: %s", e.what());
    return 1;
  }
}
