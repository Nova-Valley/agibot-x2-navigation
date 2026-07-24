#include "aimdk_msgs/srv/play_emoji.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <signal.h>
#include <string>

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

class PlayEmojiClient : public rclcpp::Node {
public:
  PlayEmojiClient() : Node("play_emoji_client") {
    client_ = this->create_client<aimdk_msgs::srv::PlayEmoji>(
        "/face_ui_proxy/play_emoji");
    RCLCPP_INFO(this->get_logger(), "✅ PlayEmoji client node started.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool send_request(uint8_t emoji, uint8_t mode, int32_t priority) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::PlayEmoji::Request>();

      request->emotion_id = emoji;
      request->mode = mode;
      request->priority = priority;

      RCLCPP_INFO(
          this->get_logger(),
          "📨 Sending request to play emoji: id=%hhu, mode=%hhu, priority=%d",
          emoji, mode, priority);

      const std::chrono::milliseconds timeout(250);
      for (int i = 0; i < 8; i++) {
        request->header.header.stamp = this->now();
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
        if (response->success) {
          RCLCPP_INFO(this->get_logger(),
                      "✅ Request to play emoji recorded successfully: %s",
                      response->message.c_str());
          return true;
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "❌ Failed to record play-emoji request: %s",
                       response->message.c_str());
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
  rclcpp::Client<aimdk_msgs::srv::PlayEmoji>::SharedPtr client_;
};

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);

    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int32_t priority = 10;

    int emotion = 1; // Expression type, 1 means Blink
    std::cout
        << "Enter expression ID: 1-Blink, 60-Bored, 70-Abnormal, 80-Sleeping, "
           "90-Happy, 190-Very Angry, 200-Adoration"
        << std::endl;
    std::cin >> emotion;

    int mode = 1; // Playback mode, 1 means play once, 2 means loop
    std::cout << "Enter play mode (1: once, 2: loop): ";
    std::cin >> mode;
    if (mode < 1 || mode > 2) {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid play mode: %d", mode);
      rclcpp::shutdown();
      return 1;
    }

    g_node = std::make_shared<PlayEmojiClient>();
    auto client = std::dynamic_pointer_cast<PlayEmojiClient>(g_node);

    if (client) {
      client->send_request(emotion, mode, priority);
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
