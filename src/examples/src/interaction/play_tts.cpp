#include <aimdk_msgs/msg/tts_priority_level.hpp>
#include <aimdk_msgs/srv/play_tts.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

using PlayTTS = aimdk_msgs::srv::PlayTts;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("play_tts_client_min");

  const std::string service_name = "/aimdk_5Fmsgs/srv/PlayTts";
  auto client = node->create_client<PlayTTS>(service_name);

  // Get text to speak
  std::string tts_text;
  if (argc > 1) {
    tts_text = argv[1];
  } else {
    std::cout << "Enter text to speak: ";
    std::getline(std::cin, tts_text);
    if (tts_text.empty()) {
      tts_text = "Hello, I am Lingxi X2.";
    }
  }

  auto req = std::make_shared<PlayTTS::Request>();
  req->header.header.stamp = node->now();
  req->tts_req.text = tts_text;
  req->tts_req.domain = "demo_client"; // Required: identifies the caller
  req->tts_req.trace_id =
      "demo"; // Optional: request identifier for the TTS request
  req->tts_req.is_interrupted =
      true; // Required: whether to interrupt same-priority playback
  req->tts_req.priority_weight = 0;
  req->tts_req.priority_level.value = 6;

  if (!client->wait_for_service(
          std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::seconds(5)))) {
    RCLCPP_ERROR(node->get_logger(), "Service unavailable: %s",
                 service_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(
          node, future,
          std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::seconds(10))) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Call timed out");
    rclcpp::shutdown();
    return 1;
  }

  const auto resp = future.get();
  if (resp->tts_resp.is_success) {
    RCLCPP_INFO(node->get_logger(), "✅ TTS play request succeeded");
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ TTS play request failed");
  }

  rclcpp::shutdown();
  return 0;
}
