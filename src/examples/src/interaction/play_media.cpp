#include <aimdk_msgs/msg/tts_priority_level.hpp>
#include <aimdk_msgs/srv/play_media_file.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

using PlayMediaFile = aimdk_msgs::srv::PlayMediaFile;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("play_media_file_client_min");

  // 1) Service name
  const std::string service_name = "/aimdk_5Fmsgs/srv/PlayMediaFile";
  auto client = node->create_client<PlayMediaFile>(service_name);

  // 2) Input file path (prompt user if not provided as argument)
  std::string default_file =
      "/agibot/data/var/interaction/tts_cache/normal/demo.wav";
  std::string file_name;

  if (argc > 1) {
    file_name = argv[1];
  } else {
    std::cout << "Enter the media file path to play (default: " << default_file
              << "): ";
    std::getline(std::cin, file_name);
    if (file_name.empty()) {
      file_name = default_file;
    }
  }

  // 3) Build the request
  auto req = std::make_shared<PlayMediaFile::Request>();
  // CommonRequest request -> RequestHeader header -> builtin_interfaces/Time
  // stamp
  req->header.header.stamp = node->now();

  // PlayMediaFileRequest required fields
  req->media_file_req.file_name = file_name;
  req->media_file_req.domain = "demo_client"; // Required: identifies the caller
  req->media_file_req.trace_id = "demo";      // Optional: request identifier
  req->media_file_req.is_interrupted =
      true; // Whether to interrupt same-priority playback
  req->media_file_req.priority_weight = 0; // Optional: 0~99
  // Priority level: default INTERACTION_L6
  req->media_file_req.priority_level.value = 6;

  // 4) Wait for service and call
  RCLCPP_INFO(node->get_logger(), "Waiting for service: %s",
              service_name.c_str());
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Service unavailable: %s",
                 service_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto future = client->async_send_request(req);
  auto rc = rclcpp::spin_until_future_complete(node, future,
                                               std::chrono::seconds(10));

  if (rc == rclcpp::FutureReturnCode::INTERRUPTED) {
    RCLCPP_WARN(node->get_logger(), "Interrupted while waiting");
    rclcpp::shutdown();
    return 1;
  }

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Call timed out or did not complete");
    rclcpp::shutdown();
    return 1;
  }

  // 5) Handle response (success is in tts_resp)
  try {
    const auto resp = future.get();
    bool success = resp->tts_resp.is_success;

    if (success) {
      RCLCPP_INFO(node->get_logger(), "✅ Media file play request succeeded: %s",
                  file_name.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "❌ Media file play request failed: %s",
                   file_name.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Call exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
