#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class SaveOneRaw : public rclcpp::Node {
public:
  SaveOneRaw() : Node("save_one_image"), saved_(false) {
    topic_ = declare_parameter<std::string>(
        "image_topic", "/aima/hal/sensor/stereo_head_front_left/rgb_image");

    std::filesystem::create_directories("images");

    auto qos = rclcpp::SensorDataQoS(); // BestEffort/Volatile
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        topic_, qos, std::bind(&SaveOneRaw::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing (raw): %s", topic_.c_str());
  }

private:
  void cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (saved_)
      return;

    try {
      // Obtain the Mat without copying by not specifying encoding
      cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg);
      cv::Mat img = cvp->image;

      // Convert to BGR for uniform saving
      if (msg->encoding == "rgb8") {
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == "mono8") {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
      } // bgr8 直接用；其它编码可加更多分支

      auto now = std::chrono::system_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch())
                    .count();
      std::string path = "images/frame_" + std::to_string(ms) + ".png";

      if (cv::imwrite(path, img)) {
        RCLCPP_INFO(get_logger(), "Saved: %s  (%dx%d)", path.c_str(), img.cols,
                    img.rows);
        saved_ = true;
        rclcpp::shutdown();
      } else {
        RCLCPP_ERROR(get_logger(), "cv::imwrite failed: %s", path.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "raw decode failed: %s", e.what());
      // Do not set the saved flag; wait for the next frame
    }
  }

  std::string topic_;
  bool saved_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SaveOneRaw>());
  return 0;
}
