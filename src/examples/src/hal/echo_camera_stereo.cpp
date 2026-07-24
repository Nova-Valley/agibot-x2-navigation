#include <deque>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
#include <vector>

// OpenCV headers for image/video writing
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/**
 * @brief Example of subscribing to multiple topics for the stereo head camera
 *
 * You can select which topic type to subscribe to via the startup argument
 * --ros-args -p topic_type:=<type>:
 *   - left_rgb_image: left camera RGB image (sensor_msgs/Image)
 *   - left_rgb_image_compressed: left camera RGB compressed image
 * (sensor_msgs/CompressedImage)
 *   - left_camera_info: left camera intrinsic parameters
 * (sensor_msgs/CameraInfo)
 *   - right_rgb_image: right camera RGB image (sensor_msgs/Image)
 *   - right_rgb_image_compressed: right camera RGB compressed image
 * (sensor_msgs/CompressedImage)
 *   - right_camera_info: right camera intrinsic parameters
 * (sensor_msgs/CameraInfo)
 *
 * Examples:
 *   ros2 run examples echo_camera_stereo --ros-args -p
 * topic_type:=left_rgb_image ros2 run examples echo_camera_stereo --ros-args -p
 * topic_type:=right_rgb_image ros2 run examples echo_camera_stereo --ros-args
 * -p topic_type:=left_camera_info
 *
 * topic_type defaults to "left_rgb_image"
 *
 * See individual callbacks for more detailed comments
 */
class StereoCameraTopicEcho : public rclcpp::Node {
public:
  StereoCameraTopicEcho() : Node("stereo_camera_topic_echo") {
    // Select which topic type to subscribe to
    topic_type_ =
        declare_parameter<std::string>("topic_type", "left_rgb_image");
    dump_video_path_ = declare_parameter<std::string>("dump_video_path", "");

    // Subscribed topics and their message layouts
    // 1. /aima/hal/sensor/stereo_head_front_left/rgb_image
    //    - topic_type: left_rgb_image
    //    - message type: sensor_msgs::msg::Image
    //    - frame_id: stereo_head_front
    //    - child_frame_id: /
    //    - contents: left camera raw image
    // 2. /aima/hal/sensor/stereo_head_front_left/rgb_image/compressed
    //    - topic_type: left_rgb_image_compressed
    //    - message type: sensor_msgs::msg::CompressedImage
    //    - frame_id: stereo_head_front
    //    - contents: left camera compressed image
    // 3. /aima/hal/sensor/stereo_head_front_left/camera_info
    //    - topic_type: left_camera_info
    //    - message type: sensor_msgs::msg::CameraInfo
    //    - frame_id: stereo_head_front
    //    - contents: left camera intrinsic parameters
    // 4. /aima/hal/sensor/stereo_head_front_right/rgb_image
    //    - topic_type: right_rgb_image
    //    - message type: sensor_msgs::msg::Image
    //    - frame_id: stereo_head_front_right
    //    - child_frame_id: /
    //    - contents: right camera raw image
    // 5. /aima/hal/sensor/stereo_head_front_right/rgb_image/compressed
    //    - topic_type: right_rgb_image_compressed
    //    - message type: sensor_msgs::msg::CompressedImage
    //    - frame_id: stereo_head_front_right
    //    - contents: right camera compressed image
    // 6. /aima/hal/sensor/stereo_head_front_right/camera_info
    //    - topic_type: right_camera_info
    //    - message type: sensor_msgs::msg::CameraInfo
    //    - frame_id: stereo_head_front_right
    //    - contents: right camera intrinsic parameters

    // Set QoS parameters - use SensorData QoS
    auto qos = rclcpp::SensorDataQoS();

    // Enable left camera RGB image subscription
    if (topic_type_ == "left_rgb_image") {
      topic_name_ = "/aima/hal/sensor/stereo_head_front_left/rgb_image";
      sub_image_ = create_subscription<sensor_msgs::msg::Image>(
          topic_name_, qos,
          std::bind(&StereoCameraTopicEcho::cb_image, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing Left RGB Image: %s",
                  topic_name_.c_str());
      if (!dump_video_path_.empty()) {
        RCLCPP_INFO(get_logger(), "📝 Will dump received images to video: %s",
                    dump_video_path_.c_str());
      }

      // Enable left camera RGB compressed image subscription
    } else if (topic_type_ == "left_rgb_image_compressed") {
      topic_name_ =
          "/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed";
      sub_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          topic_name_, qos,
          std::bind(&StereoCameraTopicEcho::cb_compressed, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing Left CompressedImage: %s",
                  topic_name_.c_str());

      // Enable left camera info subscription
    } else if (topic_type_ == "left_camera_info") {

      topic_name_ = "/aima/hal/sensor/stereo_head_front_left/camera_info";
      // CameraInfo subscriptions must use reliable + transient_local
      // QoS in order to receive latched/history messages (even if only one
      // message was published). Here we use keep_last(1) + reliable
      // + transient_local.
      sub_camerainfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          topic_name_,
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&StereoCameraTopicEcho::cb_camerainfo, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(),
                  "✅ Subscribing Left CameraInfo (with transient_local): %s",
                  topic_name_.c_str());

      // Enable right camera RGB image subscription
    } else if (topic_type_ == "right_rgb_image") {
      topic_name_ = "/aima/hal/sensor/stereo_head_front_right/rgb_image";
      sub_image_ = create_subscription<sensor_msgs::msg::Image>(
          topic_name_, qos,
          std::bind(&StereoCameraTopicEcho::cb_image, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing Right RGB Image: %s",
                  topic_name_.c_str());
      if (!dump_video_path_.empty()) {
        RCLCPP_INFO(get_logger(), "📝 Will dump received images to video: %s",
                    dump_video_path_.c_str());
      }

      // Enable right camera RGB compressed image subscription
    } else if (topic_type_ == "right_rgb_image_compressed") {
      topic_name_ =
          "/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed";
      sub_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
          topic_name_, qos,
          std::bind(&StereoCameraTopicEcho::cb_compressed, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing Right CompressedImage: %s",
                  topic_name_.c_str());

      // Enable right camera info subscription
    } else if (topic_type_ == "right_camera_info") {
      topic_name_ = "/aima/hal/sensor/stereo_head_front_right/camera_info";
      // CameraInfo subscriptions must use reliable + transient_local
      // QoS in order to receive latched/history messages (even if only one
      // message was published). Here we use keep_last(1) + reliable
      // + transient_local.
      sub_camerainfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          topic_name_,
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&StereoCameraTopicEcho::cb_camerainfo, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(),
                  "✅ Subscribing Right CameraInfo (with transient_local): %s",
                  topic_name_.c_str());

      // Unknown topic_type error
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown topic_type: %s", topic_type_.c_str());
      throw std::runtime_error("Unknown topic_type");
    }
  }

  ~StereoCameraTopicEcho() override {
    if (video_writer_.isOpened()) {
      video_writer_.release();
      RCLCPP_INFO(get_logger(), "Video file closed.");
    }
  }

private:
  // Image callback (left/right RGB image)
  void cb_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    update_arrivals();

    if (should_print()) {
      RCLCPP_INFO(get_logger(),
                  "📸 %s received\n"
                  "  • frame_id:        %s\n"
                  "  • stamp (sec):     %.6f\n"
                  "  • encoding:        %s\n"
                  "  • size (WxH):      %u x %u\n"
                  "  • step (bytes/row):%u\n"
                  "  • is_bigendian:    %u\n"
                  "  • recv FPS (1s):   %.1f",
                  topic_type_.c_str(), msg->header.frame_id.c_str(),
                  rclcpp::Time(msg->header.stamp).seconds(),
                  msg->encoding.c_str(), msg->width, msg->height, msg->step,
                  msg->is_bigendian, get_fps());
    }

    // Video dump is supported only for RGB images
    if ((topic_type_ == "left_rgb_image" || topic_type_ == "right_rgb_image") &&
        !dump_video_path_.empty()) {
      dump_image_to_video(msg);
    }
  }

  // CompressedImage callback (left/right RGB compressed image)
  void cb_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    update_arrivals();

    if (should_print()) {
      RCLCPP_INFO(get_logger(),
                  "🗜️  %s received\n"
                  "  • frame_id:        %s\n"
                  "  • stamp (sec):     %.6f\n"
                  "  • format:          %s\n"
                  "  • data size:       %zu\n"
                  "  • recv FPS (1s):   %.1f",
                  topic_type_.c_str(), msg->header.frame_id.c_str(),
                  rclcpp::Time(msg->header.stamp).seconds(),
                  msg->format.c_str(), msg->data.size(), get_fps());
    }
  }

  // CameraInfo callback (left/right camera intrinsic parameters)
  void cb_camerainfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // CameraInfo is typically published once; print it once
    std::ostringstream oss;
    oss << "📷 " << topic_type_ << " received\n"
        << "  • frame_id:        " << msg->header.frame_id << "\n"
        << "  • stamp (sec):     " << rclcpp::Time(msg->header.stamp).seconds()
        << "\n"
        << "  • width x height:  " << msg->width << " x " << msg->height << "\n"
        << "  • distortion_model:" << msg->distortion_model << "\n"
        << "  • D: [";
    for (size_t i = 0; i < msg->d.size(); ++i) {
      oss << msg->d[i];
      if (i + 1 < msg->d.size())
        oss << ", ";
    }
    oss << "]\n  • K: [";
    for (int i = 0; i < 9; ++i) {
      oss << msg->k[i];
      if (i + 1 < 9)
        oss << ", ";
    }
    oss << "]\n  • R: [";
    for (int i = 0; i < 9; ++i) {
      oss << msg->r[i];
      if (i + 1 < 9)
        oss << ", ";
    }
    oss << "]\n  • P: [";
    for (int i = 0; i < 12; ++i) {
      oss << msg->p[i];
      if (i + 1 < 12)
        oss << ", ";
    }
    oss << "]\n"
        << "  • binning_x: " << msg->binning_x << "\n"
        << "  • binning_y: " << msg->binning_y << "\n"
        << "  • roi: { x_offset: " << msg->roi.x_offset
        << ", y_offset: " << msg->roi.y_offset
        << ", height: " << msg->roi.height << ", width: " << msg->roi.width
        << ", do_rectify: " << (msg->roi.do_rectify ? "true" : "false") << " }";
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  // Track arrival timestamps to compute FPS
  void update_arrivals() {
    const rclcpp::Time now = this->get_clock()->now();
    arrivals_.push_back(now);
    while (!arrivals_.empty() && (now - arrivals_.front()).seconds() > 1.0) {
      arrivals_.pop_front();
    }
  }
  double get_fps() const { return static_cast<double>(arrivals_.size()); }

  // Control printing frequency
  bool should_print() {
    const rclcpp::Time now = this->get_clock()->now();
    if ((now - last_print_).seconds() >= 1.0) {
      last_print_ = now;
      return true;
    }
    return false;
  }

  // Dump received images to a video file (RGB images only)
  void dump_image_to_video(const sensor_msgs::msg::Image::SharedPtr &msg) {
    cv::Mat image;
    try {
      // Obtain the Mat without copying by not specifying encoding
      cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg);
      image = cvp->image;
      // Convert to BGR for uniform saving
      if (msg->encoding == "rgb8") {
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
      } else {
        RCLCPP_WARN(get_logger(), "image encoding not expected: %s",
                    msg->encoding.c_str());
        return;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Initialize VideoWriter
    if (!video_writer_.isOpened()) {
      int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
      double fps = std::max(1.0, get_fps());
      bool ok = video_writer_.open(dump_video_path_, fourcc, fps,
                                   cv::Size(image.cols, image.rows), true);
      if (!ok) {
        RCLCPP_ERROR(get_logger(), "Failed to open video file: %s",
                     dump_video_path_.c_str());
        dump_video_path_.clear(); // stop trying
        return;
      }
      RCLCPP_INFO(get_logger(), "VideoWriter started: %s, size=%dx%d, fps=%.1f",
                  dump_video_path_.c_str(), image.cols, image.rows, fps);
    }
    video_writer_.write(image);
  }

  // Member variables
  std::string topic_type_;
  std::string topic_name_;
  std::string dump_video_path_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      sub_compressed_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camerainfo_;

  // FPS statistics
  rclcpp::Time last_print_{0, 0, RCL_ROS_TIME};
  std::deque<rclcpp::Time> arrivals_;

  // Video writer
  cv::VideoWriter video_writer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoCameraTopicEcho>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
