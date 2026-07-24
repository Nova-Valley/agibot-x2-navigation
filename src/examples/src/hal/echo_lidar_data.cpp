#include <deque>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <string>
#include <vector>

/**
 * @brief Example for subscribing to chest LIDAR data
 *
 * Supports subscribing to the following topics:
 *   1. /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
 *      - Data type: sensor_msgs::msg::PointCloud2
 *      - frame_id: lidar_chest_front
 *      - child_frame_id: /
 *      - Content: LIDAR point cloud data
 *   2. /aima/hal/sensor/lidar_chest_front/imu
 *      - Data type: sensor_msgs::msg::Imu
 *      - frame_id: lidar_imu_chest_front
 *      - Content: LIDAR IMU data
 *
 * You can select the topic type to subscribe to using the launch parameter
 * --ros-args -p topic_type:=<type>:
 *   - pointcloud: Subscribe to LIDAR point cloud
 *   - imu: Subscribe to LIDAR IMU
 * The default topic_type is pointcloud
 */
class LidarChestEcho : public rclcpp::Node {
public:
  LidarChestEcho() : Node("lidar_chest_echo") {
    topic_type_ = declare_parameter<std::string>("topic_type", "pointcloud");

    auto qos = rclcpp::SensorDataQoS();

    if (topic_type_ == "pointcloud") {
      topic_name_ = "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud";
      sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          topic_name_, qos,
          std::bind(&LidarChestEcho::cb_pointcloud, this,
                    std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing LIDAR PointCloud2: %s",
                  topic_name_.c_str());
    } else if (topic_type_ == "imu") {
      topic_name_ = "/aima/hal/sensor/lidar_chest_front/imu";
      sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
          topic_name_, qos,
          std::bind(&LidarChestEcho::cb_imu, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "✅ Subscribing LIDAR IMU: %s",
                  topic_name_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown topic_type: %s", topic_type_.c_str());
      throw std::runtime_error("Unknown topic_type");
    }
  }

private:
  // PointCloud2 callback
  void cb_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    update_arrivals();

    if (should_print()) {
      std::ostringstream oss;
      oss << "🟢 LIDAR PointCloud2 received\n"
          << "  • frame_id:        " << msg->header.frame_id << "\n"
          << "  • stamp (sec):     "
          << rclcpp::Time(msg->header.stamp).seconds() << "\n"
          << "  • width x height:  " << msg->width << " x " << msg->height
          << "\n"
          << "  • point_step:      " << msg->point_step << "\n"
          << "  • row_step:        " << msg->row_step << "\n"
          << "  • fields:          ";
      for (const auto &f : msg->fields)
        oss << f.name << "(" << (int)f.datatype << ") ";
      oss << "\n  • is_bigendian:    " << msg->is_bigendian
          << "\n  • is_dense:        " << msg->is_dense
          << "\n  • data size:       " << msg->data.size()
          << "\n  • recv FPS (1s):   " << get_fps();
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }
  }

  // IMU callback
  void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    update_arrivals();

    if (should_print()) {
      std::ostringstream oss;
      oss << "🟢 LIDAR IMU received\n"
          << "  • frame_id:        " << msg->header.frame_id << "\n"
          << "  • stamp (sec):     "
          << rclcpp::Time(msg->header.stamp).seconds() << "\n"
          << "  • orientation:     [" << msg->orientation.x << ", "
          << msg->orientation.y << ", " << msg->orientation.z << ", "
          << msg->orientation.w << "]\n"
          << "  • angular_velocity:[" << msg->angular_velocity.x << ", "
          << msg->angular_velocity.y << ", " << msg->angular_velocity.z << "]\n"
          << "  • linear_accel:    [" << msg->linear_acceleration.x << ", "
          << msg->linear_acceleration.y << ", " << msg->linear_acceleration.z
          << "]\n"
          << "  • recv FPS (1s):   " << get_fps();
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }
  }

  // Update FPS statistics
  void update_arrivals() {
    const rclcpp::Time now = this->get_clock()->now();
    arrivals_.push_back(now);
    while (!arrivals_.empty() && (now - arrivals_.front()).seconds() > 1.0) {
      arrivals_.pop_front();
    }
  }
  double get_fps() const { return static_cast<double>(arrivals_.size()); }

  // Control print frequency
  bool should_print() {
    const rclcpp::Time now = this->get_clock()->now();
    if ((now - last_print_).seconds() >= 1.0) {
      last_print_ = now;
      return true;
    }
    return false;
  }

  // Member variables
  std::string topic_type_;
  std::string topic_name_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_pointcloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  rclcpp::Time last_print_{0, 0, RCL_ROS_TIME};
  std::deque<rclcpp::Time> arrivals_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarChestEcho>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
