#include <chrono>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

class ImuPrinter : public rclcpp::Node {
public:
  ImuPrinter() : rclcpp::Node("imu_printer") {
    // Allow overriding the topic name via a parameter
    imu_topic_ = this->declare_parameter<std::string>(
        "imu_topic",
        "/aima/hal/imu/chest/state"); // Change to other IMU topics, e.g.
                                      // waist IMU: /aima/hal/imu/torse/state
                                      // lidar IMU:
                                      // /aima/hal/sensor/lidar_chest_front/imu

    auto qos = rclcpp::SensorDataQoS(); // BestEffort / Volatile
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos,
        std::bind(&ImuPrinter::cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing IMU: %s", imu_topic_.c_str());
  }

private:
  void cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Compute receive interval (based on wall time, for observation only)
    auto now = std::chrono::steady_clock::now();
    double dt_ms =
        last_recv_.time_since_epoch().count() == 0
            ? 0.0
            : std::chrono::duration<double, std::milli>(now - last_recv_)
                  .count();
    last_recv_ = now;

    const auto &q = msg->orientation;
    const auto &w = msg->angular_velocity;
    const auto &a = msg->linear_acceleration;

    // Print
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "stamp=" << rclcpp::Time(msg->header.stamp).seconds()
        << "s  frame=" << msg->header.frame_id
        << "  recv_dt=" << std::setprecision(3) << dt_ms << "ms\n"
        << std::setprecision(6) << "  orientation (x,y,z,w): [" << q.x << ", "
        << q.y << ", " << q.z << ", " << q.w << "]\n"
        << "  angular_velocity (rad/s): [" << w.x << ", " << w.y << ", " << w.z
        << "]\n"
        << "  linear_accel (m/s^2): [" << a.x << ", " << a.y << ", " << a.z
        << "]";

    // Optional: print diagonal covariances (-1 means unknown)
    auto oc0 = msg->orientation_covariance[0];
    auto oc4 = msg->orientation_covariance[4];
    auto oc8 = msg->orientation_covariance[8];
    auto wc0 = msg->angular_velocity_covariance[0];
    auto wc4 = msg->angular_velocity_covariance[4];
    auto wc8 = msg->angular_velocity_covariance[8];
    auto ac0 = msg->linear_acceleration_covariance[0];
    auto ac4 = msg->linear_acceleration_covariance[4];
    auto ac8 = msg->linear_acceleration_covariance[8];

    oss << "\n  cov_diag(ori, ang, acc): [" << oc0 << "," << oc4 << "," << oc8
        << " | " << wc0 << "," << wc4 << "," << wc8 << " | " << ac0 << ","
        << ac4 << "," << ac8 << "]";

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  std::string imu_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::chrono::steady_clock::time_point last_recv_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPrinter>());
  rclcpp::shutdown();
  return 0;
}
