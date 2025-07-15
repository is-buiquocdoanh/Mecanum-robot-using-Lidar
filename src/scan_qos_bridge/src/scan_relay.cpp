#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanRelay : public rclcpp::Node {
public:
  ScanRelay() : Node("scan_qos_relay") {
    rclcpp::QoS qos_in(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_in.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    rclcpp::QoS qos_out(10);
    qos_out.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_relay", qos_out);

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_in,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
          // Sử dụng clock hệ thống thực (không phụ thuộc /clock)
          msg->header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        }
        pub_->publish(*msg);
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRelay>());
  rclcpp::shutdown();
  return 0;
}
