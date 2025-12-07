#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace mavic_driver {
class MavicDriver : public webots_ros2_driver::PluginInterface {
 public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

 private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  geometry_msgs::msg::Twist target_twist;
  std::array<WbDeviceTag, 4> propellers;
  WbDeviceTag gps;
  WbDeviceTag imu;
  WbDeviceTag gyro;
  float vertical_ref;
  float linear_x_integral;
  float linear_y_integral;
  int timestep;
  std::string name;
};
}  // namespace mavic_driver
#endif