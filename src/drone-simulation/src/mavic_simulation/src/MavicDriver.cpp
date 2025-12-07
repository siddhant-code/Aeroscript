#include "mavic_simulation/MavicDriver.hpp"

#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <algorithm>
#include <cstdio>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#define K_VERTICAL_THRUST 68.5
#define K_VERTICAL_P 3.0
#define K_ROLL_P 50.0
#define K_PITCH_P 30.0
#define K_YAW_P 2.0
#define K_X_VELOCITY_P 1
#define K_Y_VELOCITY_P 1
#define K_X_VELOCITY_I 0.01
#define K_Y_VELOCITY_I 0.01
#define LIFT_HEIGHT 1.0

namespace mavic_driver {
void MavicDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {
  timestep = static_cast<int>(wb_robot_get_basic_time_step());
  gps = wb_robot_get_device("gps");
  gyro = wb_robot_get_device("gyro");
  imu = wb_robot_get_device("inertial unit");
  propellers[0] = wb_robot_get_device("front right propeller");
  propellers[1] = wb_robot_get_device("front left propeller");
  propellers[2] = wb_robot_get_device("rear right propeller");
  propellers[3] = wb_robot_get_device("rear left propeller");

  for (int i = 0; i < 4; i++) {
    wb_motor_set_position(propellers[i], INFINITY);
    wb_motor_set_velocity(propellers[i], 0.0);
  }

  vertical_ref = LIFT_HEIGHT;
  linear_x_integral = 0.0;
  linear_y_integral = 0.0;
  name = std::string(wb_robot_get_name());
  std::string topic_name = name + "/cmd_vel";

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      topic_name, rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->target_twist.linear = msg->linear;
        this->target_twist.angular = msg->angular;
      });
}

void MavicDriver::step() {
  double roll_ref = 0.0;
  double pitch_ref = 0.0;
  const double *roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(imu);
  double roll = roll_pitch_yaw[0];
  double pitch = roll_pitch_yaw[1];
  const double *gps_values = wb_gps_get_values(gps);
  double vertical = gps_values[2];
  const double *gyro_values = wb_gyro_get_values(gyro);
  double roll_velocity = gyro_values[0];
  double pitch_velocity = gyro_values[1];
  double twist_yaw = gyro_values[2];
  double velocity = wb_gps_get_speed(gps);

  if (std::isnan(velocity)) {
    return;
  }

  if (vertical > 0.2) {
    double velocity_x = (pitch / (std::abs(roll) + std::abs(pitch))) * velocity;
    double velocity_y = -(roll / (std::abs(roll) + std::abs(pitch))) * velocity;

    double linear_y_error = this->target_twist.linear.y - velocity_y;
    double linear_x_error = this->target_twist.linear.x - velocity_x;

    linear_x_integral += linear_x_error;
    linear_y_integral += linear_y_error;
    roll_ref =
        K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * linear_y_integral;
    pitch_ref =
        -K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * linear_x_integral;
    vertical_ref = std::clamp(
        vertical_ref + this->target_twist.linear.z * (timestep / 1000.0),
        std::max(vertical - 0.5, LIFT_HEIGHT), vertical + 0.5);
  }

  double vertical_input = K_VERTICAL_P * (vertical_ref - vertical);
  double yaw_ref = this->target_twist.angular.z;

  double roll_input =
      K_ROLL_P * std::clamp(roll, -1.0, 1.0) + roll_velocity + roll_ref;
  double pitch_input =
      K_PITCH_P * std::clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_ref;
  double yaw_input = K_YAW_P * (yaw_ref - twist_yaw);

  double m1 =
      K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input;
  double m2 =
      K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input;
  double m3 =
      K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input;
  double m4 =
      K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input;

  wb_motor_set_velocity(propellers[0], -m1);
  wb_motor_set_velocity(propellers[1], m2);
  wb_motor_set_velocity(propellers[2], m3);
  wb_motor_set_velocity(propellers[3], -m4);
}
}  // namespace mavic_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mavic_driver::MavicDriver,
                       webots_ros2_driver::PluginInterface)