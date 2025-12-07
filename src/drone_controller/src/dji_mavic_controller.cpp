/**
 * @file dji_mavic_controller.cpp
 * @author Siddhant (iamsid@umd.edu)
 * @brief
 * @version 0.1
 * @date 2025-12-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "orchestrator.hpp"

using std::placeholders::_1;
using POS_SUBSCRIBER =
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr;
using VEL_SUBSCRIBER =
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr;
using VEL_PUBLISHER = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using namespace std::chrono_literals;
using TIMER = rclcpp::TimerBase::SharedPtr;

/**
 * @brief Get the Goals For Letter object
 *
 * @param letter The letter to get goals for
 * @param csv_path The path to the CSV file containing letter goals
 * @return std::vector<RVO::Vector3>
 */
std::vector<RVO::Vector3> GetGoalsForLetter(char letter,
                                            const std::string &csv_path) {
  letter = std::toupper(letter);
  std::vector<RVO::Vector3> result;

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    std::cerr << "Could not open CSV: " << csv_path << std::endl;
    return result;
  }

  std::string line;
  std::getline(file, line);  // skip header

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;

    char row_letter;
    float x, y, z;
    // read letter
    std::getline(ss, token, ',');
    row_letter = token[0];
    if (std::toupper(row_letter) != letter) continue;
    // read x
    std::getline(ss, token, ',');
    x = std::stof(token) - 20.0f;  // center the letter
    // read y
    std::getline(ss, token, ',');
    y = std::stof(token);
    // read z
    std::getline(ss, token, ',');
    z = std::stof(token);
    result.push_back(RVO::Vector3(x, y, z));
  }
  return result;
}

/**
 * @brief Minimal Subscriber class to manage drone positions and velocities
 *
 */
class DroneManager : public rclcpp::Node {
 public:
  DroneManager() : Node("drone_position_manager") {
    this->declare_parameter("text", "WIND");
    std::string text_to_write_ = this->get_parameter("text").as_string();
    RCLCPP_INFO(this->get_logger(), "Text to write: '%s'",
                text_to_write_.c_str());
    // Populate the letters queue
    for (char &letter : text_to_write_) {
      letter = std::toupper(letter);
      letters_queue_.push(letter);
    }
    this->declare_parameter("csv_file", "");
    std::string csv_path_param = this->get_parameter("csv_file").as_string();

    std::string csv_path;
    if (csv_path_param.empty()) {
      // Fallback to installed location (for safety)
      std::string pkg_share_dir =
          ament_index_cpp::get_package_share_directory("drone_controller");
      csv_path = pkg_share_dir + "/config/letters_AZ_cad.csv";
    } else {
      // Use the provided parameter
      csv_path = csv_path_param;
    }
    // Get initial goals for the first letter
    goals_ = GetGoalsForLetter(letters_queue_.front(), csv_path);

    letters_queue_.pop();
    orchestrator_.SetWebotDroneGoals(goals_);
    // Create subscribers and publishers for each drone
    for (int idx = 0; idx < num_drones_; idx++) {
      std::string topic_name =
          "Mavic_2_PRO_" + std::to_string(idx + 1) + "/gps";
      std::function<void(const geometry_msgs::msg::PointStamped &msg)>
          callback_pos =
              std::bind(&DroneManager::PositionCallback, this, _1, idx);
      position_subscribers_[idx] =
          this->create_subscription<geometry_msgs::msg::PointStamped>(
              topic_name, 10, callback_pos);
      std::string topic_name_vel_sub = topic_name + "/speed_vector";
      std::function<void(const geometry_msgs::msg::Vector3 &msg)> callback_vel =
          std::bind(&DroneManager::VelocityCallback, this, _1, idx);
      velocity_subscribers_[idx] =
          this->create_subscription<geometry_msgs::msg::Vector3>(
              topic_name_vel_sub, 10, callback_vel);
      std::string topic_name_vel_pub =
          "Mavic_2_PRO_" + std::to_string(idx + 1) + "/cmd_vel";
      velocity_publishers_[idx] =
          this->create_publisher<geometry_msgs::msg::Twist>(topic_name_vel_pub,
                                                            10);
      timer_ = this->create_wall_timer(
          8ms, std::bind(&DroneManager::TimerCallback, this));
    }
  }

 private:
  /**
   * @brief Callback function for drone position updates
   *
   * @param msg The position message
   * @param idx The index of the drone
   */
  void PositionCallback(const geometry_msgs::msg::PointStamped &msg, int idx) {
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.point.x);
    drone_positions_[idx] = RVO::Vector3(msg.point.x, msg.point.y, msg.point.z);
  }
  /**
   * @brief Callback function for drone velocity updates
   *
   * @param msg The velocity message
   * @param idx The index of the drone
   */
  void VelocityCallback(const geometry_msgs::msg::Vector3 &msg, int idx) {
    // RCLCPP_INFO(this->get_logger(), "I heard velocity: '%f'", msg.x);
    drone_velocities_[idx] = RVO::Vector3(msg.x, msg.y, msg.z);
  }

  /**
   * @brief Timer callback to update drone control commands
   *
   */
  void TimerCallback() {
    // Check if all drones have reached their goals
    if (orchestrator_.ReachedGoals()) {
      RCLCPP_INFO(this->get_logger(), "All drones reached goals");
      for (int idx = 0; idx < num_drones_; idx++) {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        velocity_publishers_[idx]->publish(message);
      }
      // If there are more letters to write, set new goals
      if (!letters_queue_.empty()) {
        char next_letter = letters_queue_.front();
        letters_queue_.pop();
        RCLCPP_INFO(this->get_logger(), "Setting new letter goals: '%c'",
                    next_letter);
        // Reuse the same csv_path from constructor
        std::string csv_path;
        std::string csv_path_param =
            this->get_parameter("csv_file").as_string();
        if (csv_path_param.empty()) {
          std::string pkg_share_dir =
              ament_index_cpp::get_package_share_directory("drone_controller");
          csv_path = pkg_share_dir + "/config/letters_AZ_cad.csv";
        } else {
          csv_path = csv_path_param;
        }
        goals_ = GetGoalsForLetter(next_letter, csv_path);
        orchestrator_.SetWebotDroneGoals(goals_);
      }
      return;
    }
    // Update drone positions and velocities in the orchestrator
    orchestrator_.SetWebotDronePositionsAndVelocities(drone_positions_,
                                                      drone_velocities_);
    orchestrator_.SetWebotPrefferedVelocities();
    auto new_velocities = orchestrator_.GetNewVelocities();
    // Publish new velocity commands to each drone
    for (int idx = 0; idx < num_drones_; idx++) {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = -new_velocities[idx].x();
      message.linear.y = -new_velocities[idx].y();
      message.linear.z = new_velocities[idx].z();
      velocity_publishers_[idx]->publish(message);
    }
  }
  /**
   * @brief The number of drones being controlled
   *
   */
  int num_drones_ = 20;
  /**
   * @brief Subscribers for drone positions
   *
   */
  std::vector<POS_SUBSCRIBER> position_subscribers_ =
      std::vector<POS_SUBSCRIBER>(num_drones_);
  /**
   * @brief Subscribers for drone velocities
   *
   */
  std::vector<VEL_SUBSCRIBER> velocity_subscribers_ =
      std::vector<VEL_SUBSCRIBER>(num_drones_);
  /**
   * @brief Positions of the drones
   *
   */
  std::vector<RVO::Vector3> drone_positions_ =
      std::vector<RVO::Vector3>(num_drones_);
  /**
   * @brief Velocities of the drones
   *
   */
  std::vector<RVO::Vector3> drone_velocities_ =
      std::vector<RVO::Vector3>(num_drones_);
  /**
   * @brief Publishers for drone velocity commands
   *
   */
  std::vector<VEL_PUBLISHER> velocity_publishers_ =
      std::vector<VEL_PUBLISHER>(num_drones_);
  /**
   * @brief Timer for periodic updates
   *
   */
  TIMER timer_;
  /**
   * @brief Orchestrator instance for managing drone coordination
   *
   */
  Orchestrator orchestrator_ = Orchestrator(drone_positions_);
  /**
   * @brief Queue of letters to write
   *
   */
  std::queue<char> letters_queue_;
  /**
   * @brief Current goals for the drones
   *
   */
  std::vector<RVO::Vector3> goals_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneManager>());
  rclcpp::shutdown();
  return 0;
}
