// Copyright 2016 Open Source Robotics Foundation, Inc.

//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "orchestrator.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cctype>
#include <queue>
#include <ament_index_cpp/get_package_share_directory.hpp>


using std::placeholders::_1;

using POS_SUBSCRIBER = rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr;
using VEL_SUBSCRIBER = rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr;
using VEL_PUBLISHER = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using namespace std::chrono_literals;
using TIMER     = rclcpp::TimerBase::SharedPtr;

std::vector<RVO::Vector3> getGoalsForLetter(char letter, const std::string &csv_path)
{
    letter = std::toupper(letter);
    std::vector<RVO::Vector3> result;

    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "Could not open CSV: " << csv_path << std::endl;
        return result;
    }

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        char row_letter;
        float x, y, z;

        // read letter
        std::getline(ss, token, ',');
        row_letter = token[0];

        if (std::toupper(row_letter) != letter)
            continue;

        // read x
        std::getline(ss, token, ',');
        x = std::stof(token) - 20.0f; // center the letter

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

class MinimalSubscriber : public rclcpp::Node {
public:

  MinimalSubscriber()
    : Node("drone_position_subscriber")
  {
    this->declare_parameter("text", "M");
    std::string text_to_write_ = this->get_parameter("text").as_string();
    RCLCPP_INFO(this->get_logger(), "Text to write: '%s'", text_to_write_.c_str());
    for (char& letter : text_to_write_)
    {
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

    goals = getGoalsForLetter(letters_queue_.front(), csv_path);

    letters_queue_.pop();
    orchestrator_.setWebotDroneGoals(goals);
    for (int idx = 0; idx < numSubs; idx++)
    {
      std::string topic_name = "Mavic_2_PRO_" + std::to_string(idx+1) + "/gps";
      
      std::function<void(const geometry_msgs::msg::PointStamped& msg)> callback_pos =
      std::bind(&MinimalSubscriber::position_callback, this, _1,idx);
      position_subscribers_[idx] = this->create_subscription<geometry_msgs::msg::PointStamped>(
        topic_name,
        10,
        callback_pos);

      std::string topic_name_vel_sub = topic_name + "/speed_vector";
      std::function<void(const geometry_msgs::msg::Vector3& msg)> callback_vel =
      std::bind(&MinimalSubscriber::velocity_callback, this, _1,idx);
      velocity_subscribers_[idx] = this->create_subscription<geometry_msgs::msg::Vector3>(
        topic_name_vel_sub,
        10,
        callback_vel);

        std::string topic_name_vel_pub = "Mavic_2_PRO_" + std::to_string(idx+1) + "/cmd_vel";
        velocity_publishers_[idx] = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_vel_pub, 10);
        timer_ = this->create_wall_timer(8ms,std::bind(&MinimalSubscriber::timer_callback, this));
        //goals.push_back(RVO::Vector3(1.0f*idx,1.0f* idx,10.0f));
        
    }
    
    
  }

private:

  void position_callback(const geometry_msgs::msg::PointStamped& msg, int idx )
  {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.point.x);
    drone_positions_[idx] = RVO::Vector3(msg.point.x, msg.point.y, msg.point.z);
  }
  void velocity_callback(const geometry_msgs::msg::Vector3& msg, int idx )
  {
    //RCLCPP_INFO(this->get_logger(), "I heard velocity: '%f'", msg.x);
    drone_velocities_[idx] = RVO::Vector3(msg.x, msg.y, msg.z);
  }

  void timer_callback()
  {
    if (orchestrator_.reachedGoals(goals))
    {
        RCLCPP_INFO(this->get_logger(), "All drones reached goals");
        for (int idx = 0; idx < numSubs; idx++)
          {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x  = 0.0;
            message.linear.y  = 0.0;
            message.linear.z  = 0.0;
            velocity_publishers_[idx]->publish(message);
          }
          if (!letters_queue_.empty())
              {
                  char next_letter = letters_queue_.front();
                  letters_queue_.pop();
                  RCLCPP_INFO(this->get_logger(), "Setting new letter goals: '%c'", next_letter);

                  // Reuse the same csv_path from constructor
                  std::string csv_path;
                  std::string csv_path_param = this->get_parameter("csv_file").as_string();
                  if (csv_path_param.empty()) {
                      std::string pkg_share_dir =
                          ament_index_cpp::get_package_share_directory("drone_controller");
                      csv_path = pkg_share_dir + "/config/letters_AZ_cad.csv";
                  } else {
                      csv_path = csv_path_param;
                  }

                  goals = getGoalsForLetter(letters_queue_.front(), csv_path);

                  orchestrator_.setWebotDroneGoals(goals);
              }
        return;
    }
    orchestrator_.setWebotDronePositionsAndVelocities(drone_positions_,drone_velocities_);
    orchestrator_.setWebotPrefferedVelocities();
    auto new_velocities = orchestrator_.getNewVelocities();
    for (int idx = 0; idx < numSubs; idx++)
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x  = -new_velocities[idx].x();
      message.linear.y  = -new_velocities[idx].y();
      message.linear.z  = new_velocities[idx].z();
      velocity_publishers_[idx]->publish(message);
    }
  }

  int numSubs                           = 20;
  std::vector<POS_SUBSCRIBER>position_subscribers_ = std::vector<POS_SUBSCRIBER>(numSubs);
  std::vector<VEL_SUBSCRIBER>velocity_subscribers_ = std::vector<VEL_SUBSCRIBER>(numSubs);
  std::vector<RVO::Vector3>drone_positions_= std::vector<RVO::Vector3>(numSubs);
  std::vector<RVO::Vector3>drone_velocities_= std::vector<RVO::Vector3>(numSubs);
  std::vector<VEL_PUBLISHER>velocity_publishers_ = std::vector<VEL_PUBLISHER>(numSubs);
  TIMER     timer_;
  Orchestrator orchestrator_ = Orchestrator(drone_positions_);
  std::queue<char> letters_queue_;
  std::vector<RVO::Vector3> goals;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
