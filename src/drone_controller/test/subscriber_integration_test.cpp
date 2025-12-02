#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class SubscriberIntegrationTestFixture {
public:
    SubscriberIntegrationTestFixture() {
        tester_node = rclcpp::Node::make_shared("SubscriberIntegrationTestNode");
        
        tester_node->declare_parameter<double>("test_duration", 5.0);
        test_duration = tester_node->get_parameter("test_duration")
            .get_parameter_value().get<double>();
    }

protected:
    rclcpp::Node::SharedPtr tester_node;
    double test_duration;
};

TEST_CASE_METHOD(SubscriberIntegrationTestFixture, 
                 "Test position subscription and velocity publishing",
                 "[integration]") {
    
    // Track received messages
    std::vector<geometry_msgs::msg::Twist> received_velocities;
    
    // Subscribe to velocity commands that the controller publishes
    auto vel_subscriber = tester_node->create_subscription<geometry_msgs::msg::Twist>(
        "Mavic_2_PRO_1/cmd_vel",
        10,
        [&received_velocities, this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            received_velocities.push_back(*msg);
            RCLCPP_INFO(this->tester_node->get_logger(), "Received velocity: x=%.3f, y=%.3f, z=%.3f", 
                       msg->linear.x, msg->linear.y, msg->linear.z);
        }
    );
    
    // Publish position data that the controller subscribes to
    auto pos_publisher = tester_node->create_publisher<geometry_msgs::msg::PointStamped>(
        "Mavic_2_PRO_1/gps",
        10
    );
    
    auto vel_publisher = tester_node->create_publisher<geometry_msgs::msg::Vector3>(
        "Mavic_2_PRO_1/gps/speed_vector",
        10
    );
    
    // Publish test data
    geometry_msgs::msg::PointStamped pos_msg;
    pos_msg.header.frame_id = "world";
    pos_msg.header.stamp = tester_node->now();
    pos_msg.point.x = 0.0;
    pos_msg.point.y = 0.0;
    pos_msg.point.z = 10.0;
    
    geometry_msgs::msg::Vector3 vel_msg;
    vel_msg.x = 0.0;
    vel_msg.y = 0.0;
    vel_msg.z = 0.0;
    
    rclcpp::Rate rate(125.0);  // 125 Hz (8ms period)
    auto start_time = rclcpp::Clock().now();
    auto timeout = rclcpp::Duration::from_seconds(test_duration);
    
    while ((rclcpp::Clock().now() - start_time) < timeout) {
        pos_msg.header.stamp = tester_node->now();
        pos_publisher->publish(pos_msg);
        vel_publisher->publish(vel_msg);
        
        rclcpp::spin_some(tester_node);
        rate.sleep();
    }
    
    // Give some time for messages to be processed
    rclcpp::sleep_for(500ms);
    rclcpp::spin_some(tester_node);
    
    // Verify that velocity commands were published
    CHECK(received_velocities.size() > 0);
    
    // Verify velocity values are reasonable
    if (received_velocities.size() > 0) {
        const auto& vel = received_velocities.back();
        // Velocities should be within reasonable bounds
        CHECK(std::abs(vel.linear.x) <= 1.0);
        CHECK(std::abs(vel.linear.y) <= 1.0);
        CHECK(std::abs(vel.linear.z) <= 1.0);
    }
}

TEST_CASE_METHOD(SubscriberIntegrationTestFixture,
                 "Test multiple drone coordination",
                 "[integration]") {
    
    const int num_drones = 3;
    std::vector<std::vector<geometry_msgs::msg::Twist>> received_velocities(num_drones);
    
    // Create subscribers for each drone
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> vel_subs;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> pos_pubs;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr> vel_pubs;
    
    for (int i = 0; i < num_drones; ++i) {
        std::string drone_name = "Mavic_2_PRO_" + std::to_string(i + 1);
        
        // Subscribe to velocity commands
        vel_subs.push_back(tester_node->create_subscription<geometry_msgs::msg::Twist>(
            drone_name + "/cmd_vel",
            10,
            [&received_velocities, i, this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                received_velocities[i].push_back(*msg);
                RCLCPP_INFO(this->tester_node->get_logger(), "Drone %d: Received velocity: x=%.3f, y=%.3f, z=%.3f", 
                           i+1, msg->linear.x, msg->linear.y, msg->linear.z);
            }
        ));
        
        // Publish position and velocity data
        pos_pubs.push_back(tester_node->create_publisher<geometry_msgs::msg::PointStamped>(
            drone_name + "/gps",
            10
        ));
        
        vel_pubs.push_back(tester_node->create_publisher<geometry_msgs::msg::Vector3>(
            drone_name + "/gps/speed_vector",
            10
        ));
    }
    
    // Publish test data for all drones
    rclcpp::Rate rate(125.0);
    auto start_time = rclcpp::Clock().now();
    auto timeout = rclcpp::Duration::from_seconds(test_duration);
    
    while ((rclcpp::Clock().now() - start_time) < timeout) {
        for (int i = 0; i < num_drones; ++i) {
            geometry_msgs::msg::PointStamped pos_msg;
            pos_msg.header.frame_id = "world";
            pos_msg.header.stamp = tester_node->now();
            pos_msg.point.x = i * 1.0;
            pos_msg.point.y = 0.0;
            pos_msg.point.z = 10.0;
            
            geometry_msgs::msg::Vector3 vel_msg;
            vel_msg.x = 0.0;
            vel_msg.y = 0.0;
            vel_msg.z = 0.0;
            
            pos_pubs[i]->publish(pos_msg);
            vel_pubs[i]->publish(vel_msg);
        }
        
        rclcpp::spin_some(tester_node);
        rate.sleep();
    }
    
    // Give some time for messages to be processed
    rclcpp::sleep_for(500ms);
    rclcpp::spin_some(tester_node);
    
    // Verify all drones received velocity commands
    for (int i = 0; i < num_drones; ++i) {
        CHECK(received_velocities[i].size() > 0);
    }
}

TEST_CASE_METHOD(SubscriberIntegrationTestFixture,
                 "Test velocity command frequency",
                 "[integration]") {
    
    std::vector<rclcpp::Time> received_times;
    
    auto vel_subscriber = tester_node->create_subscription<geometry_msgs::msg::Twist>(
        "Mavic_2_PRO_1/cmd_vel",
        10,
        [&received_times, this](const geometry_msgs::msg::Twist::SharedPtr) {
            received_times.push_back(this->tester_node->now());
        }
    );
    
    auto pos_publisher = tester_node->create_publisher<geometry_msgs::msg::PointStamped>(
        "Mavic_2_PRO_1/gps",
        10
    );
    
    auto vel_publisher = tester_node->create_publisher<geometry_msgs::msg::Vector3>(
        "Mavic_2_PRO_1/gps/speed_vector",
        10
    );
    
    geometry_msgs::msg::PointStamped pos_msg;
    pos_msg.header.frame_id = "world";
    pos_msg.point.x = 0.0;
    pos_msg.point.y = 0.0;
    pos_msg.point.z = 10.0;
    
    geometry_msgs::msg::Vector3 vel_msg;
    vel_msg.x = 0.0;
    vel_msg.y = 0.0;
    vel_msg.z = 0.0;
    
    rclcpp::Rate rate(125.0);
    auto start_time = rclcpp::Clock().now();
    auto timeout = rclcpp::Duration::from_seconds(2.0);  // 2 seconds test
    
    while ((rclcpp::Clock().now() - start_time) < timeout) {
        pos_msg.header.stamp = tester_node->now();
        pos_publisher->publish(pos_msg);
        vel_publisher->publish(vel_msg);
        
        rclcpp::spin_some(tester_node);
        rate.sleep();
    }
    
    rclcpp::sleep_for(500ms);
    rclcpp::spin_some(tester_node);
    
    // Should receive multiple messages
    CHECK(received_times.size() > 10);
    
    // Calculate average frequency
    if (received_times.size() > 1) {
        auto total_duration = (received_times.back() - received_times.front()).seconds();
        double frequency = (received_times.size() - 1) / total_duration;
        
        // Should be close to 125 Hz (8ms period)
        CHECK(frequency > 50.0);  // At least 50 Hz
        CHECK(frequency < 200.0); // Not more than 200 Hz
    }
}

