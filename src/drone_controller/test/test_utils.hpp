#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <RVO.h>
#include <vector>
#include <cmath>

namespace test_utils {

// Helper to create test positions
inline std::vector<RVO::Vector3> createTestPositions(int num_drones, float spacing = 1.0f) {
    std::vector<RVO::Vector3> positions;
    for (int i = 0; i < num_drones; ++i) {
        positions.push_back(RVO::Vector3(i * spacing, 0.0f, 10.0f));
    }
    return positions;
}

// Helper to create test goals
inline std::vector<RVO::Vector3> createTestGoals(int num_drones, float offset = 5.0f) {
    std::vector<RVO::Vector3> goals;
    for (int i = 0; i < num_drones; ++i) {
        goals.push_back(RVO::Vector3(i * 1.0f + offset, 0.0f, 10.0f));
    }
    return goals;
}

// Helper to convert RVO::Vector3 to PointStamped
inline geometry_msgs::msg::PointStamped toPointStamped(const RVO::Vector3& pos, int drone_id = 0) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 0;
    msg.point.x = pos.x();
    msg.point.y = pos.y();
    msg.point.z = pos.z();
    return msg;
}

// Helper to convert RVO::Vector3 to Vector3 message
inline geometry_msgs::msg::Vector3 toVector3Msg(const RVO::Vector3& vel) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = vel.x();
    msg.y = vel.y();
    msg.z = vel.z();
    return msg;
}

// Helper to check if positions are close
inline bool positionsClose(const RVO::Vector3& pos1, const RVO::Vector3& pos2, float threshold = 0.5f) {
    float dx = pos1.x() - pos2.x();
    float dy = pos1.y() - pos2.y();
    float dz = pos1.z() - pos2.z();
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance <= threshold;
}

// Helper to calculate distance between two positions
inline float distance(const RVO::Vector3& pos1, const RVO::Vector3& pos2) {
    float dx = pos1.x() - pos2.x();
    float dy = pos1.y() - pos2.y();
    float dz = pos1.z() - pos2.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Helper to create a grid of positions
inline std::vector<RVO::Vector3> createGridPositions(int rows, int cols, float spacing = 1.0f) {
    std::vector<RVO::Vector3> positions;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            positions.push_back(RVO::Vector3(col * spacing, row * spacing, 10.0f));
        }
    }
    return positions;
}

} // namespace test_utils

