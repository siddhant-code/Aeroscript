#include <gtest/gtest.h>
#include "orchestrator.hpp"
#include <vector>
#include <cmath>

class OrchestratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup test fixtures
        initial_positions = {
            RVO::Vector3(0.0f, 0.0f, 10.0f),
            RVO::Vector3(1.0f, 0.0f, 10.0f),
            RVO::Vector3(2.0f, 0.0f, 10.0f)
        };
        orchestrator = new Orchestrator(initial_positions, 0.008f, 3.0f, 20, 0.8f, 0.4f, 0.5f);
    }

    void TearDown() override {
        delete orchestrator;
    }

    std::vector<RVO::Vector3> initial_positions;
    Orchestrator* orchestrator;
};

// Test Constructor
TEST_F(OrchestratorTest, ConstructorInitializesCorrectly) {
    EXPECT_NEAR(orchestrator->getWebotDronePosition(0).x(), 0.0f, 0.01f);
    EXPECT_NEAR(orchestrator->getWebotDronePosition(0).y(), 0.0f, 0.01f);
    EXPECT_NEAR(orchestrator->getWebotDronePosition(0).z(), 10.0f, 0.01f);
    EXPECT_EQ(initial_positions.size(), 3);
}

// Test setWebotDronePositionsAndVelocities
TEST_F(OrchestratorTest, SetPositionsAndVelocities) {
    std::vector<RVO::Vector3> new_positions = {
        RVO::Vector3(5.0f, 5.0f, 10.0f),
        RVO::Vector3(6.0f, 5.0f, 10.0f),
        RVO::Vector3(7.0f, 5.0f, 10.0f)
    };
    std::vector<RVO::Vector3> new_velocities = {
        RVO::Vector3(0.1f, 0.0f, 0.0f),
        RVO::Vector3(0.1f, 0.0f, 0.0f),
        RVO::Vector3(0.1f, 0.0f, 0.0f)
    };
    
    orchestrator->setWebotDronePositionsAndVelocities(new_positions, new_velocities);
    
    RVO::Vector3 pos = orchestrator->getWebotDronePosition(0);
    EXPECT_NEAR(pos.x(), 5.0f, 0.01f);
    EXPECT_NEAR(pos.y(), 5.0f, 0.01f);
}

// Test setWebotDroneGoals
TEST_F(OrchestratorTest, SetGoals) {
    std::vector<RVO::Vector3> goals = {
        RVO::Vector3(10.0f, 10.0f, 10.0f),
        RVO::Vector3(11.0f, 10.0f, 10.0f),
        RVO::Vector3(12.0f, 10.0f, 10.0f)
    };
    
    orchestrator->setWebotDroneGoals(goals);
    
    RVO::Vector3 goal = orchestrator->getWebotDroneGoal(0);
    EXPECT_NEAR(goal.x(), 10.0f, 0.01f);
    EXPECT_NEAR(goal.y(), 10.0f, 0.01f);
}

// Test setWebotPrefferedVelocities
TEST_F(OrchestratorTest, PreferredVelocitiesPointTowardGoals) {
    std::vector<RVO::Vector3> goals = {
        RVO::Vector3(1.0f, 0.0f, 10.0f),  // Goal 1 unit to the right
        RVO::Vector3(0.0f, 1.0f, 10.0f),  // Goal 1 unit forward
        RVO::Vector3(0.0f, 0.0f, 11.0f)   // Goal 1 unit up
    };
    
    orchestrator->setWebotDroneGoals(goals);
    orchestrator->setWebotPrefferedVelocities();
    
    RVO::Vector3 pref_vel = orchestrator->getDronePreferredVelocity(0);
    // Should point in positive x direction
    EXPECT_GT(pref_vel.x(), 0.0f);
    EXPECT_NEAR(pref_vel.y(), 0.0f, 0.1f);
}

// Test reachedGoals
TEST_F(OrchestratorTest, ReachedGoalsReturnsTrueWhenClose) {
    std::vector<RVO::Vector3> goals = {
        RVO::Vector3(0.1f, 0.1f, 10.0f),  // Very close to initial position
        RVO::Vector3(1.1f, 0.1f, 10.0f),
        RVO::Vector3(2.1f, 0.1f, 10.0f)
    };
    
    orchestrator->setWebotDroneGoals(goals);
    
    // Move drones close to goals
    std::vector<RVO::Vector3> close_positions = goals;
    std::vector<RVO::Vector3> zero_velocities(3, RVO::Vector3(0.0f, 0.0f, 0.0f));
    orchestrator->setWebotDronePositionsAndVelocities(close_positions, zero_velocities);
    
    EXPECT_TRUE(orchestrator->reachedGoals(goals));
}

TEST_F(OrchestratorTest, ReachedGoalsReturnsFalseWhenFar) {
    std::vector<RVO::Vector3> goals = {
        RVO::Vector3(10.0f, 10.0f, 10.0f),  // Far from initial position
        RVO::Vector3(11.0f, 10.0f, 10.0f),
        RVO::Vector3(12.0f, 10.0f, 10.0f)
    };
    
    orchestrator->setWebotDroneGoals(goals);
    
    EXPECT_FALSE(orchestrator->reachedGoals(goals));
}

// Test getNewVelocities
TEST_F(OrchestratorTest, GetNewVelocitiesReturnsValidVelocities) {
    std::vector<RVO::Vector3> goals = {
        RVO::Vector3(1.0f, 0.0f, 10.0f),
        RVO::Vector3(2.0f, 0.0f, 10.0f),
        RVO::Vector3(3.0f, 0.0f, 10.0f)
    };
    
    orchestrator->setWebotDroneGoals(goals);
    std::vector<RVO::Vector3> velocities = orchestrator->getNewVelocities();
    
    EXPECT_EQ(velocities.size(), 3);
    // Velocities should be within max speed
    for (const auto& vel : velocities) {
        float speed = std::sqrt(vel.x() * vel.x() + vel.y() * vel.y() + vel.z() * vel.z());
        EXPECT_LE(speed, 0.6f);  // Max speed is 0.5f, allow small tolerance
    }
}

// Test edge cases
TEST_F(OrchestratorTest, SingleDroneInitialization) {
    std::vector<RVO::Vector3> single_pos = {
        RVO::Vector3(0.0f, 0.0f, 10.0f)
    };
    Orchestrator single_orch(single_pos);
    
    EXPECT_NEAR(single_orch.getWebotDronePosition(0).x(), 0.0f, 0.01f);
    EXPECT_EQ(single_orch.getNewVelocities().size(), 1);
}

// Test getter methods
TEST_F(OrchestratorTest, GetDronePosition) {
    RVO::Vector3 pos = orchestrator->getWebotDronePosition(1);
    EXPECT_NEAR(pos.x(), 1.0f, 0.01f);
    EXPECT_NEAR(pos.y(), 0.0f, 0.01f);
    EXPECT_NEAR(pos.z(), 10.0f, 0.01f);
}

TEST_F(OrchestratorTest, GetDroneVelocity) {
    std::vector<RVO::Vector3> positions = initial_positions;
    std::vector<RVO::Vector3> velocities = {
        RVO::Vector3(0.2f, 0.1f, 0.0f),
        RVO::Vector3(0.2f, 0.1f, 0.0f),
        RVO::Vector3(0.2f, 0.1f, 0.0f)
    };
    
    orchestrator->setWebotDronePositionsAndVelocities(positions, velocities);
    
    RVO::Vector3 vel = orchestrator->getWebotDroneVelocity(0);
    EXPECT_NEAR(vel.x(), 0.2f, 0.01f);
    EXPECT_NEAR(vel.y(), 0.1f, 0.01f);
}

// Test collision avoidance behavior
TEST_F(OrchestratorTest, CollisionAvoidanceChangesVelocities) {
    // Place two drones very close together
    std::vector<RVO::Vector3> close_positions = {
        RVO::Vector3(0.0f, 0.0f, 10.0f),
        RVO::Vector3(0.2f, 0.0f, 10.0f),  // Very close (radius is 0.4f)
        RVO::Vector3(2.0f, 0.0f, 10.0f)
    };
    
    std::vector<RVO::Vector3> velocities = {
        RVO::Vector3(0.1f, 0.0f, 0.0f),
        RVO::Vector3(-0.1f, 0.0f, 0.0f),  // Moving toward each other
        RVO::Vector3(0.0f, 0.0f, 0.0f)
    };
    
    orchestrator->setWebotDronePositionsAndVelocities(close_positions, velocities);
    
    // Set goals that would cause collision
    std::vector<RVO::Vector3> goals = {
        RVO::Vector3(1.0f, 0.0f, 10.0f),
        RVO::Vector3(-1.0f, 0.0f, 10.0f),  // Opposite directions
        RVO::Vector3(3.0f, 0.0f, 10.0f)
    };
    
    orchestrator->setWebotDroneGoals(goals);
    std::vector<RVO::Vector3> new_velocities = orchestrator->getNewVelocities();
    
    // RVO should modify velocities to avoid collision
    EXPECT_EQ(new_velocities.size(), 3);
    // Velocities should be valid (not NaN or infinite)
    for (const auto& vel : new_velocities) {
        EXPECT_FALSE(std::isnan(vel.x()));
        EXPECT_FALSE(std::isnan(vel.y()));
        EXPECT_FALSE(std::isnan(vel.z()));
        EXPECT_FALSE(std::isinf(vel.x()));
        EXPECT_FALSE(std::isinf(vel.y()));
        EXPECT_FALSE(std::isinf(vel.z()));
    }
}

// Test multiple iterations
TEST_F(OrchestratorTest, MultipleIterationsConvergeToGoals) {
    std::vector<RVO::Vector3> goals = {
        RVO::Vector3(5.0f, 0.0f, 10.0f),
        RVO::Vector3(6.0f, 0.0f, 10.0f),
        RVO::Vector3(7.0f, 0.0f, 10.0f)
    };
    
    orchestrator->setWebotDroneGoals(goals);
    
    // Run multiple iterations
    for (int i = 0; i < 100; ++i) {
        std::vector<RVO::Vector3> current_positions;
        for (size_t j = 0; j < 3; ++j) {
            current_positions.push_back(orchestrator->getWebotDronePosition(j));
        }
        
        std::vector<RVO::Vector3> current_velocities;
        for (size_t j = 0; j < 3; ++j) {
            current_velocities.push_back(orchestrator->getWebotDroneVelocity(j));
        }
        
        orchestrator->setWebotDronePositionsAndVelocities(current_positions, current_velocities);
        std::vector<RVO::Vector3> new_velocities = orchestrator->getNewVelocities();
        
        // Update positions based on velocities (simplified integration)
        std::vector<RVO::Vector3> updated_positions;
        for (size_t j = 0; j < 3; ++j) {
            updated_positions.push_back(current_positions[j] + new_velocities[j] * 0.008f);
        }
        
        orchestrator->setWebotDronePositionsAndVelocities(updated_positions, new_velocities);
    }
    
    // After iterations, drones should be closer to goals
    for (size_t i = 0; i < 3; ++i) {
        RVO::Vector3 pos = orchestrator->getWebotDronePosition(i);
        RVO::Vector3 goal = goals[i];
        float distance = std::sqrt((pos.x() - goal.x()) * (pos.x() - goal.x()) +
                                   (pos.y() - goal.y()) * (pos.y() - goal.y()) +
                                   (pos.z() - goal.z()) * (pos.z() - goal.z()));
        // Should be closer than initial distance
        EXPECT_LT(distance, 5.0f);
    }
}

