/**
 * @file orchestrator.cpp
 * @author Siddhant (iamsid@umd.edu)
 * @brief
 * @version 0.1
 * @date 2025-12-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "orchestrator.hpp"

Orchestrator::Orchestrator(std::vector<RVO::Vector3> initialPos, float timeStep,
                           float neighborDist, size_t maxNeighbors,
                           float timeHorizon, float radius, float maxSpeed) {
  // Create RVO Simulator
  simulator_ = new RVO::RVOSimulator(timeStep, neighborDist, maxNeighbors,
                                     timeHorizon, radius, maxSpeed);
  // Set time step
  simulator_->setTimeStep(timeStep);
  // Add agents at initial positions
  num_drones_ = initialPos.size();
  for (size_t i = 0; i < num_drones_; ++i) {
    simulator_->addAgent(initialPos[i]);
  }
}

void Orchestrator::SetWebotDronePositionsAndVelocities(
    std::vector<RVO::Vector3>& positions,
    std::vector<RVO::Vector3>& velocities) {
  // Update drone positions and velocities in the simulator
  for (std::size_t i = 0; i < simulator_->getNumAgents(); ++i) {
    simulator_->setAgentPosition(i, positions[i]);
    simulator_->setAgentVelocity(i, velocities[i]);
  }
}

void Orchestrator::SetWebotPrefferedVelocities() {
  // Set preferred velocities towards goals
  for (std::size_t i = 0; i < simulator_->getNumAgents(); ++i) {
    RVO::Vector3 goalVector =
        drone_goals_[i] - Orchestrator::GetWebotDronePosition(i);
    // Normalize goal vector if its length is greater than 1
    if (RVO::absSq(goalVector) > 1.0f) {
      goalVector = RVO::normalize(goalVector);
    }
    simulator_->setAgentPrefVelocity(i, goalVector);
  }
}

void Orchestrator::SetWebotDroneGoals(std::vector<RVO::Vector3>& goals) {
  // Set the goal nearest to each drone
  auto newGoals = goals;
  for (std::size_t i = 0; i < simulator_->getNumAgents() - 1; ++i) {
    // Find the closest goal to drone i from the remaining goals
    auto dronePosition = Orchestrator::GetWebotDronePosition(i);
    float minDistance = std::numeric_limits<float>::max();
    int closestGoalIndex = -1;
    for (std::size_t j = i; j < goals.size(); ++j) {
      float distance = RVO::absSq(newGoals[j] - dronePosition);
      // Update closest goal if this one is nearer
      if (distance < minDistance) {
        minDistance = distance;
        closestGoalIndex = j;
      }
    }
    // Swap the closest goal to the current index
    std::swap(newGoals[i], newGoals[closestGoalIndex]);
  }
  drone_goals_ = newGoals;
}

bool Orchestrator::ReachedGoals() {
  // Check if all drones have reached their goals
  for (std::size_t i = 0; i < simulator_->getNumAgents(); ++i) {
    RVO::Vector3 goalVector =
        Orchestrator::GetWebotDronePosition(i) - drone_goals_[i];
    if (RVO::absSq(goalVector) > 0.5f) {
      return false;
    }
  }
  return true;
}

std::vector<RVO::Vector3> Orchestrator::GetNewVelocities() {
  // Compute new velocities for all drones
  Orchestrator::SetWebotPrefferedVelocities();
  // Perform a simulation step
  simulator_->doStep();
  std::vector<RVO::Vector3> newVelocities;
  for (std::size_t i = 0; i < simulator_->getNumAgents(); ++i) {
    newVelocities.push_back(Orchestrator::GetWebotDroneVelocity(i));
  }
  return newVelocities;
}

RVO::Vector3 Orchestrator::GetWebotDroneGoal(int drone_id) {
  return drone_goals_[drone_id];
}

RVO::Vector3 Orchestrator::GetWebotDronePosition(int drone_id) {
  return simulator_->getAgentPosition(drone_id);
}

RVO::Vector3 Orchestrator::GetWebotDroneVelocity(int drone_id) {
  return simulator_->getAgentVelocity(drone_id);
}

RVO::Vector3 Orchestrator::GetDronePreferredVelocity(int drone_id) {
  return simulator_->getAgentPrefVelocity(drone_id);
}

Orchestrator::~Orchestrator() { delete simulator_; }
