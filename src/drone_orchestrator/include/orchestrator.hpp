/**
 * @file orchestrator.hpp
 * @author Siddhant(iamsid@umd.edu)
 * @brief
 * @version 0.1
 * @date 2025-12-06
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <RVO.h>

/**
 * @brief Orchestrator class to manage multiple drones using RVO
 *
 */
class Orchestrator {
 public:
  /**
   * @brief Construct a new Orchestrator object
   *
   * @param initialPos Initial positions of the drones
   * @param timeStep The time step of the simulation. Must be positive.
   * @param neighborDist The default maximal distance (center point to center
   * point) to other agents a new agent takes into account in the navigation.
   * The larger this number, the longer he running time of the simulation. If
   * the number is too low, the simulation will not be safe. Must be
   * non-negative.
   * @param maxNeighbors The default maximal number of other agents a new agent
   * takes into account in the navigation. The larger this number, the longer
   * the running time of the simulation. If the number is too low, the
   * simulation will not be safe.
   * @param timeHorizon The default minimal amount of time for which a new
   * agent's velocities that are computed by the simulation are safe with
   * respect to other agents. The larger this number, the sooner an agent will
   * respond to the presence of other agents, but the less freedom the agent has
   * in choosing its velocities. Must be positive.
   * @param radius The default radius of a new agent. Must be non-negative.
   * @param maxSpeed The default maximal speed of a new agent. Must be
   * non-negative.
   */
  Orchestrator(std::vector<RVO::Vector3> initialPos, float timeStep = 0.008f,
               float neighborDist = 3.0f, size_t maxNeighbors = 20,
               float timeHorizon = 3.5f, float radius = 0.4f,
               float maxSpeed = 0.5f);

  /**
   * @brief Set the Webot Drone Positions And Velocities
   *
   * @param positions The positions of the drones
   * @param velocities The velocities of the drones
   */
  void SetWebotDronePositionsAndVelocities(
      std::vector<RVO::Vector3>& positions,
      std::vector<RVO::Vector3>& velocities);

  /**
   * @brief Set the Webot Preffered Velocities for Drones
   *
   */
  void SetWebotPrefferedVelocities();
  /**
   * @brief Set the Webot Drone Goals
   *
   * @param goals The goal positions for the drones
   */
  void SetWebotDroneGoals(std::vector<RVO::Vector3>& goals);
  /**
   * @brief Check if all drones have reached their goals
   *
   * @return true if all drones have reached their goals
   * @return false otherwise
   */
  bool ReachedGoals();
  /**
   * @brief Get the New Velocities for the drones
   *
   * @return std::vector<RVO::Vector3> The new velocities for the drones
   */
  std::vector<RVO::Vector3> GetNewVelocities();
  /**
   * @brief Get the Webot Drone Goal for a specific drone
   *
   * @param drone_id The ID of the drone
   * @return RVO::Vector3 The goal position of the drone
   */
  RVO::Vector3 GetWebotDroneGoal(int drone_id);
  /**
   * @brief Get the Webot Drone Position for a specific drone
   *
   * @param drone_id The ID of the drone
   * @return RVO::Vector3 The position of the drone
   */
  RVO::Vector3 GetWebotDronePosition(int drone_id);
  /**
   * @brief Get the Webot Drone Velocity for a specific drone
   *
   * @param drone_id The ID of the drone
   * @return RVO::Vector3 The velocity of the drone
   */
  RVO::Vector3 GetWebotDroneVelocity(int drone_id);
  /**
   * @brief Get the Drone Preferred Velocity for a specific drone
   *
   * @param drone_id The ID of the drone
   * @return RVO::Vector3 The preferred velocity of the drone
   */
  RVO::Vector3 GetDronePreferredVelocity(int drone_id);
  /**
   * @brief Destroy the Orchestrator object
   *
   */
  ~Orchestrator();

 private:
  /**
   * @brief Pointer to the RVO Simulator
   *
   */
  RVO::RVOSimulator* simulator_;
  /**
   * @brief Number of drones being managed
   *
   */
  size_t num_drones_;
  /**
   * @brief Vectors representing the preferred velocities of the drones
   *
   */
  std::vector<RVO::Vector3> drone_preferred_velocities_;
  /**
   * @brief Vector representing Goals of the drones
   *
   */
  std::vector<RVO::Vector3> drone_goals_;
};