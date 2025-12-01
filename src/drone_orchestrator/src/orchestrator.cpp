#include "orchestrator.hpp"

Orchestrator::Orchestrator(std::vector<RVO::Vector3> initial_pos,float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed) {
    simulator = new RVO::RVOSimulator(timeStep, neighborDist, maxNeighbors, timeHorizon, radius, maxSpeed);
    simulator->setTimeStep(timeStep);
    numDrones = initial_pos.size();
    for (size_t i = 0; i < numDrones; ++i) {
        simulator->addAgent(initial_pos[i]);
    }
}

void Orchestrator::setWebotDronePositionsAndVelocities(std::vector<RVO::Vector3>& positions,std::vector<RVO::Vector3>& velocities) {
   
    for (std::size_t i = 0; i < simulator->getNumAgents(); ++i) {
        simulator->setAgentPosition(i, positions[i]);
        simulator->setAgentVelocity(i, velocities[i]);
    }
}

void Orchestrator::setWebotPrefferedVelocities() {

    for (std::size_t i = 0; i < simulator->getNumAgents(); ++i) {
        RVO::Vector3 goalVector = droneGoals[i] - Orchestrator::getWebotDronePosition(i);
        if (RVO::absSq(goalVector) > 1.0f) {
            goalVector = RVO::normalize(goalVector);
        }
        simulator->setAgentPrefVelocity(i, goalVector);
    }
}

void Orchestrator::setWebotDroneGoals(std::vector<RVO::Vector3>& goals) {
    droneGoals = goals;
}

bool Orchestrator::reachedGoals(std::vector<RVO::Vector3> goals) {
   
    for (std::size_t i = 0; i < simulator->getNumAgents(); ++i) {
        RVO::Vector3 goalVector = Orchestrator::getWebotDronePosition(i)- goals[i];
        if (RVO::absSq(goalVector) > 0.5f) {
            return false;
        }
    }
    return true;
}

std::vector<RVO::Vector3> Orchestrator::getNewVelocities() {
    Orchestrator::setWebotPrefferedVelocities();
    simulator->doStep();
    std::vector<RVO::Vector3> newVelocities;
    for (std::size_t i = 0; i < simulator->getNumAgents(); ++i) {
        newVelocities.push_back(Orchestrator::getWebotDroneVelocity(i));
    }
    return newVelocities;
}

RVO::Vector3 Orchestrator::getWebotDroneGoal(int drone_id) {
    return droneGoals[drone_id];
}

RVO::Vector3 Orchestrator::getWebotDronePosition(int drone_id) {
    return simulator->getAgentPosition(drone_id);
}

RVO::Vector3 Orchestrator::getWebotDroneVelocity(int drone_id) {
    return simulator->getAgentVelocity(drone_id);
}

RVO::Vector3 Orchestrator::getDronePreferredVelocity(int drone_id) {
    return simulator->getAgentPrefVelocity(drone_id);
}

Orchestrator::~Orchestrator() {
    delete simulator;
}
