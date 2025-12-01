#pragma once

#include <RVO.h>

class Orchestrator{
    public:
        Orchestrator(std::vector<RVO::Vector3> initial_pos,float timeStep = 0.008f, float neighborDist = 3.0f, size_t maxNeighbors = 20, float timeHorizon = 0.8f, float radius = 0.4f, float maxSpeed = 0.5f);
        void setWebotDronePositionsAndVelocities(std::vector<RVO::Vector3>& positions,std::vector<RVO::Vector3>& velocities);
        void setWebotPrefferedVelocities();
        void setWebotDroneGoals(std::vector<RVO::Vector3>& goals);
        bool reachedGoals(std::vector<RVO::Vector3> goals);
        std::vector<RVO::Vector3> getNewVelocities();
        RVO::Vector3 getWebotDroneGoal(int drone_id);
        RVO::Vector3 getWebotDronePosition(int drone_id);
        RVO::Vector3 getWebotDroneVelocity(int drone_id);
        RVO::Vector3 getDronePreferredVelocity(int drone_id);
        
        ~Orchestrator();
    private:
        RVO::RVOSimulator* simulator;
        size_t numDrones;
        std::vector<RVO::Vector3> dronePreferredVelocities;
        std::vector<RVO::Vector3> droneGoals;
   
};