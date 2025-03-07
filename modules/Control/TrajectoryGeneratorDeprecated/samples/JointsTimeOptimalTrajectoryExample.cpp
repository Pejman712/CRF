/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <list>
#include <vector>
#include <string>
#include <memory>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "TrajectoryGeneratorDeprecated/IJointsTrajectoryGenerator.hpp"
#include "TrajectoryGeneratorDeprecated/JointsTimeOptimalTrajectory.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"

using crf::control::trajectorygeneratordeprecated::IJointsTrajectoryGenerator;
using crf::control::trajectorygeneratordeprecated::JointsTimeOptimalTrajectory;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;

int main(int argc, char** argv) {
    crf::utility::logger::EventLogger logger("TrajectoryGeneratorExample");

    std::vector<JointPositions> path;
    path.push_back(JointPositions({0.0, 0.0, 0.0}));
    path.push_back(JointPositions({0.0, 0.2, 1.0}));
    path.push_back(JointPositions({0.0, 3.0, 0.5}));
    path.push_back(JointPositions({1.1, 2.0, 0.0}));
    path.push_back(JointPositions({1.0, 0.0, 0.0}));
    path.push_back(JointPositions({0.0, 1.0, 0.0}));
    path.push_back(JointPositions({0.0, 0.0, 1.0}));

    JointVelocities maxVelocity({1.0, 1.0, 1.0});
    JointAccelerations maxAcceleration({1.0, 1.0, 1.0});

    float timeStep = 0.01;
    float maxDeviation = 0.1;
    std::unique_ptr<IJointsTrajectoryGenerator> timeOptTrajec;
    try {
        timeOptTrajec.reset(new JointsTimeOptimalTrajectory(
            maxVelocity,
            maxAcceleration,
            timeStep,
            maxDeviation));
    } catch (const std::exception& e) {
        logger->error("Failed to read the configuration file {}", e.what());
        return -1;
    }

    if (!timeOptTrajec->computeTrajectory(path)) {
        logger->error("Failed to calculate the trajectory");
    }
    logger->info("Trajectory calculated");

    auto duration = timeOptTrajec->getDuration();
    if (duration ==  boost::none) {
        logger->error("Failed to calculate the trajectory");
    }
    logger->info("Trajectory duration = {}", duration.get());

    float time = 10.5;
    auto position = timeOptTrajec->getJointPositions(time);
    if (position ==  boost::none) {
        logger->error("Failed to calculate the trajectory");
    }
    logger->info("The position at {} is {}", time, position.get());

    auto velocity = timeOptTrajec->getJointVelocities(time);
    if (velocity ==  boost::none) {
        logger->error("Failed to calculate the trajectory");
    }
    logger->info("The velocity at {} is {}", time, velocity.get());

    auto trajectoryResult = *(timeOptTrajec->getJointsTrajectory());

    logger->info(" Time | Position | Velocity");
    for (int i=0; i < duration.get()/timeStep; i++) {
        logger->info("{} | {} {} {} | {} {} {}", trajectoryResult.time[i],
            trajectoryResult.position[i][0],
            trajectoryResult.position[i][1],
            trajectoryResult.position[i][2],
            trajectoryResult.velocity[i][0],
            trajectoryResult.velocity[i][1],
            trajectoryResult.velocity[i][2]);
    }
}
