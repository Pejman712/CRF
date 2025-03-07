/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <list>
#include <vector>
#include <string>
#include <memory>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "TrajectoryGeneratorDeprecated/TaskLinearTrajectory.hpp"
#include "TrajectoryGeneratorDeprecated/TrajectoryData.hpp"

int main(int argc, char** argv) {
    std::vector<crf::utility::types::TaskPose> path;

    path.push_back(crf::utility::types::TaskPose(
        {1.0, 1.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0})));
    path.push_back(crf::utility::types::TaskPose(
        {1.0, 1.0, 1.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0})));
    path.push_back(crf::utility::types::TaskPose(
        {0.0, 1.0, 1.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0})));
    path.push_back(crf::utility::types::TaskPose(
        {0.0, 0.0, 1.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0})));
    path.push_back(crf::utility::types::TaskPose(
        {0.0, 0.0, 0.0}, crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0})));

    crf::utility::types::TaskVelocity maxVelocity({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration maxAcceleration({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    float timeStep = 0.01;

    auto timeOptTrajec =
        std::make_shared<crf::control::trajectorygeneratordeprecated::TaskLinearTrajectory>(
            maxVelocity, maxAcceleration, timeStep);

    if (!timeOptTrajec->computeTrajectory(path)) {
       std::puts("Failed to calculate the trajectory");
       return -1;
    }

    boost::optional<float> durationOpt = timeOptTrajec->getDuration();
    if (!durationOpt) {
        std::puts("Failed to calculate the trajectory");
        return -1;
    }
    float duration = durationOpt.get();

    std::vector<crf::utility::types::TaskPose> pos;
    std::vector<crf::utility::types::TaskVelocity> vel;
    float time = 0;
    time += timeStep;
    while (time < duration) {
        boost::optional<crf::utility::types::TaskPose> positionOpt =
            timeOptTrajec->getTaskPose(time);
        if (!positionOpt) {
            std::puts("Failed to calculate the trajectory");
        }
        crf::utility::types::TaskPose position = positionOpt.get();

        boost::optional<crf::utility::types::TaskVelocity> velocityOpt =
            timeOptTrajec->getTaskVelocity(time);
        if (!velocityOpt) {
            std::puts("Failed to calculate the trajectory");
        }
        crf::utility::types::TaskVelocity velocity = velocityOpt.get();

        pos.push_back(position);
        vel.push_back(velocity);

        time += timeStep;
    }

    std::ofstream recordedData;
    recordedData.open("trajectoryData.csv");
    recordedData << "TaskPose, TaskVelocity \n";
    for (int i = 0; i < pos.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            recordedData << pos.at(i).getPosition()(j) << ",";
        }
        for (size_t j = 0; j < 3; j++) {
            recordedData << pos.at(i).getCardanXYZ()[j] << ",";
        }
        for (size_t j = 0; j < 6; j++) {
            recordedData << vel.at(i)[j] << ",";
        }
        recordedData << "\n";
    }
}
