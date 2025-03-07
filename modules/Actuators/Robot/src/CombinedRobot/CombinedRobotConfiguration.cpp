/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "Robot/CombinedRobot/CombinedRobotConfiguration.hpp"

namespace crf::actuators::robot {

using crf::utility::types::stdVectorFromEigenVector;

CombinedRobotConfiguration::CombinedRobotConfiguration(const nlohmann::json& robotConfig) :
    RobotConfiguration(robotConfig),
    logger_("CombinedRobotConfiguration") {
        logger_->debug("CTor");
        parse(robotConfig);
}

std::vector<nlohmann::json> CombinedRobotConfiguration::getRobotConfigFiles() const {
    logger_->debug("getRobotConfigFiles");
    return robotJSONs_;
}

uint64_t CombinedRobotConfiguration::getNumberOfRobots() const {
    logger_->debug("getNumberOfRobots");
    return numberOfRobots_;
}

std::vector<uint64_t> CombinedRobotConfiguration::getJointDimensionsOfRobots() const {
    logger_->debug("getJointDimensionsOfRobots {}", jointDimensions_.size());
    return jointDimensions_;
}

std::vector<uint64_t> CombinedRobotConfiguration::getTaskDimensionsOfRobots() const {
    logger_->debug("getTaskDimensionsOfRobots");
    return taskDimensions_;
}

// Private

void CombinedRobotConfiguration::parse(const nlohmann::json& robotConfig) {
    logger_->debug("parse");
    try {
        numberOfRobots_ = robotConfig["NumberOfRobots"].get<uint64_t>();
        jointDimensions_ = robotConfig["RobotJointDimensions"].get<std::vector<uint64_t>>();
        taskDimensions_ = robotConfig["RobotTaskDimensions"].get<std::vector<uint64_t>>();
        if (numberOfRobots_ != jointDimensions_.size() ||
            numberOfRobots_ != taskDimensions_.size() ||
            numberOfRobots_ != robotConfig["Robots"].size()) {
            throw std::invalid_argument(
                "Number of robots does not match with the size of the individual robots");
        }
        std::vector<uint64_t> jointDim = jointDimensions_;
        std::vector<uint64_t> taskDim = taskDimensions_;
        jointDim.insert(jointDim.begin(), 0);
        taskDim.insert(taskDim.begin(), 0);
        for (uint64_t i = 0; i < numberOfRobots_; i++) {
            nlohmann::json robotJSON;
            robotJSON = robotConfig["Robots"][i];
            robotJSON["ControllerLoopTimeMs"] = getRobotControllerLoopTime().count();
            robotJSON["JointSpaceDegreeOfFreedom"] = jointDim[i+1];
            robotJSON["TaskSpaceDegreeOfFreedom"] = taskDim[i+1];

            robotJSON["JointLimits"]["MaxPosition"] = getSubVector(
                stdVectorFromEigenVector(getJointLimits().maxPosition.raw()),
                jointDim[i], jointDim[i+1]);
            robotJSON["JointLimits"]["MinPosition"] = getSubVector(
                stdVectorFromEigenVector(getJointLimits().minPosition.raw()),
                jointDim[i], jointDim[i+1]);
            robotJSON["JointLimits"]["MaxVelocity"] = getSubVector(
                stdVectorFromEigenVector(getJointLimits().maxVelocity.raw()),
                jointDim[i], jointDim[i+1]);
            robotJSON["JointLimits"]["MaxAcceleration"] = getSubVector(
                stdVectorFromEigenVector(getJointLimits().maxAcceleration.raw()),
                jointDim[i], jointDim[i+1]);
            robotJSON["JointLimits"]["MaxTorque"] = getSubVector(
                stdVectorFromEigenVector(getJointLimits().maxTorque.raw()),
                jointDim[i], jointDim[i+1]);

            if (robotConfig["Robots"][i].contains("ProfileParameters")) {
                robotJSON["ProfileParameters"]["JointVelocities"] = getSubVector(
                    stdVectorFromEigenVector(getProfileParameters().jointVelocities.raw()),
                    jointDim[i], jointDim[i+1]);
                robotJSON["ProfileParameters"]["JointAccelerations"] = getSubVector(
                    stdVectorFromEigenVector(
                        getProfileParameters().jointAccelerations.raw()),
                    jointDim[i], jointDim[i+1]);
            }
            robotJSONs_.push_back(robotJSON);
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        throw std::invalid_argument(std::string(
            "The configuration file provided could not be parsed in CombinedRobotConfiguration")
            + e.what());
    }
}

std::vector<double> CombinedRobotConfiguration::getSubVector(
    const std::vector<double>& vec, const uint64_t& start, const uint64_t& count) {
    return std::vector<double>(vec.begin() + start, vec.begin() + start + count);
}

}  // namespace crf::actuators::robot
