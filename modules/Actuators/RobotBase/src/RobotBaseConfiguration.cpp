/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <exception>
#include <fstream>
#include <string>
#include <vector>

#include "RobotBase/RobotBaseConfiguration.hpp"

namespace crf::actuators::robotbase {

RobotBaseConfiguration::RobotBaseConfiguration():
    logger_("RobotBaseConfiguration"),
    botConfig_(),
    taskLimits_(),
    rtLoopTimeUs_(0) {
        logger_->debug("CTor");
    }

bool RobotBaseConfiguration::parse(const nlohmann::json& robotJSON) {
    logger_->debug("parse");
    try {
        logger_->info("Reading configuration file of {}", robotJSON.at("Name").get<std::string>());
        rtLoopTimeUs_ = robotJSON.at("LoopTimeUs").get<int64_t>();
        botConfig_.hasLiftingStage = robotJSON.at("hasLiftingStage").get<int>();
        // TODO(jukabala) : Change parsing and config file to adapt to case when wheels are not all the same NOLINT
        botConfig_.wheelsCount = robotJSON.at("NumberOfWheels").get<uint8_t>();
        botConfig_.wheelsDistanceX = robotJSON.at("Wheels").at("PositionX").get<float>();
        botConfig_.wheelsDistanceY = robotJSON.at("Wheels").at("PositionY").get<float>();
        botConfig_.wheelsDiameter = robotJSON.at("Wheels").at("WheelsDiameter").get<float>();
        botConfig_.maximumWheelsVelocity = robotJSON.at("Wheels")
            .at("MaximumVelocity").get<float>();
        botConfig_.maximumWheelsAcceleration = robotJSON.at("Wheels")
            .at("MaximumAcceleration").get<float>();

        taskLimits_.maximumVelocity = robotJSON.at("TaskLimits").at("MaximumVelocity").get<utility::types::TaskVelocity>();  // NOLINT
        taskLimits_.maximumAcceleration = robotJSON.at("TaskLimits").at("MaximumAcceleration").get<utility::types::TaskAcceleration>();  // NOLINT
    } catch (const std::exception& e) {
        botConfig_ = {};
        taskLimits_ = {};
        logger_->warn("Failed to parse because: {}", e.what());
        return false;
    }
    return true;
}

uint8_t RobotBaseConfiguration::getNumberOfWheels() const {
    return botConfig_.wheelsCount;
}

RobotParameters RobotBaseConfiguration::getRobotParameters() const {
    return botConfig_;
}

TaskLimits RobotBaseConfiguration::getTaskLimits() const {
    return taskLimits_;
}

uint64_t RobotBaseConfiguration::getRTLoopTime() const {
    return rtLoopTimeUs_;
}

}  // namespace crf::actuators::robotbase
