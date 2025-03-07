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

#include "Robot/UniversalRobot/UniversalRobotConfiguration.hpp"

namespace crf::actuators::robot {

UniversalRobotConfiguration::UniversalRobotConfiguration(const nlohmann::json& robotConfig) :
    RobotConfiguration(robotConfig),
    logger_("UniversalRobotConfiguration") {
        logger_->debug("CTor");
        parse(robotConfig);
}

std::string UniversalRobotConfiguration::getIPAddress() const {
    return ipAddress_;
}

double UniversalRobotConfiguration::getLookAheadTime() const {
    return lookAheadTime_;
}

double UniversalRobotConfiguration::getGain() const {
    return gain_;
}

// Private

void UniversalRobotConfiguration::parse(const nlohmann::json& robotConfig) {
    logger_->debug("parse");
    try {
        ipAddress_ = robotConfig["IPAddress"].get<std::string>();
        lookAheadTime_ = robotConfig["LookAheadTime"].get<double>();
        gain_ = robotConfig["Gain"].get<double>();
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        throw std::invalid_argument(
            "The configuration file provided could not be parsed in UniversalRobotConfiguration");
    }
}

}  // namespace crf::actuators::robot
