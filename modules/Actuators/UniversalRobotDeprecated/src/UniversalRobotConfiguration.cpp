/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "UniversalRobot/UniversalRobotConfiguration.hpp"

namespace crf::actuators::universalrobot {

UniversalRobotConfiguration::UniversalRobotConfiguration() :
    RobotArmConfiguration(),
    ipAddress_(),
    lookAheadTime_(),
    gain_() {
}

bool UniversalRobotConfiguration::parse(const nlohmann::json& robotJSON) {
    logger_->debug("parse");
    cleanup();
    if (!RobotArmConfiguration::parse(robotJSON)) {
        return false;
    }
    try {
        ipAddress_ = robotJSON["IPAdress"].get<std::string>();
        lookAheadTime_ = robotJSON["lookAheadTime"].get<float>();
        gain_ = robotJSON["gain"].get<float>();
    } catch (std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    return true;
}

std::string UniversalRobotConfiguration::getIPAddress() {
    return ipAddress_;
}

float UniversalRobotConfiguration::getLookAheadTime() {
    return lookAheadTime_;
}

float UniversalRobotConfiguration::getGain() {
    return gain_;
}

void UniversalRobotConfiguration::cleanup() {
    RobotArmConfiguration::cleanup();
    ipAddress_ = std::string();
    lookAheadTime_ = 0.0f;
    gain_ = 0.0f;
}

}  // namespace crf::actuators::universalrobot
