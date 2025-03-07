/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "YoubotArm/YoubotArmConfiguration.hpp"

namespace crf {
namespace robots {
namespace youbot {

YoubotArmConfiguration::YoubotArmConfiguration() :
    RobotArmConfiguration(),
    serialNumber_() {
}

bool YoubotArmConfiguration::parse(const nlohmann::json& robotJSON) {
    cleanup();
    if (!RobotArmConfiguration::parse(robotJSON)) {
        return false;
    }
    if ((robotData.rdstate() & std::ifstream::failbit) != 0) {
        return false;
    }
    try {
        youbotConfigurationFile_ =  robotJSON.at("YoubotConfiguration").get<std::string>();
    } catch (std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    return true;
}

std::string YoubotArmConfiguration::getYoubotConfigurationFile() {
    return youbotConfigurationFile_;
}

void YoubotArmConfiguration::cleanup() {
    RobotArmConfiguration::cleanup();
    youbotConfigurationFile_ = "";
}

}  // namespace youbot
}  // namespace robots
}  // namespace crf
