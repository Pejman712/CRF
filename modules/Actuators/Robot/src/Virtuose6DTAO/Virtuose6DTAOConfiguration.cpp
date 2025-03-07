/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "Robot/Virtuose6DTAO/Virtuose6DTAOConfiguration.hpp"

namespace crf::actuators::robot {

Virtuose6DTAOConfiguration::Virtuose6DTAOConfiguration(const nlohmann::json& robotConfig) :
    RobotConfiguration(robotConfig),
    ipAddress_(),
    logger_("Virtuose6DTAOConfiguration") {
    logger_->debug("CTor");
    parse(robotConfig);
}

void Virtuose6DTAOConfiguration::parse(const nlohmann::json& robotConfig) {
    logger_->debug("parse");
    try {
        ipAddress_ = robotConfig["IPAddress"].get<std::string>();
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        throw std::invalid_argument(std::string(
            "The configuration file provided could not be parsed in RobotConfiguration: ")
            + e.what());
    }
    return;
}

std::string Virtuose6DTAOConfiguration::getIPAddress() {
    return ipAddress_;
}

}  // namespace crf::actuators::robot
