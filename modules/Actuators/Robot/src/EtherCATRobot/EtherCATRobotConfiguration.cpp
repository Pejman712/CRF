/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: David Forkel CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "Robot/EtherCATRobot/EtherCATRobotConfiguration.hpp"

namespace crf::actuators::robot {

EtherCATRobotConfiguration::EtherCATRobotConfiguration(const nlohmann::json& robotConfig) :
    RobotConfiguration(robotConfig),
    gearBoxRatio_(1),
    logger_("EtherCATRobotConfiguration") {
    parse(robotConfig);
}

uint32_t EtherCATRobotConfiguration::getGearBoxReduction() const {
    return gearBoxRatio_;
}

uint32_t EtherCATRobotConfiguration::getMaxCurrent() const {
    return maxCurrent_;
}

uint32_t EtherCATRobotConfiguration::getMaxTorque() const {
    return maxTorque_;
}

double EtherCATRobotConfiguration::getRadToCountRatio() const {
    return radToCountRatio_;
}

// Private

void EtherCATRobotConfiguration::parse(const nlohmann::json& robotConfig) {
    logger_->debug("parse");
    try {
        // Mandatory
        if (!robotConfig.contains("MaxCurrent"))
            throw std::invalid_argument("Field MaxCurrent is not present");
        if (!robotConfig.contains("MaxTorque"))
            throw std::invalid_argument("Field MaxTorque is not present");
        if (!robotConfig.contains("RadToCountRatio"))
            throw std::invalid_argument("Field RadToCountRatio is not present");

        maxCurrent_ = robotConfig["MaxCurrent"].get<uint32_t>();
        maxTorque_ = robotConfig["MaxTorque"].get<uint32_t>();
        radToCountRatio_ = robotConfig["RadToCountRatio"].get<double>();

        // Optional
        if (robotConfig.contains("GearBoxRatio")) {
            gearBoxRatio_ = robotConfig["GearBoxRatio"].get<uint32_t>();
        }
    } catch (std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        throw std::invalid_argument(std::string(
            "The configuration file provided could not be parsed in EtherCATRobotConfiguration: ")
            + e.what());
    }
}

}  // namespace crf::actuators::robot
