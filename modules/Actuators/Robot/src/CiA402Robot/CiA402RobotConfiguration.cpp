/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "Robot/CiA402Robot/CiA402RobotConfiguration.hpp"

namespace crf::actuators::robot {

CiA402RobotConfiguration::CiA402RobotConfiguration(const nlohmann::json& robotConfig) :
    RobotConfiguration(robotConfig),
    logger_("CiA402RobotConfiguration") {
        logger_->debug("CTor");
        parse(robotConfig);
}

PositionMode CiA402RobotConfiguration::getPositionMode() const {
    return posMode_;
}

VelocityMode CiA402RobotConfiguration::getVelocityMode() const {
    return velMode_;
}

TorqueMode CiA402RobotConfiguration::getTorqueMode() const {
    return tqeMode_;
}

std::vector<nlohmann::json> CiA402RobotConfiguration::getMotorConfigFiles() const {
    return motorJSONs_;
}

uint64_t CiA402RobotConfiguration::getNumberOfMotors() const {
    return numberOfMotors_;
}

// Private

void CiA402RobotConfiguration::parse(const nlohmann::json& robotConfig) {
    logger_->debug("parse");
    try {
        numberOfMotors_ = robotConfig.at("NumberOfMotors").get<uint64_t>();
        posMode_ = positionMap_.at(robotConfig.at("PositionMode").get<std::string>());
        velMode_ = velocityMap_.at(robotConfig.at("VelocityMode").get<std::string>());
        tqeMode_ = torqueMap_.at(robotConfig.at("TorqueMode").get<std::string>());

        for (uint64_t i = 0; i < robotConfig.at("Motors").size(); i++) {
            if (robotConfig.at("Motors").at(i).at("SlaveID").is_number()) {
                motorJSONs_.push_back(robotConfig["Motors"].at(i));
            } else {
                for (uint64_t j = 0; j < robotConfig.at("Motors").at(i).at("SlaveID").size(); j++) {
                    nlohmann::json slave = robotConfig.at("Motors").at(i);
                    slave["SlaveID"] = robotConfig.at("Motors").at(i).at("SlaveID").at(j);
                    motorJSONs_.push_back(slave);
                }
            }
        }

        if (motorJSONs_.size() != numberOfMotors_) {
            throw std::invalid_argument(
                "Number of motors and number of slave config files does not match");
        }
    } catch (const std::exception& e) {
        throw std::invalid_argument(std::string(
            "The configuration file provided could not be parsed in CiA402RobotConfiguration: ")
            + e.what());
    }
}

}  // namespace crf::actuators::robot
