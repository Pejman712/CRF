/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "SchunkPowerCube/SchunkPowerCubeConfiguration.hpp"

namespace crf {
namespace robots {
namespace schunkpowercube {

SchunkPowerCubeConfiguration::SchunkPowerCubeConfiguration() :
    RobotArmConfiguration(),
    jointsCanID_() {
}

bool SchunkPowerCubeConfiguration::parse(const nlohmann::json& robotJSON) {
    logger_->debug("parse");
    cleanup();
    if (!RobotArmConfiguration::parse(robotJSON)) {
        logger_->error("AQUI");
        return false;
    }
    try {
        for (int i=0; i < getNumberOfJoints(); i++) {
            jointsCanID_.push_back(robotJSON.at("Joints")[i].at("CanID").get<int>());
        }
    } catch (std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    return true;
}

std::vector<int> SchunkPowerCubeConfiguration::getJointsCanID() {
    return jointsCanID_;
}

void SchunkPowerCubeConfiguration::cleanup() {
    RobotArmConfiguration::cleanup();
    jointsCanID_.clear();
}

}  // namespace schunkpowercube
}  // namespace robots
}  // namespace crf
