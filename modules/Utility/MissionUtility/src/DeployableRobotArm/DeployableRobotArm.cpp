/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <memory>
#include <iostream>
#include <fstream>
#include <string>

#include "MissionUtility/DeployableRobotArm/DeployableRobotArm.hpp"

namespace crf::utility::missionutility {

DeployableRobotArm::DeployableRobotArm(
    std::shared_ptr<crf::control::robotarmcontroller::IRobotArmController> armController,
    const nlohmann::json& configFile):
    armController_(armController),
    trajectoryJson_(configFile),
    logger_("DeployableRobotArm"),
    movingArm_(false) {
    logger_->debug("CTor");
    bigMargin_ = trajectoryJson_["ArmPositionMargin"].get<float>();
}

DeployableRobotArm::~DeployableRobotArm() {
    logger_->debug("DTor");
}

bool DeployableRobotArm::initialize() {
    return armController_->initialize();
}

bool DeployableRobotArm::deinitialize() {
    return armController_->deinitialize();
}

bool DeployableRobotArm::deploy() {
    logger_->info("deploy");
    if (isDeployed()) {
        return true;
    }
    if (!isRetracted()) {
        logger_->warn("Arm is not in a safe position to be deployed");
        return false;
    }
    std::scoped_lock<std::mutex> lck(mtx_);
    for (unsigned int i = 0 ; i < trajectoryJson_.at("RetractToDeploy").size(); i++) {
        nlohmann::json point = trajectoryJson_.at("RetractToDeploy")[i];
        if (!moveArm(point.at("Position").get<crf::utility::types::JointPositions>(),
            point.at("Velocity").get<crf::utility::types::JointVelocities>())) {
            logger_->warn("Couldn't finish trajectory");
            return false;
        }
    }
    movingArm_ = false;
    return true;
}

bool DeployableRobotArm::retract() {
    logger_->info("retract");
    if (isRetracted()) {
        return true;
    }
    std::scoped_lock<std::mutex> lck(mtx_);
    for (unsigned int i = trajectoryJson_.at("RetractToDeploy").size(); i > 0; i--) {
        nlohmann::json point = trajectoryJson_.at("RetractToDeploy")[i-1];
        if (!moveArm(point.at("Position").get<crf::utility::types::JointPositions>(),
            point.at("Velocity").get<crf::utility::types::JointVelocities>())) {
            logger_->warn("Couldn't finish trajectory");
            return false;
        }
    }
    movingArm_ = false;
    return true;
}

bool DeployableRobotArm::stop() {
    return armController_->interruptTrajectory();
}

bool DeployableRobotArm::isRetracted() {
    logger_->debug("isRetracted");
    crf::utility::types::JointPositions currentPosition = armController_->getJointPositions();
    crf::utility::types::JointPositions retractedPos =
        trajectoryJson_.at("RetractedPosition").get<crf::utility::types::JointPositions>();
    if (!equalMinusLastJoint(currentPosition, retractedPos, bigMargin_)) {
        logger_->debug("not retracted {} vs {}", currentPosition, retractedPos);
        return false;
    }
    return true;
}

bool DeployableRobotArm::isDeployed() {
    logger_->debug("isDeployed");
    crf::utility::types::JointPositions currentPosition = armController_->getJointPositions();
    crf::utility::types::JointPositions deployedPos =
        trajectoryJson_.at("DeployedPosition").get<crf::utility::types::JointPositions>();
    if (!equalMinusLastJoint(currentPosition, deployedPos, bigMargin_)) {
        logger_->debug("not deployed {} vs {}", currentPosition, deployedPos);
        return false;
    }
    return true;
}

bool DeployableRobotArm::isMoving() {
    return movingArm_.load();
}

// Private Methods

bool DeployableRobotArm::moveArm(const crf::utility::types::JointPositions &position,
    const crf::utility::types::JointVelocities &velocity) {
    if (!armController_->setJointsMaximumVelocity(velocity)) {
        logger_->warn("Could not set maximum joints velocity from json");
        return false;
    }
    movingArm_ = true;
    std::future<bool> result = armController_->setPosition(position);
    if (!result.valid()) {
        logger_->warn("Trajectory not valid");
        return false;
    }
    return result.get();
}

bool DeployableRobotArm::equalMinusLastJoint(const crf::utility::types::JointPositions& pos1,
    const crf::utility::types::JointPositions& pos2, float delta) {
    if (pos1.size() <= 2 || pos2.size() <= 2 || pos1.size() != pos2.size()) {
        return false;
    }
    logger_->debug("equalMinusLastJoint {} vs {}", pos1, pos2);
    for (uint8_t i = 0; i < pos1.size() - 2; i++) {  // Not the last Joint
        if (fabs(pos1[i]-pos2[i]) > delta) return false;
    }
    return true;
}

}  // namespace crf::utility::missionutility
