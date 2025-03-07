/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "MissionUtility/DeployableRobot/DeployableRobot.hpp"

namespace crf::utility::missionutility {

DeployableRobot::DeployableRobot(
    std::shared_ptr<crf::control::motioncontroller::IMotionController> armController,
    const nlohmann::json& configFile):
    armController_(armController),
    retract2deploy_(),
    deploy2retract_(),
    retractedPos_(configFile.at("RetractedPosition").get<crf::utility::types::JointPositions>()),
    deployedPos_(configFile.at("DeployedPosition").get<crf::utility::types::JointPositions>()),
    logger_("DeployableRobot"),
    movingArm_(false) {
    logger_->debug("CTor");
    bigMargin_ = configFile["ArmPositionMargin"].get<float>();
    for (unsigned int i = 0 ; i < configFile.at("RetractToDeploy").size(); i++) {
        nlohmann::json point = configFile.at("RetractToDeploy")[i];
        retract2deploy_.push_back(point.at("Position").get<crf::utility::types::JointPositions>());
    }
    for (unsigned int i = configFile.at("RetractToDeploy").size(); i > 0; i--) {
        nlohmann::json point = configFile.at("RetractToDeploy")[i-1];
        deploy2retract_.push_back(point.at("Position").get<crf::utility::types::JointPositions>());
    }
}

DeployableRobot::~DeployableRobot() {
    logger_->debug("DTor");
    deinitialize();
}

bool DeployableRobot::initialize() {
    return armController_->initialize();
}

bool DeployableRobot::deinitialize() {
    return armController_->deinitialize();
}

bool DeployableRobot::deploy() {
    logger_->info("deploy");
    if (isDeployed()) {
        return true;
    }
    if (!isRetracted()) {
        logger_->warn("Arm is not in a safe position to be deployed");
        return false;
    }
    if (!armController_->appendPath(retract2deploy_)) {
        logger_->warn("Couldn't add and finish trajectory");
        return false;
    }
    while (armController_->isTrajectoryRunning().value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    movingArm_ = false;
    return true;
}

bool DeployableRobot::retract() {
    logger_->info("retract");
    if (isRetracted()) {
        return true;
    }
    if (!armController_->appendPath(deploy2retract_)) {
        logger_->warn("Couldn't add and finish trajectory");
        return false;
    }
    while (armController_->isTrajectoryRunning().value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    movingArm_ = false;
    return true;
}

bool DeployableRobot::stop() {
    armController_->softStop();
    return true;
}

bool DeployableRobot::isRetracted() {
    logger_->debug("isRetracted");
    crf::utility::types::JointPositions currentPosition =
        armController_->getSignals().joints.positions.value();
    if (!equalMinusLastJoint(currentPosition, retractedPos_, bigMargin_)) {
        logger_->debug("not retracted {} vs {}", currentPosition, retractedPos_);
        return false;
    }
    return true;
}

bool DeployableRobot::isDeployed() {
    logger_->debug("isDeployed");
    crf::utility::types::JointPositions currentPosition =
        armController_->getSignals().joints.positions.value();
    if (!equalMinusLastJoint(currentPosition, deployedPos_, bigMargin_)) {
        logger_->debug("not deployed {} vs {}", currentPosition, deployedPos_);
        return false;
    }
    return true;
}

bool DeployableRobot::isMoving() {
    return armController_->isTrajectoryRunning();
}

// Private

bool DeployableRobot::equalMinusLastJoint(const crf::utility::types::JointPositions& pos1,
    const crf::utility::types::JointPositions& pos2, float delta) {
    if (pos1.size() <= 2 || pos2.size() <= 2 || pos1.size() != pos2.size()) {
        return false;
    }
    logger_->debug("equalMinusLastJoint {} vs {}", pos1, pos2);
    for (uint8_t i = 0; i < pos1.size() - 2; i++) {
        if (fabs(pos1[i]-pos2[i]) > delta) return false;
    }
    return true;
}

}  // namespace crf::utility::missionutility
