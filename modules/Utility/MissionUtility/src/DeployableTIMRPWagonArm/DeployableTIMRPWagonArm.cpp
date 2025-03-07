/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/

#include <memory>
#include <iostream>
#include <fstream>
#include <string>

#include "MissionUtility/DeployableTIMRPWagonArm/DeployableTIMRPWagonArm.hpp"

namespace crf::utility::missionutility {

DeployableTIMRPWagonArm::DeployableTIMRPWagonArm(
    std::shared_ptr<crf::actuators::timrpwagon::ITIMRPWagon> timRPWagon):
    timRPWagon_(timRPWagon),
    logger_("DeployableTIMRPWagonArm"),
    movingArm_(false),
    mtx_() {
    logger_->debug("CTor");
}

DeployableTIMRPWagonArm::~DeployableTIMRPWagonArm() {
    logger_->debug("DTor");
}

bool DeployableTIMRPWagonArm::initialize() {
    return timRPWagon_->initialize();
}

bool DeployableTIMRPWagonArm::deinitialize() {
    return timRPWagon_->deinitialize();
}

bool DeployableTIMRPWagonArm::deploy() {
    logger_->debug("deploy");
    if (isDeployed()) {
        return true;
    }
    std::scoped_lock<std::mutex> lck(mtx_);
    std::optional<bool> res = timRPWagon_->deployRPArm();
    if (!res) {
        logger_->warn("Failed to deploy the arm");
        return false;
    }
    return res.value();
}

bool DeployableTIMRPWagonArm::retract() {
    logger_->debug("retract");
    if (isRetracted()) {
        return true;
    }
    std::scoped_lock<std::mutex> lck(mtx_);
    std::optional<bool> res = timRPWagon_->retractRPArm();
    if (!res) {
        logger_->warn("Failed to retract the arm");
        return false;
    }
    return res.value();
}

bool DeployableTIMRPWagonArm::stop() {
    std::optional<bool> res = timRPWagon_->stopRPArm();
    if (!res) {
        logger_->warn("Failed to retract the arm");
        return false;
    }
    return res.value();
}

bool DeployableTIMRPWagonArm::isRetracted() {
    logger_->debug("isRetracted");
    crf::actuators::timrpwagon::RPArmPosition currentPosition = timRPWagon_->getRPArmPosition();
    if (currentPosition != crf::actuators::timrpwagon::RPArmPosition::Retracted) {
        logger_->debug("Not retracted, current status is {} vs 1", currentPosition);
        return false;
    }
    return true;
}

bool DeployableTIMRPWagonArm::isDeployed() {
    logger_->debug("isDeployed");
    crf::actuators::timrpwagon::RPArmPosition currentPosition = timRPWagon_->getRPArmPosition();
    if (currentPosition != crf::actuators::timrpwagon::RPArmPosition::Deployed) {
        logger_->debug("Not deployed, current status is {} vs 3", currentPosition);
        return false;
    }
    return true;
}

bool DeployableTIMRPWagonArm::isMoving() {
    logger_->debug("isMoving");
    crf::actuators::timrpwagon::RPArmPosition currentPosition = timRPWagon_->getRPArmPosition();
    if (currentPosition != crf::actuators::timrpwagon::RPArmPosition::InTheMiddle) {
        logger_->debug("Not retracted, current status is {} vs 2", currentPosition);
        return false;
    }
    return true;
}

}  // namespace crf::utility::missionutility
