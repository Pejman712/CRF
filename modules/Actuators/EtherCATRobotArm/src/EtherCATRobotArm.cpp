/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "EtherCATRobotArm/EtherCATRobotArm.hpp"
#include "EtherCATDevices/IEtherCATMotor.hpp"

namespace crf::actuators::ethercatrobotarm {

EtherCATRobotArm::EtherCATRobotArm(const nlohmann::json& armConfigFile,
    std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint1,
    std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint2,
    std::shared_ptr<devices::ethercatdevices::IEtherCATMotor> joint3) :
    logger_("EtherCATRobotArm"),
    armConfigFile_(armConfigFile),
    joints_(),
    initialized_(false),
    configuration_(new robotarm::RobotArmConfiguration()),
    jointPositionsExtraOffset_({0.0, 0.0, 0.0}) {
    logger_->debug("CTor");
    joints_.push_back(joint1);
    joints_.push_back(joint2);
    joints_.push_back(joint3);
}

EtherCATRobotArm::~EtherCATRobotArm() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool EtherCATRobotArm::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!configuration_->parse(armConfigFile_)) {
        logger_->warn("Could not parse configuration file");
        return false;
    }
    for (size_t i=0; i < joints_.size(); i++) {
        joints_[i]->setMaxTorque(jointMaxCurrents_[i]);
        joints_[i]->setModeOfOperation(
            crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode);
    }
    for (size_t i=0; i < joints_.size(); i++) {
        for (int k=0; k < numberOfInitializationAttempts; k++) {
            std::optional<bool> isInFault = joints_[i]->inFault();
            if (!isInFault) {
                logger_->error("Could not get if motor {} is in fault", i);
                return false;
            }
            if ((isInFault.value()) && !joints_[i]->faultReset()) {
                logger_->warn("Could not reset the fault of motor {}", i);
                continue;
            }
            if (!joints_[i]->shutdown()) {
                logger_->warn("Could not shutdown motor {}", i);
                continue;
            }
            if (!joints_[i]->enableOperation()) {
                logger_->warn("Could not enable operation of motor {}", i);
                continue;
            } else {
                // The motor is properly enabled, jumping to the next motor if there is
                break;
            }
        }
    }
    for (size_t i=0; i < joints_.size(); i++) {
        std::optional<bool> enabled = joints_[i]->isEnabled();
        if (!enabled) {
            logger_->error("Could not check if joint {} is enabled", i);
            return false;
        }
        if (!enabled.value()) {
            logger_->error("Joint {} is not enabled", i);
            return false;
        }
    }
    jointsDirection_ = configuration_->getJointsDirection();
    jointPositionsOffset_ = configuration_->getJointsOffset();
    for (size_t i=0; i < joints_.size(); i++) {
        std::optional<int32_t> position = joints_[i]->getPosition();
        if (!position) {
            logger_->error("Failed to get position from joint {}", i);
            return false;
        }
        float convertedPosition = static_cast<float>(position.value()/jointConverFactors_[i]);
        if ((convertedPosition < -M_PI) || (convertedPosition > M_PI)) {
            logger_->warn("Joint {0} is not in the range [-PI, PI]. Measured position is {1}", i,
                convertedPosition);
            logger_->warn("Adding an extra position offset of -2*PI to the joint {}", i);
            jointPositionsExtraOffset_[i] += -2*M_PI;
        }
    }
    initialized_ = true;
    return true;
}

bool EtherCATRobotArm::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    for (size_t i=0; i < joints_.size(); i++) {
        if (!joints_[i]->disableOperation()) {
            return false;
        }
    }
    initialized_ = false;
    return true;
}

boost::optional<crf::utility::types::JointPositions> EtherCATRobotArm::getJointPositions() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    crf::utility::types::JointPositions convertedPosition(joints_.size());
    for (size_t i=0; i < joints_.size(); i++) {
        std::optional<int32_t> position = joints_[i]->getPosition();
        if (!position) {
            logger_->error("Failed to get position from joint {}", i);
            return boost::none;
        }
        convertedPosition[i] = static_cast<float>(position.value());
        convertedPosition[i] /= jointConverFactors_[i];
        convertedPosition[i] /= jointsDirection_[i];
        if (i == 0) {
            convertedPosition[i] /= HDBelt_;
        }
        convertedPosition[i] += (jointPositionsOffset_[i] + jointPositionsExtraOffset_[i]);
    }
    return convertedPosition;
}

boost::optional<crf::utility::types::JointVelocities> EtherCATRobotArm::getJointVelocities() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    crf::utility::types::JointVelocities convertedVelocity(joints_.size());
    for (size_t i=0; i < joints_.size(); i++) {
        std::optional<int32_t> velocity = joints_[i]->getVelocity();
        if (!velocity) {
            logger_->error("Failed to get position from joint {}", i);
            return boost::none;
        }
        convertedVelocity[i] = static_cast<float>(velocity.value());
        convertedVelocity[i] /= jointConverFactors_[i];
        convertedVelocity[i] /= jointsDirection_[i];
        if (i == 0) {
            convertedVelocity[i] /= HDBelt_;
        }
    }
    return convertedVelocity;
}

boost::optional<crf::utility::types::JointForceTorques> EtherCATRobotArm::getJointForceTorques() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    // TODO(frriccar): The current needs to be transfomerd to the torque
    crf::utility::types::JointForceTorques conertedTorque(joints_.size());
    for (size_t i=0; i < joints_.size(); i++) {
        std::optional<int32_t> torque = joints_[i]->getCurrent();
        if (!torque) {
            logger_->error("Failed to get position from joint {}", i);
            return boost::none;
        }
        conertedTorque[i] = static_cast<float>(torque.value());
        conertedTorque[i] /= jointsDirection_[i];
    }
    return conertedTorque;
}

boost::optional<crf::utility::types::TaskPose> EtherCATRobotArm::getTaskPose() {
    logger_->error("Task position not implemented");
    return boost::none;
}

boost::optional<crf::utility::types::TaskVelocity> EtherCATRobotArm::getTaskVelocity() {
    logger_->error("Task velocity not implemented");
    return boost::none;
}

boost::optional<crf::utility::types::TaskForceTorque> EtherCATRobotArm::getTaskForceTorque() {
    logger_->error("Task torque not implemented");
    return boost::none;
}

bool EtherCATRobotArm::setJointPositions(
    const crf::utility::types::JointPositions& jointPositions) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (static_cast<size_t>(jointPositions.size()) != joints_.size()) {
        logger_->warn("Input size ({}) different than number of joints ({})",
            jointPositions.size(), joints_.size());
        return false;
    }
    for (size_t i = 0; i < joints_.size(); i++) {
        if ((jointPositions[i] > configuration_->getJointsConfiguration()[i].maximumPosition) ||
            (jointPositions[i] < configuration_->getJointsConfiguration()[i].minimumPosition)) {
            logger_->warn("Selected position is out of position limits");
            return false;
        }
    }
    for (size_t i = 0; i < joints_.size(); i++) {
        float convertedPosition = jointPositions[i];
        convertedPosition -= (jointPositionsOffset_[i] + jointPositionsExtraOffset_[i]);
        if (i == 0) {
            convertedPosition *= HDBelt_;
        }
        convertedPosition *= jointsDirection_[i];
        convertedPosition *= jointConverFactors_[i];
        if (!joints_[i]->setPosition(static_cast<int32_t>(convertedPosition), false)) {
            logger_->error("Failed to set position, stopping arm");
            stopArm();
            return false;
        }
    }
    return true;
}

bool EtherCATRobotArm::setJointVelocities(
    const crf::utility::types::JointVelocities& jointVelocities) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (static_cast<size_t>(jointVelocities.size()) != joints_.size()) {
        logger_->warn("Input size ({}) different than number of joints ({})",
            jointVelocities.size(), joints_.size());
        return false;
    }
    for (size_t i = 0; i < joints_.size(); i++) {
        if (fabs(jointVelocities[i]) >
            configuration_->getJointsConfiguration()[i].maximumVelocity) {
            logger_->warn("Selected velocity is higher than maximum velocity");
            return false;
        }
    }
    for (size_t i = 0; i < joints_.size(); i++) {
        float convertedVelocity = jointVelocities[i];
        if (i == 0) {
            convertedVelocity *= HDBelt_;
        }
        convertedVelocity *= jointsDirection_[i];
        convertedVelocity *= jointConverFactors_[i];
        if (!joints_[i]->setVelocity(static_cast<int32_t>(convertedVelocity))) {
            logger_->error("Failed to set velocity, stopping arm");
            stopArm();
            return false;
        }
    }
    return true;
}

bool EtherCATRobotArm::setJointForceTorques(
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (static_cast<size_t>(jointForceTorques.size()) != joints_.size()) {
        logger_->warn("Input size ({}) different than number of joints ({})",
            jointForceTorques.size(), joints_.size());
        return false;
    }
    for (size_t i = 0; i < joints_.size(); i++) {
        if (fabs(jointForceTorques[i]) >
            configuration_->getJointsConfiguration()[i].maximumTorque) {
            logger_->warn("Selected velocity is higher than maximum velocity");
            return false;
        }
    }
    // TODO(frriccar): The torque needs to be transfomerd to the current
    for (size_t i = 0; i < joints_.size(); i++) {
        float conertedTorque = jointForceTorques[i];
        conertedTorque *= jointsDirection_[i];
        if (!joints_[i]->setTorque(static_cast<int32_t>(conertedTorque))) {
            logger_->error("Failed to set torque, stopping arm");
            stopArm();
            return false;
        }
    }
    return true;
}

bool EtherCATRobotArm::setTaskPose(
    const crf::utility::types::TaskPose& position) {
    logger_->error("Task position not implemented");
    return false;
}

bool EtherCATRobotArm::setTaskVelocity(const crf::utility::types::TaskVelocity& velocity,
    bool TCP) {
    logger_->error("Task velocity not implemented");
    return false;
}

bool EtherCATRobotArm::stopArm() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    for (size_t i=0; i < joints_.size(); i++) {
        if (!joints_[i]->setVelocity(0)) {
            return false;
        }
    }
    return true;
}

bool EtherCATRobotArm::enableBrakes() {
    logger_->error("Brake control no implemented");
    return false;
}

bool EtherCATRobotArm::disableBrakes() {
    logger_->error("Brake control no implemented");
    return false;
}

std::shared_ptr<actuators::robotarm::RobotArmConfiguration> EtherCATRobotArm::getConfiguration() {
    return configuration_;
}

}  // namespace crf::actuators::ethercatrobotarm
