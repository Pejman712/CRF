/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>

#include "crf/expected.hpp"
#include "Types/Types.hpp"
#include "Robot/IRobot.hpp"
#include "Robot/CiA402Robot/CiA402Robot.hpp"
#include "Robot/CiA402Robot/CiA402RobotConfiguration.hpp"

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

namespace crf::actuators::robot {

CiA402Robot::CiA402Robot(std::vector<std::shared_ptr<ICiA402Driver>> motors,
    const CiA402RobotConfiguration& configuration):
    motors_(motors),
    robotConfiguration_(configuration),
    initialized_(false),
    logger_("CiA402Robot") {
    logger_->debug("CTor");
    nMotors_ = robotConfiguration_.getNumberOfMotors();
    profileParams_ = robotConfiguration_.getProfileParameters();
}

CiA402Robot::~CiA402Robot() {
    logger_->debug("DTor");
    if (initialized_) deinitialize();
}

bool CiA402Robot::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->error("The CiA402Robot has already been initialized");
        return false;
    }
    for (unsigned int i = 0; i < nMotors_; i++) {
        if (motors_[i]->initialize()) continue;
        logger_->error("Motor {} couldn't be initialized!", i);
        return false;
    }
    initialized_ = true;
    return true;
}

bool CiA402Robot::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->error("The CiA402Robot hasn't been initialized");
        return false;
    }
    for (unsigned int i = 0; i < nMotors_; i++) {
        if (motors_[i]->deinitialize()) continue;
        logger_->error("Robot {} couldn't be deinitialized!", i);
        return false;
    }
    initialized_ = false;
    return true;
}

crf::expected<JointPositions> CiA402Robot::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!initialized_) return crf::Code::NotInitialized;

    JointPositions q(nMotors_);
    for (unsigned int i = 0; i < nMotors_; i++) {
        crf::expected<double> posExpected = motors_[i]->getPosition();
        if (!posExpected) return posExpected;
        q[i] = posExpected.value();
    }
    return q;
}

crf::expected<JointVelocities> CiA402Robot::getJointVelocities() {
    logger_->debug("getJointVelocities");
    if (!initialized_) return crf::Code::NotInitialized;

    JointVelocities qd(nMotors_);
    for (unsigned int i = 0; i < nMotors_; i++) {
        crf::expected<double> velExpected = motors_[i]->getVelocity();
        if (!velExpected) return velExpected;
        qd[i] = velExpected.value();
    }
    return qd;
}

crf::expected<JointAccelerations> CiA402Robot::getJointAccelerations() {
    logger_->debug("getJointAccelerations");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

crf::expected<JointForceTorques> CiA402Robot::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    if (!initialized_) return crf::Code::NotInitialized;

    JointForceTorques torque(nMotors_);
    for (unsigned int i = 0; i < nMotors_; i++) {
        crf::expected<double> tqeExpected = motors_[i]->getTorque();
        if (!tqeExpected) return tqeExpected;
        torque[i] = tqeExpected.value();
    }
    return torque;
}

crf::expected<TaskPose> CiA402Robot::getTaskPose() {
    logger_->debug("getTaskPose");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskVelocity> CiA402Robot::getTaskVelocity() {
    logger_->debug("getTaskVelocity");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskAcceleration> CiA402Robot::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskForceTorque> CiA402Robot::getTaskForceTorque() {
    logger_->debug("getTaskForceTorque");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CiA402Robot::setJointPositions(
    const bool& isSmoothTrajectory,
    const JointPositions& jointPositions,
    const JointVelocities& jointVelocities,
    const JointAccelerations& jointAccelerations) {
    logger_->debug("setJointPositions");
    if (!initialized_) return crf::Code::NotInitialized;

    if (jointPositions.size() != nMotors_) {
        logger_->error("Input position size not valid");
        return crf::Code::BadRequest;
    }
    if (jointVelocities.size() != nMotors_ && jointVelocities.size() != 0) {
        logger_->info("jointVelocities size is not matching with nDoF({}).", nMotors_);
        return crf::Code::BadRequest;
    }
    if (jointAccelerations.size() != nMotors_ && jointAccelerations.size() != 0) {
        logger_->info("jointAccelerations size is not matching nDoF({}). ", nMotors_);
        return crf::Code::BadRequest;
    }

    for (unsigned int i = 0; i < nMotors_; i++) {
        crf::expected<bool> res;
        switch (robotConfiguration_.getPositionMode()) {
            case PositionMode::ProfilePositionMode:
                res =  motors_[i]->setProfilePosition(
                    jointPositions[i],
                    profileParams_.jointVelocities[i],
                    profileParams_.jointAccelerations[i],
                    profileParams_.jointAccelerations[i],
                    crf::devices::canopendrivers::PositionReference::Absolute);
                break;
            case PositionMode::CyclicSyncPositionMode:
                res =  motors_[i]->setCyclicPosition(jointPositions[i]);
                break;
            case PositionMode::InterpolatedPositionMode:
                res = motors_[i]->setInterpolatedPosition(
                    jointPositions[i],
                    profileParams_.jointVelocities[i],
                    profileParams_.jointAccelerations[i],
                    profileParams_.jointAccelerations[i]);
                break;
        }

        if (!res) {
            softStop();
            return res;
        }
    }
    return true;
}

crf::expected<bool> CiA402Robot::setJointVelocities(
    const bool& isSmoothTrajectory,
    const JointVelocities& jointVelocities,
    const JointAccelerations& jointAccelerations) {
    logger_->debug("setJointVelocities");
    if (!initialized_) return crf::Code::NotInitialized;

    if (jointVelocities.size() != nMotors_) {
        logger_->info("jointVelocities size is not matching with nDoF({}).", nMotors_);
        return crf::Code::BadRequest;
    }
    if (jointAccelerations.size() != nMotors_ && jointAccelerations.size() != 0) {
        logger_->info("jointAccelerations size is not matching nDoF({}). ", nMotors_);
        return crf::Code::BadRequest;
    }

    for (unsigned int i = 0; i < nMotors_; i++) {
        crf::expected<bool> res;
        switch (robotConfiguration_.getVelocityMode()) {
            case VelocityMode::ProfileVelocityMode:
                res =  motors_[i]->setProfileVelocity(
                    jointVelocities[i],
                    profileParams_.jointAccelerations[i],
                    profileParams_.jointAccelerations[i]);
                break;
            case VelocityMode::CyclicSyncVelocityMode:
                res =  motors_[i]->setCyclicVelocity(jointVelocities[i]);
                break;
            case VelocityMode::VelocityMode:
                res = motors_[i]->setVelocity(
                    jointVelocities[i],
                    profileParams_.jointAccelerations[i],
                    1,
                    profileParams_.jointAccelerations[i],
                    1);
                break;
        }

        if (!res) {
            softStop();
            return res;
        }
    }
    return true;
}

crf::expected<bool> CiA402Robot::setJointForceTorques(
    const bool& isSmoothTrajectory,
    const JointForceTorques& jointForceTorques) {
    logger_->debug("setJointForceTorques");
    if (!initialized_) return crf::Code::NotInitialized;

    if (jointForceTorques.size() != nMotors_) {
        logger_->info("jointForceTorques size is not matching with nDoF({}).", nMotors_);
        return crf::Code::BadRequest;
    }

    for (unsigned int i = 0; i < nMotors_; i++) {
        crf::expected<bool> res;
        switch (robotConfiguration_.getTorqueMode()) {
            case TorqueMode::ProfileTorqueMode:
                res =  motors_[i]->setProfileTorque(jointForceTorques[i]);
                break;
            case TorqueMode::CyclicSyncTorqueMode:
                res =  motors_[i]->setCyclicTorque(jointForceTorques[i]);
                break;
        }

        if (!res) {
            softStop();
            return res;
        }
    }
    return true;
}

crf::expected<bool> CiA402Robot::setTaskPose(const bool& isSmoothTrajectory,
    const TaskPose& taskPose, const TaskVelocity& taskVelocity,
    const TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskPose");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CiA402Robot::setTaskVelocity(const bool& isSmoothTrajectory,
    const TaskVelocity& taskVelocity, const TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CiA402Robot::setTaskForceTorque(const bool& isSmoothTrajectory,
    const TaskForceTorque& taskForceTorque) {
    logger_->debug("setTaskForceTorque");
    return crf::Code::MethodNotAllowed;
}

crf::expected<JointVelocities> CiA402Robot::getProfileJointVelocities() {
    logger_->debug("getProfileJointVelocities");
    if (!initialized_) return crf::Code::NotInitialized;
    return profileParams_.jointVelocities;
}

crf::expected<JointAccelerations> CiA402Robot::getProfileJointAccelerations() {
    logger_->debug("getProfileJointAccelerations");
    if (!initialized_) return crf::Code::NotInitialized;
    return profileParams_.jointAccelerations;
}

crf::expected<TaskVelocity> CiA402Robot::getProfileTaskVelocity() {
    logger_->debug("getProfileTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskAcceleration> CiA402Robot::getProfileTaskAcceleration() {
    logger_->debug("getProfileTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CiA402Robot::setProfileJointVelocities(
    const JointVelocities& jointVelocities) {
    logger_->debug("setProfileJointVelocities");
    if (!initialized_) return crf::Code::NotInitialized;
    profileParams_.jointVelocities = jointVelocities;
    return true;
}

crf::expected<bool> CiA402Robot::setProfileJointAccelerations(
    const JointAccelerations& jointAccelerations) {
    logger_->debug("setProfileJointAccelerations");
    if (!initialized_) return crf::Code::NotInitialized;
    profileParams_.jointAccelerations = jointAccelerations;
    return true;
}

crf::expected<bool> CiA402Robot::setProfileTaskVelocity(
    const TaskVelocity& taskVelocity) {
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CiA402Robot::setProfileTaskAcceleration(
    const TaskAcceleration& taskAcceleration) {
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CiA402Robot::setGravity(const std::array<double, 3>& gravity) {
    logger_->debug("setGravity");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CiA402Robot::softStop() {
    logger_->debug("softStop");
    if (!initialized_) return crf::Code::NotInitialized;

    for (unsigned int i = 0; i < nMotors_; i++) {
        motors_[i]->stop();
    }
    return true;
}

crf::expected<bool> CiA402Robot::hardStop() {
    logger_->debug("hardStop");
    if (!initialized_) return crf::Code::NotInitialized;

    bool result = true;
    for (unsigned int i = 0; i < nMotors_; i++) {
        bool res = motors_[i]->quickStop();
        if (!res) result = res;
    }
    return result;
}

crf::expected<bool> CiA402Robot::setBrakes(std::vector<bool> brakesStatus) {
    logger_->debug("setBrakes");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

crf::expected<std::vector<bool>> CiA402Robot::getBrakes() {
    logger_->debug("getBrakes");
    if (!initialized_) return crf::Code::NotInitialized;
    return crf::Code::MethodNotAllowed;
}

std::set<Code> CiA402Robot::robotStatus() {
    logger_->debug("robotStatus");

    // This might need to change to return a ResponseCode object since otherwise we lose details
    std::set<Code> result;
    for (unsigned int i = 0; i < nMotors_; i++) {
        std::vector<crf::ResponseCode> status = motors_[i]->getMotorStatus();
        for (uint64_t j = 0; j < status.size(); j++) {
            result.insert(status[j].code());
        }
    }
    return result;
}

crf::expected<bool> CiA402Robot::resetFaultState() {
    logger_->debug("resetFaultState");
    if (!initialized_) return crf::Code::NotInitialized;
    bool result = true;
    for (unsigned int i = 0; i < nMotors_; i++) {
        bool response = motors_[i]->resetFault();
        if (!response) {
            logger_->error("Motor {} could not be reset", i);
            result = response;
        }
    }

    // We also reset the quick stop
    for (unsigned int i = 0; i < nMotors_; i++) {
        bool response = motors_[i]->resetQuickStop();
        if (!response) {
            logger_->error("Motor {} could not be reset", i);
            result = response;
        }
    }
    return result;
}

std::shared_ptr<RobotConfiguration> CiA402Robot::getConfiguration() {
    logger_->debug("getConfiguration");
    return std::make_shared<RobotConfiguration>(robotConfiguration_);
}

}  // namespace crf::actuators::robot
