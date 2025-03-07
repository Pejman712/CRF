/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: David Forkel BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#define NUMBER_OF_ATTEMPTS 10

#include <exception>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>
#include <thread>
#include <nlohmann/json.hpp>

#include "Robot/EtherCATRobot/EtherCATRobot.hpp"

namespace crf::actuators::robot {

EtherCATRobot::EtherCATRobot(
    std::shared_ptr<crf::devices::ethercatdevices::EtherCATManager> manager,
    std::vector<std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor>> motors,
    std::shared_ptr<EtherCATRobotConfiguration> config) :
        manager_(manager),
        motors_(motors),
        config_(config),
        dimensions_(motors_.size()),
        initialized_(false),
        logger_("EtherCATRobot") {
        logger_->info("CTor");
}

EtherCATRobot::~EtherCATRobot() {
    logger_->info("DTor");
    deinitialize();
}

bool EtherCATRobot::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    if (!manager_->initialize()) {
        logger_->error("Manager could not be initialized");
        return false;
    }

    for (int i = 0; i < dimensions_; i++) {
        sleep(1);
        if (motors_[i]->initialize()) {
            logger_->info("Motor of wheel {} initialized", i + 1);
        } else {
            logger_->error("Cannot initialize Motor of wheel {}", i + 1);
            return false;
        }
    }
    sleep(1);

    if (!manager_->configureIOMap()) {
        logger_->error("Manager could not be configure IO map");
        return false;
    }

    sleep(1);

    for (int i = 0; i < dimensions_; i++) {
        if (!motors_[i]->bindPDOs()) {
            logger_->error("Cannot bind PDO for Motor of wheel {}", i + 1);
            return false;
        }
    }

    sleep(1);

    if (!manager_->enterOp()) {
        logger_->error("Manager could not enter operational!");
        return false;
    }

    sleep(1);

    uint32_t maximumAccelerationWheel = static_cast<uint32_t>(
        config_->getJointLimits().maxAcceleration[0]*config_->getRadToCountRatio());

    for (size_t i = 0; i < dimensions_; i++) {
        if (!motors_[i]->setProfileAcceleration(maximumAccelerationWheel)) {
            logger_->error("Could not set ProfileAcceleration parameter"\
            "for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
        if (!motors_[i]->setProfileDeceleration(maximumAccelerationWheel)) {
            logger_->error("Could not set ProfileDeceleration parameter"\
            "for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
        if (!motors_[i]->setQuickstopDeceleration(maximumAccelerationWheel)) {
            logger_->error("Could not set QuickstopDeceleration parameter"\
            "for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
    }

    for (size_t i = 0; i < dimensions_; i++) {
        sleep(1);
        if (!motors_[i]->setMaxCurrent(config_->getMaxCurrent())) {
            logger_->error("Could not set MaxCurrent parameter for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
        if (!motors_[i]->setMaxTorque(config_->getMaxTorque())) {
            logger_->error("Could not set MaxTorque parameter for Motor of wheel {}", i + 1);
            return false;
        }
        sleep(1);
        if (!motors_[i]->setModeOfOperation(
            crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode)) {
            logger_->error("Could not set MaxCurrent parameter for Motor of wheel {}", i + 1);
            return false;
        }
    }

    for (size_t i = 0; i < dimensions_; i++) {
        for (int k = 0; k < NUMBER_OF_ATTEMPTS; k++) {
            logger_->info("Checking motor {}", i);
            auto isInFault = motors_[i]->inFault();
            if (!isInFault) {
                logger_->error("Could not get if Motor of wheel {} is in fault", i + 1);
                return false;
            }

            if ((isInFault.value()) && !motors_[i]->faultReset()) {
                logger_->error("Could not reset the fault of Motor of wheel {}", i + 1);
                continue;
            }

            if (!motors_[i]->shutdown()) {
                logger_->error("Could not shutdown Motor of wheel {}", i + 1);
                continue;
            }

            if (motors_[i]->enableOperation()) {
                break;
            }
            logger_->error("Could not enable operation of Motor of wheel {}", i + 1);
        }
    }

    for (size_t i = 0; i < dimensions_; i++) {
        logger_->info("Checking enabled {}", i);
        auto enabled = motors_[i]->isEnabled();
        if (!enabled) {
            logger_->error("Could not check if Motor of wheel {} is enabled", i + 1);
            return false;
        }

        if (!enabled.value()) {
            logger_->error("Motor of wheel {} did not enable", i + 1);
            return false;
        }
    }

    initialized_ = true;
    return true;
}

bool EtherCATRobot::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return false;
    }
    for (uint64_t i = 0; i < dimensions_; i++) {
        motors_[i]->deinitialize();
    }
    return true;
}

crf::expected<crf::utility::types::JointPositions> EtherCATRobot::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    crf::utility::types::JointPositions pos(dimensions_);
    double conversion = config_->getRadToCountRatio() * config_->getGearBoxReduction();
    for (uint64_t i = 0; i < dimensions_; i++) {
        std::optional<int32_t> res = motors_[i]->getPosition();
        if (!res) return crf::Code::RequestToDeviceFailed;
        pos[i] = res.value()/conversion;
    }
    return pos;
}

crf::expected<crf::utility::types::JointVelocities> EtherCATRobot::getJointVelocities() {
    logger_->debug("getJointVelocities");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    crf::utility::types::JointVelocities vel(dimensions_);
    double conversion = config_->getRadToCountRatio() * config_->getGearBoxReduction();
    for (uint64_t i = 0; i < dimensions_; i++) {
        std::optional<int32_t> res = motors_[i]->getVelocity();
        if (!res) return crf::Code::RequestToDeviceFailed;
        vel[i] = res.value()/conversion;
    }
    return vel;
}

crf::expected<crf::utility::types::JointAccelerations> EtherCATRobot::getJointAccelerations() {
    logger_->debug("getJointAccelerations not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointForceTorques> EtherCATRobot::getJointForceTorques() {
    logger_->debug("getJointForceTorques not supported");
    return crf::Code::MethodNotAllowed;
}
crf::expected<crf::utility::types::TaskPose> EtherCATRobot::getTaskPose() {
    logger_->debug("getJointForceTorques not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskVelocity> EtherCATRobot::getTaskVelocity() {
    logger_->warn("getTaskVelocity not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskAcceleration> EtherCATRobot::getTaskAcceleration() {
    logger_->warn("getTaskAcceleration not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskForceTorque> EtherCATRobot::getTaskForceTorque() {
    logger_->debug("getJointForceTorques not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> EtherCATRobot::setJointPositions(const bool& isSmoothTrajectory,
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointPositions");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    double conversion = config_->getRadToCountRatio() * config_->getGearBoxReduction();
    for (uint64_t i = 0; i < dimensions_; i++) {
        bool res =  motors_[i]->setPosition(
            static_cast<int32_t>(jointPositions[i] * conversion),
            static_cast<int32_t>(jointVelocities[i] * conversion),
            static_cast<int32_t>(jointAccelerations[i] * conversion),
            static_cast<int32_t>(jointAccelerations[i] * conversion),
            false);
        if (!res) return crf::Code::RequestToDeviceFailed;
    }
    return true;
}

crf::expected<bool> EtherCATRobot::setJointVelocities(const bool& isSmoothTrajectory,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointVelocities");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    // Check units of velocity
    double conversion = config_->getRadToCountRatio() * config_->getGearBoxReduction();
    for (uint64_t i = 0; i < dimensions_; i++) {
        logger_->info("size {}", i);
        bool res =  motors_[i]->setVelocity(static_cast<int32_t>(jointVelocities[i] * conversion));
        if (!res) return crf::Code::RequestToDeviceFailed;
    }
    return true;
}

crf::expected<bool> EtherCATRobot::setJointForceTorques(const bool& isSmoothTrajectory,
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->debug("setJointForceTorques");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    // Check units of torque
    double conversion = config_->getGearBoxReduction();
    for (uint64_t i = 0; i < dimensions_; i++) {
        bool res =  motors_[i]->setTorque(jointForceTorques[i] / conversion);
        if (!res) return crf::Code::RequestToDeviceFailed;
    }
    return true;
}

crf::expected<bool> EtherCATRobot::setTaskPose(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskPose& taskPose,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->warn("setTaskPose not supported due to an API bug");
    return crf::Code::NotImplemented;
}

crf::expected<bool> EtherCATRobot::setTaskVelocity(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->warn("setTaskVelocity not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> EtherCATRobot::setTaskForceTorque(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskForceTorque& taskForceTorque) {
    logger_->warn("setTaskForceTorque not supported");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointVelocities> EtherCATRobot::getProfileJointVelocities() {
    logger_->warn("getProfileJointVelocities not supported");
    return crf::Code::NotImplemented;
}

crf::expected<crf::utility::types::JointAccelerations>
    EtherCATRobot::getProfileJointAccelerations() {
    logger_->warn("getProfileJointAccelerations not supported");
    return crf::Code::NotImplemented;
}

crf::expected<crf::utility::types::TaskVelocity> EtherCATRobot::getProfileTaskVelocity() {
    logger_->warn("getProfileTaskVelocity not supported");
    return crf::Code::NotImplemented;
}

crf::expected<crf::utility::types::TaskAcceleration> EtherCATRobot::getProfileTaskAcceleration() {
    logger_->warn("getProfileTaskAcceleration not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> EtherCATRobot::setProfileJointVelocities(
    const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->warn("setProfileJointVelocities not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> EtherCATRobot::setProfileJointAccelerations(
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->warn("setProfileJointAccelerations not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> EtherCATRobot::setProfileTaskVelocity(
    const crf::utility::types::TaskVelocity& taskVelocity) {
    logger_->warn("setProfileTaskVelocity not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> EtherCATRobot::setProfileTaskAcceleration(
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->warn("setProfileTaskAcceleration not supported");
    return crf::Code::NotImplemented;
}


crf::expected<bool> EtherCATRobot::setGravity(const std::array<double, 3>& gravity) {
    logger_->warn("setGravity not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> EtherCATRobot::softStop() {
    logger_->debug("softStop");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    for (uint64_t i = 0; i < dimensions_; i++) {
        motors_[i]->stop();
    }
    return true;
}

crf::expected<bool> EtherCATRobot::hardStop() {
    if (!initialized_) {
        logger_->error("dEtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    for (uint64_t i = 0; i < dimensions_; i++) {
        motors_[i]->quickStop();
    }
    return true;
}

crf::expected<bool> EtherCATRobot::setBrakes(std::vector<bool> brakesStatus) {
    logger_->warn("EtherCATRobot has no brakes");
    return crf::Code::MethodNotAllowed;
}

crf::expected<std::vector<bool>> EtherCATRobot::getBrakes() {
    logger_->warn("EtherCATRobot has no brakes");
    return crf::Code::MethodNotAllowed;
}

std::set<Code> EtherCATRobot::robotStatus() {
    logger_->debug("robotStatus");
    std::set<Code> statusList;
    if (!initialized_) {
        statusList.insert(crf::Code::NotInitialized);
    }
    return statusList;
}

crf::expected<bool> EtherCATRobot::resetFaultState() {
    logger_->debug("softStop");
    if (!initialized_) {
        logger_->error("EtherCATRobot has not been initialized");
        return crf::Code::NotInitialized;
    }
    for (uint64_t i = 0; i < dimensions_; i++) {
        motors_[i]->faultReset();
    }
    return true;
}

std::shared_ptr<RobotConfiguration> EtherCATRobot::getConfiguration() {
    return config_;
}

}  // namespace crf::actuators::robot



