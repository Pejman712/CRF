/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:       Thomas Breant CERN EN/SMM/MRO 2020
 * Contributor:  Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
*/

#include "PilzArm/PilzArm.hpp"

namespace crf::actuators::pilzarm {

PilzArm::PilzArm(std::shared_ptr<crf::communication::cansocket::ICANSocket> socket,
    const nlohmann::json& robotConfigFile,
    std::shared_ptr<crf::actuators::gripper::SchunkGripperCANOpen> gripper,
    std::shared_ptr<crf::devices::canopendevices::CANOpenContext> ctx):
    robotConfigFile_(robotConfigFile),
    configuration_(new robotarm::RobotArmConfiguration),
    logger_("PilzArm"),
    socket_(socket),
    ctx_(ctx),
    gripper_(gripper),
    initialized_(false) {
    logger_->debug("CTor");
    if (!configuration_->parse(robotConfigFile_)) {
        logger_->critical("Could not parse the configuration file");
    }
}

PilzArm::~PilzArm() {
    logger_->debug("DTor");
    if (initialized_) deinitialize();
}

bool PilzArm::initialize() {
    logger_->debug("initialize()");
    if (initialized_) {
        logger_->warn("PilzArm already initialized");
        return true;
    }

    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        // Create Joint and add it to arm
        auto joint = std::make_shared<crf::devices::canopendevices::ERB>(0x3 + i, socket_,
            crf::devices::canopendevices::ModesOfOperation::InterpolatedPositionMode);
        // 0x3 is the CAN ID stablished by PILZ for the First Joint
        arm_.push_back(joint);

        // Add to context and initialize
        if (!ctx_->addDevice(arm_.at(i))) {
            logger_->critical("Failed to add motor " + std::to_string(i+3) + " to the context");
            return false;
        }
        if (!arm_.at(i)->initialize()) {
            logger_->critical("Initialization of motor " + std::to_string(i+3) + " failed!");
            return false;
        }
        if (!arm_.at(i)->enableOperation()) {
            logger_->error("Could not enable the operation ont the motor {}", i);
            return false;
        }
    }
    if (gripper_ != nullptr) {
        if (!gripper_->initialize()) {
            logger_->critical("Initialization of the gripper failed");
            return false;
        }
    }
    initialized_ = true;
    // This procedure allows the pilz arm to start properly after a powercut
    // without the need of starting the program twice
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        int jointDirection = std::copysign(1.0, getJointPositions().value()[i]);
        arm_.at(i)->setVelocity(-1 * jointDirection * 0.0001);
        arm_.at(i)->setVelocity(0.0);
    }
    return true;
}

bool PilzArm::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("PilzArm already deinitialized");
        return false;
    }
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        if (!arm_.at(i)->deinitialize()) {
            logger_->critical("Deinitialization of motor" + std::to_string(i+3) + " failed!");
            return false;
        }
    }
    if (gripper_ != nullptr) {
        if (!gripper_->deinitialize()) {
            logger_->critical("Initialization of the gripper failed");
            return false;
        }
    }
    initialized_ = false;
    return true;
}

boost::optional<utility::types::JointPositions> PilzArm::getJointPositions() {
    if (!initialized_) {
        logger_->debug("Unable to perform getJointPositions(), arm was not initialized");
        return boost::none;
    }
    crf::utility::types::JointPositions position(configuration_->getNumberOfJoints());
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        position[i] = arm_.at(i)->getPosition().value()*millidegreeToRad_;
    }
    return position;
}

boost::optional<utility::types::JointVelocities> PilzArm::getJointVelocities() {
    if (!initialized_) {
        logger_->debug("Unable to perform getJointVelocities(), arm was not initialized");
        return boost::none;
    }
    crf::utility::types::JointVelocities velocity(configuration_->getNumberOfJoints());
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        velocity[i] = arm_.at(i)->getVelocity().value()*millidegreeToRad_;
    }
    return velocity;
}

boost::optional<crf::utility::types::JointForceTorques> PilzArm::getJointForceTorques() {
    if (!initialized_) {
        logger_->debug("Unable to perform getJointForceTorques(), arm was not initialized");
        return boost::none;
    }
    crf::utility::types::JointForceTorques torque(configuration_->getNumberOfJoints());
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        torque[i] = arm_.at(i)->getTorque().value();
    }
    return torque;
}

boost::optional<utility::types::TaskPose> PilzArm::getTaskPose() {
    logger_->warn("getTaskPose() not implemented");
    return boost::none;
}

boost::optional<utility::types::TaskVelocity> PilzArm::getTaskVelocity() {
    logger_->warn("getTaskVelocity() not implemented");
    return boost::none;
}

boost::optional<crf::utility::types::TaskForceTorque> PilzArm::getTaskForceTorque() {
    logger_->warn("getTaskVelocity() not implemented");
    return boost::none;
}

bool PilzArm::setJointPositions(const utility::types::JointPositions& jointPositions) {
    logger_->warn("setJointPositions() not implemented");
    return false;
}

bool PilzArm::setJointPositions(const utility::types::JointPositions& jointPositions,
    const utility::types::JointVelocities& jointVelocities,
    bool relative) {
    logger_->warn("setJointPositions() not implemented");
    return false;
}

bool PilzArm::setJointPositions(const utility::types::JointPositions& jointPositions,
    const utility::types::JointVelocities& jointVelocities,
    const utility::types::JointAccelerations& jointAccelerations, // NOLINT
    bool relative) {
    logger_->warn("setJointPositions() not implemented");
    return false;
}

bool PilzArm::setJointVelocities(const utility::types::JointVelocities& jointVelocities) {
    logger_->debug("setJointVelocities");
    if (!initialized_) {
        logger_->debug("Unable to setJointVelocities(), arm was not initialized");
        return false;
    }
    auto jointsConfiguration = configuration_->getJointsConfiguration();
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        if (fabs(jointVelocities[i]) > jointsConfiguration[i].maximumVelocity) {
            logger_->warn("Requested velocity out of range.");
            return false;
        }
    }
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        arm_.at(i)->setVelocity(jointVelocities[i]*radToMilliDeg_);
    }
    return true;
}

bool PilzArm::setJointVelocities(const utility::types::JointVelocities& jointVelocities,
    const utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointVelocities");
    if (!initialized_) {
        logger_->debug("Unable to setJointVelocities(), arm was not initialized");
        return false;
    }
    auto jointsConfiguration = configuration_->getJointsConfiguration();
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        if (fabs(jointVelocities[i]) > jointsConfiguration[i].maximumVelocity) {
            logger_->warn("Requested velocity out of range.");
            return false;
        }
    }
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        if (fabs(jointAccelerations[i]) > jointsConfiguration[i].maximumAcceleration) {
            logger_->warn("Requested acceleration out of range.");
            return false;
        }
    }
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        arm_.at(i)->setVelocity(jointVelocities[i]*radToMilliDeg_, jointAccelerations[i]*radToMilliDeg_); // NOLINT
    }
    return true;
}

bool PilzArm::setJointVelocities(const utility::types::JointVelocities& jointVelocities,
    const utility::types::JointAccelerations& jointAccelerations,
    const utility::types::JointAccelerations& jointsDeceleration) {
    logger_->debug("setJointVelocities");
    if (!initialized_) {
        logger_->debug("Unable to setJointVelocities(), arm was not initialized");
        return false;
    }
    auto jointsConfiguration = configuration_->getJointsConfiguration();
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        if (fabs(jointVelocities[i]) > jointsConfiguration[i].maximumVelocity) {
            logger_->warn("Requested velocity out of range.");
            return false;
        }
    }
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        if (fabs(jointAccelerations[i]) > jointsConfiguration[i].maximumAcceleration) {
            logger_->warn("Requested acceleration out of range.");
            return false;
        }
    }
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        if (fabs(jointsDeceleration[i]) > jointsConfiguration[i].maximumAcceleration) {
            logger_->warn("Requested deceleration out of range.");
            return false;
        }
    }
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        arm_.at(i)->setVelocity(jointVelocities[i]*radToMilliDeg_,
            jointAccelerations[i]*radToMilliDeg_, jointsDeceleration[i]*radToMilliDeg_);
    }
    return true;
}

bool PilzArm::setJointForceTorques(
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->warn("setJointForceTorques() not implemented");
    return false;
}

bool PilzArm::setTaskPose(const utility::types::TaskPose& goalPosition) {
    logger_->warn("setTaskPose() not implemented");
    return false;
}

bool PilzArm::setTaskVelocity(const utility::types::TaskVelocity& velocity, bool TCP) { // NOLINT
    logger_->warn("setTaskVelocity() not supported");
    return false;
}

bool PilzArm::stopArm() {
    logger_->debug("stopArm()");
    if (!initialized_) {
        logger_->debug("Unable to stopArm(), arm was not initialized");
        return false;
    }
    return softStop();
}

bool PilzArm::enableBrakes() {
    logger_->debug("enableBrakes() not supported");
    return false;
}

bool PilzArm::disableBrakes() {
    logger_->debug("disableBrakes() not supported");
    return false;
}

std::shared_ptr<actuators::robotarm::RobotArmConfiguration> PilzArm::getConfiguration() {
    logger_->debug("getConfiguration()");
    return configuration_;
}

bool PilzArm::softStop() {
    logger_->debug("softStop()");
    for (uint8_t i = 0; i < configuration_->getNumberOfJoints(); i++) {
        arm_.at(i)->stop();
    }
    return true;
}

}  // namespace crf::actuators::pilzarm
