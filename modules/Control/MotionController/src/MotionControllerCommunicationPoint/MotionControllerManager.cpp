/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *         Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>

#include "MotionController/MotionControllerCommunicationPoint/MotionControllerManager.hpp"

namespace crf::control::motioncontroller {

MotionControllerManager::MotionControllerManager(
    std::shared_ptr<crf::control::motioncontroller::IMotionController> controller,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    DeviceManagerWithPriorityAccess(controller, initializationTimeout, controlAccessTimeout),
    controller_(controller) {
    logger_ = crf::utility::logger::EventLogger("MotionControllerManager");
    logger_->debug("CTor");
}

MotionControllerManager::~MotionControllerManager() {
    logger_->debug("DTor");
}

crf::expected<bool> MotionControllerManager::startJointsControlLoop(const int& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    controller_->startJointsControlLoop();
    return true;
}

crf::expected<bool> MotionControllerManager::startTaskControlLoop(const int& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    controller_->startTaskControlLoop();
    return true;
}

crf::expected<bool> MotionControllerManager::appendPath(
    const int priority,
    const std::vector<JointPositions>& path) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->appendPath(path);
}

crf::expected<bool> MotionControllerManager::appendPath(
    const int priority,
    const std::vector<TaskPose>& path,
    const TrajectoryExecutionMethod& method,
    const PointReferenceFrame& frame) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->appendPath(path, method, frame);
}

crf::expected<bool> MotionControllerManager::setVelocity(
    const int priority,
    const crf::utility::types::JointVelocities& velocity) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setVelocity(velocity);
}

crf::expected<bool> MotionControllerManager::setVelocity(
    const int& priority,
    const crf::utility::types::TaskVelocity& velocity,
    const PointReferenceFrame& frame) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setVelocity(velocity, frame);
}

crf::expected<bool> MotionControllerManager::setTorque(
    const int priority,
    const JointForceTorques& torque) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setTorque(torque);
}

crf::expected<bool> MotionControllerManager::setTorque(
    const int priority,
    const TaskForceTorque& torque,
    const PointReferenceFrame& frame) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setTorque(torque, frame);
}

crf::expected<bool> MotionControllerManager::setProfileVelocity(
    const int priority,
    const JointVelocities& velocity) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setProfileVelocity(velocity);
}
crf::expected<bool> MotionControllerManager::setProfileVelocity(
    const int& priority,
    const TaskVelocity& velocity) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setProfileVelocity(velocity);
}

crf::expected<bool> MotionControllerManager::setProfileAcceleration(
    const int priority,
    const JointAccelerations& acceleration) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setProfileAcceleration(acceleration);
}

crf::expected<bool> MotionControllerManager::setProfileAcceleration(
    const int& priority,
    const TaskAcceleration& acceleration) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setProfileAcceleration(acceleration);
}

crf::expected<bool> MotionControllerManager::softStop(const int& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    controller_->softStop();
    return true;
}

crf::expected<bool> MotionControllerManager::hardStop(const int& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    controller_->hardStop();
    return true;
}

crf::expected<bool> MotionControllerManager::setParameters(
    const int& priority,
    const nlohmann::json& params) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->setParameters(params);
}

nlohmann::json MotionControllerManager::getCurrentParameters(
    const int& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->getCurrentParameters();
}

nlohmann::json MotionControllerManager::getParametersDefinition(const int& priority) {
    if (!checkCommandPriority(priority)) return crf::Code::Forbidden;
    return controller_->getParametersDefinition();
}

nlohmann::json MotionControllerManager::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json statusJSON;

    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    uint32_t priorityUnderControl = simpleAccessControl_.getHighestPriority();
    statusJSON["priorityUnderControl"] = priorityUnderControl;
    statusJSON["isTrajectoryRunning"] = controller_->isTrajectoryRunning();

    statusJSON["controllerStatus"] = controller_->getStatus();
    Signals sig = controller_->getSignals();
    statusJSON["jointPositions"] = sig.joints.positions;
    statusJSON["jointVelocities"] = sig.joints.velocities;
    statusJSON["jointAccelerations"] = sig.joints.accelerations;
    statusJSON["jointForceTorques"] = sig.joints.forceTorques;
    statusJSON["taskPose"] = sig.task.pose;
    statusJSON["taskVelocity"] = sig.task.velocity;
    statusJSON["taskAcceleration"] = sig.task.acceleration;
    statusJSON["taskForceTorque"] = sig.task.forceTorque;
    return statusJSON;
}

nlohmann::json MotionControllerManager::getConfiguration() {
    return controller_->getConfiguration()->getConfigurationFile();
}

}  // namespace crf::control::motioncontroller
