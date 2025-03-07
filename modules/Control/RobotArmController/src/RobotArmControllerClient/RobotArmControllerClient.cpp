/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "RobotArmController/RobotArmControllerClient/RobotArmControllerClient.hpp"

namespace crf::control::robotarmcontroller {

RobotArmControllerClient::RobotArmControllerClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds serverReplyTimeout,
    const float frequency,
    const uint32_t priority):
    crf::utility::devicemanager::PriorityAccessClient(
        socket,
        serverReplyTimeout,
        frequency,
        priority),
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    priority_(priority),
    statusJointPositions_(1),  // Size will change to the correct one with the first status received
    statusJointVelocities_(1),
    statusJointAccelerations_(1),
    statusJointForceTorques_(1),
    logger_("RobotArmControllerClient") {
        logger_->debug("CTor");

        receivers_.insert({"setMode", &receiverSetMode_});
        receivers_.insert({"setPos", &receiverSetPos_});
        receivers_.insert({"setVel", &receiverSetVel_});
        receivers_.insert({"setAcc", &receiverSetAcc_});
        receivers_.insert({"interrupt", &receiverInterrupt_});
        receivers_.insert({"setJoiMaxVel", &receiverSetJoiMaxVel_});
        receivers_.insert({"setJoiMaxAcc", &receiverSetJoiMaxAcc_});
        receivers_.insert({"setTaskMaxVel", &receiverSetTaskMaxVel_});
        receivers_.insert({"setTaskMaxAcc", &receiverSetTaskMaxAcc_});
        receivers_.insert({"gripPos", &receiverSetGripPos_});
        receivers_.insert({"gripVel", &receiverSetGripVel_});
        receivers_.insert({"trajectoryResult", &receiverTrajectoryResult_});
}

RobotArmControllerClient::~RobotArmControllerClient() {
    logger_->debug("DTor");
}

bool RobotArmControllerClient::initialize() {
    return PriorityAccessClient::initialize();
}
bool RobotArmControllerClient::deinitialize() {
    return PriorityAccessClient::deinitialize();
}

std::future<bool> RobotArmControllerClient::setPosition(
    const crf::utility::types::JointPositions& position) {
    logger_->debug("setPosition(joi)");
    std::vector<crf::utility::types::JointPositions> path;
    path.push_back(position);
    return setPosition(path);
}

std::future<bool> RobotArmControllerClient::setPosition(
    const std::vector<crf::utility::types::JointPositions>& positions) {
    logger_->debug("setPosition(vec(joi))");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::future<bool>();
    }
    if (!lockControl()) {
        return std::future<bool>();
    }
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setPos";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = positions;

    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), false);
    socketLock.unlock();

    std::optional<bool> result = receiverSetPos_.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive answer for {}", json.data);
        return std::future<bool>();
    }
    if (!result.value()) {
        logger_->error("Not able to set position");
        return std::future<bool>();
    }
    return std::async(std::launch::async, [this]() {
        return waitJointsTrajResult();
    });
}

std::future<bool> RobotArmControllerClient::setPosition(
    const crf::utility::types::TaskPose& position,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setPosition(task)");
    std::vector<crf::utility::types::TaskPose> path;
    path.push_back(position);
    return setPosition(path, method, frame);
}

std::future<bool> RobotArmControllerClient::setPosition(
    const std::vector<crf::utility::types::TaskPose>& positions,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setPosition(vec(task))");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::future<bool>();
    }
    if (!lockControl()) {
        return std::future<bool>();
    }
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setPos";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = positions;
    json.data["method"] = method;
    json.data["reference"] = frame;

    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), false);
    socketLock.unlock();

    std::optional<bool> result = receiverSetPos_.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive answer for {}", json.data);
        return std::future<bool>();
    }
    if (!result.value()) {
        logger_->error("Not able to set position");
        return std::future<bool>();
    }
    return std::async(std::launch::async, [this]() {
        return waitTaskTrajResult();
    });
}

bool RobotArmControllerClient::setVelocity(
    const crf::utility::types::JointVelocities& velocity) {
    logger_->debug("setVelocity(joi)");
    std::vector<crf::utility::types::JointVelocities> path;
    path.push_back(velocity);
    return setVelocity(path);
}

bool RobotArmControllerClient::setVelocity(
    const std::vector<crf::utility::types::JointVelocities>& velocities) {
    logger_->debug("setVelocity(vec(joi))");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setVel";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = velocities;
    return sendPacket(json, receiverSetVel_);
}

bool RobotArmControllerClient::setVelocity(
    const crf::utility::types::TaskVelocity& velocity,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setVelocity(task)");
    std::vector<crf::utility::types::TaskVelocity> path;
    path.push_back(velocity);
    return setVelocity(path, method, frame);
}

bool RobotArmControllerClient::setVelocity(
    const std::vector<crf::utility::types::TaskVelocity>& velocities,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setVelocity(vec(task))");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setVel";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = velocities;
    json.data["method"] = method;
    json.data["reference"] = frame;
    return sendPacket(json, receiverSetVel_);
}

bool RobotArmControllerClient::setAcceleration(
    const crf::utility::types::JointAccelerations& acceleration) {
    logger_->debug("setVelocity(joi)");
    std::vector<crf::utility::types::JointAccelerations> path;
    path.push_back(acceleration);
    return setAcceleration(path);
}

bool RobotArmControllerClient::setAcceleration(
    const std::vector<crf::utility::types::JointAccelerations>& accelerations) {
    logger_->debug("setAcceleration(vec(joi))");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setAcc";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = accelerations;
    return sendPacket(json, receiverSetAcc_);
}

bool RobotArmControllerClient::setAcceleration(
    const crf::utility::types::TaskAcceleration& acceleration,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setVelocity(task)");
    std::vector<crf::utility::types::TaskAcceleration> path;
    path.push_back(acceleration);
    return setAcceleration(path, method, frame);
}

bool RobotArmControllerClient::setAcceleration(
    const std::vector<crf::utility::types::TaskAcceleration>& accelerations,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setAcceleration(vec(task))");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setAcc";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = accelerations;
    json.data["method"] = method;
    json.data["reference"] = frame;
    return sendPacket(json, receiverSetAcc_);
}

bool RobotArmControllerClient::interruptTrajectory() {
    logger_->debug("interruptTrajectory");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "interrupt";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverInterrupt_);
}

crf::utility::types::JointPositions RobotArmControllerClient::getJointPositions() {
    logger_->debug("getJointPositions");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusJointPositions_;
}

crf::utility::types::TaskPose RobotArmControllerClient::getTaskPose() {
    logger_->debug("getTaskPose");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusTaskPose_;
}

crf::utility::types::JointVelocities RobotArmControllerClient::getJointVelocities() {
    logger_->debug("getJointVelocities");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusJointVelocities_;
}

crf::utility::types::TaskVelocity RobotArmControllerClient::getTaskVelocity() {
    logger_->debug("getTaskVelocity");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusTaskVelocity_;
}

crf::utility::types::JointAccelerations RobotArmControllerClient::getJointAccelerations() {
    logger_->debug("getJointAccelerations");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusJointAccelerations_;
}

crf::utility::types::TaskAcceleration RobotArmControllerClient::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusTaskAcceleration_;
}

crf::utility::types::JointForceTorques RobotArmControllerClient::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusJointForceTorques_;
}

bool RobotArmControllerClient::setJointsMaximumVelocity(
    const crf::utility::types::JointVelocities& velocity) {
    logger_->debug("setJointsMaximumVelocity");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setJoiMaxVel";
    json.data["priority"] = priority_;
    json.data["data"] = velocity;
    return sendPacket(json, receiverSetJoiMaxVel_);
}

bool RobotArmControllerClient::setJointsMaximumAcceleration(
    const crf::utility::types::JointAccelerations& acceleration) {
    logger_->debug("setJointsMaximumAcceleration");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setJoiMaxAcc";
    json.data["priority"] = priority_;
    json.data["data"] = acceleration;
    return sendPacket(json, receiverSetJoiMaxAcc_);
}

bool RobotArmControllerClient::setTaskMaximumVelocity(
    const crf::utility::types::TaskVelocity& velocity) {
    logger_->debug("setTaskMaximumVelocity");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setTaskMaxVel";
    json.data["priority"] = priority_;
    json.data["data"] = velocity;
    return sendPacket(json, receiverSetTaskMaxVel_);
}

bool RobotArmControllerClient::setTaskMaximumAcceleration(
    const crf::utility::types::TaskAcceleration& acceleration) {
    logger_->debug("setTaskMaximumAcceleration");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setTaskMaxAcc";
    json.data["priority"] = priority_;
    json.data["data"] = acceleration;
    return sendPacket(json, receiverSetTaskMaxAcc_);
}

// Private methods

void RobotArmControllerClient::parseStatus(const nlohmann::json& json) {
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    try {
        statusJointPositions_ = json["jointPositions"].get<crf::utility::types::JointPositions>();
        statusJointVelocities_ = json["jointVelocities"].get<crf::utility::types::JointVelocities>(); // NOLINT
        statusJointAccelerations_ = json["jointAccelerations"].get<crf::utility::types::JointAccelerations>();  // NOLINT
        statusJointForceTorques_ = json["jointForceTorques"].get<crf::utility::types::JointForceTorques>(); // NOLINT
        statusTaskPose_ = json["taskPose"].get<crf::utility::types::TaskPose>();
        statusTaskVelocity_ = json["taskVelocity"].get<crf::utility::types::TaskVelocity>();
        statusTaskAcceleration_ = json["taskAcceleration"].get<crf::utility::types::TaskAcceleration>();  // NOLINT
    } catch (const std::exception& e) {
        logger_->error("Couldn't update status with message {}", json);
    }
}

bool RobotArmControllerClient::setMode(crf::control::robotarmcontroller::ControllerMode mode) {
    logger_->debug("setMode");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "unlockControl";
    json.data["priority"] = priority_;
    json.data["mode"] = mode;
    return sendPacket(json, receiverSetMode_);
}

bool RobotArmControllerClient::waitJointsTrajResult() {
    logger_->debug("waitJointsTrajResult");
    if (!receiverTrajectoryResult_.wait()) {
        logger_->error("Trajectory failed");
        return false;
    }
    unlockControl();
    return true;
}

bool RobotArmControllerClient::waitTaskTrajResult() {
    logger_->debug("waitTaskTrajResult");
    if (!receiverTrajectoryResult_.wait()) {
        logger_->error("Trajectory failed");
        return false;
    }
    unlockControl();
    return true;
}

bool RobotArmControllerClient::sendPacket(const communication::datapackets::JSONPacket& json,
    const Receiver<bool>& receiver) {
    logger_->debug("sendPacket: {}", json.data);
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (!lockControl()) {
        return false;
    }
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader(), false);
    socketLock.unlock();

    std::optional<bool> result = receiver.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive answer for {}", json.data);
        return false;
    }
    unlockControl();
    return result.value();
}

}  // namespace crf::control::robotarmcontroller
