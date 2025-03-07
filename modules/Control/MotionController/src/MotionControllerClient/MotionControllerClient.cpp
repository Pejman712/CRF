/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include "MotionController/MotionControllerClient/MotionControllerClient.hpp"

namespace crf {
namespace control {
namespace motioncontroller {

MotionControllerClient::MotionControllerClient(
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
    logger_("MotionControllerClient") {
    logger_->debug("CTor");

    receivers_.insert({"startJointsControlLoop", &receiverStartJointsControlLoop_});
    receivers_.insert({"startTaskControlLoop", &receiverStartTaskControlLoop_});
    receivers_.insert({"appendPath", &receiverAppendPath_});
    receivers_.insert({"setVelocity", &receiverSetVelocity_});
    receivers_.insert({"setTorque", &receiverSetTorque_});
    receivers_.insert({"setProfileVelocity", &receiverSetProfileVelocity_});
    receivers_.insert({"setProfileAcceleration", &receiverSetProfileAcceleration_});
    receivers_.insert({"softStop", &receiverSoftStop_});
    receivers_.insert({"hardStop", &receiverHardStop_});
    receivers_.insert({"setParameters", &receiverSetParameters_});
}

MotionControllerClient::~MotionControllerClient() {
    logger_->debug("DTor");
}

bool MotionControllerClient::initialize() {
    return PriorityAccessClient::initialize();
}
bool MotionControllerClient::deinitialize() {
    return PriorityAccessClient::deinitialize();
}

void MotionControllerClient::startJointsControlLoop() {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "startJointsControlLoop";
    json.data["priority"] = priority_;
    sendPacket(json, receiverStartJointsControlLoop_);
    return;
}
void MotionControllerClient::startTaskControlLoop() {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "startTaskControlLoop";
    json.data["priority"] = priority_;
    sendPacket(json, receiverStartTaskControlLoop_);
    return;
}

crf::expected<bool> MotionControllerClient::appendPath(const std::vector<JointPositions>& path) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "appendPath";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = path;
    return sendPacket(json, receiverAppendPath_);
}

crf::expected<bool> MotionControllerClient::appendPath(
    const std::vector<TaskPose>& path,
    const TrajectoryExecutionMethod& method,
    const PointReferenceFrame& frame) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "appendPath";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = path;
    json.data["method"] = method;
    json.data["reference"] = frame;
    return sendPacket(json, receiverAppendPath_);
}

crf::expected<bool> MotionControllerClient::setVelocity(const JointVelocities& velocity) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setVelocity";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = velocity;
    return sendPacket(json, receiverSetVelocity_);
}

crf::expected<bool> MotionControllerClient::setVelocity(
    const TaskVelocity& velocity,
    const PointReferenceFrame& frame) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setVelocity";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = velocity;
    json.data["reference"] = frame;
    return sendPacket(json, receiverSetVelocity_);
}

crf::expected<bool> MotionControllerClient::setTorque(const JointForceTorques& torque) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setTorque";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = torque;
    return sendPacket(json, receiverSetTorque_);
}

crf::expected<bool> MotionControllerClient::setTorque(
    const TaskForceTorque& torque,
    const PointReferenceFrame& frame) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setTorque";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = torque;
    json.data["reference"] = frame;
    return sendPacket(json, receiverSetTorque_);
}

crf::expected<bool> MotionControllerClient::setProfileVelocity(const JointVelocities& velocity) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileVelocity";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = velocity;
    return sendPacket(json, receiverSetProfileVelocity_);
}

crf::expected<bool> MotionControllerClient::setProfileVelocity(const TaskVelocity& velocity) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileVelocity";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = velocity;
    return sendPacket(json, receiverSetProfileVelocity_);
}

crf::expected<bool> MotionControllerClient::setProfileAcceleration(
    const JointAccelerations& acceleration) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileAcceleration";
    json.data["priority"] = priority_;
    json.data["type"] = "joints";
    json.data["data"] = acceleration;
    return sendPacket(json, receiverSetProfileAcceleration_);
}

crf::expected<bool> MotionControllerClient::setProfileAcceleration(
    const TaskAcceleration& acceleration) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setProfileAcceleration";
    json.data["priority"] = priority_;
    json.data["type"] = "task";
    json.data["data"] = acceleration;
    return sendPacket(json, receiverSetProfileAcceleration_);
}


void MotionControllerClient::softStop() {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "softStop";
    json.data["priority"] = priority_;
    sendPacket(json, receiverSoftStop_);
}

void MotionControllerClient::hardStop() {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "hardStop";
    json.data["priority"] = priority_;
    sendPacket(json, receiverHardStop_);
}

crf::expected<bool> MotionControllerClient::setParameters(const nlohmann::json& params) {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setParameters";
    json.data["priority"] = priority_;
    json.data["data"] = params;
    return sendPacket(json, receiverSetParameters_);
}

nlohmann::json MotionControllerClient::getCurrentParameters() {
    return nlohmann::json();
}

nlohmann::json MotionControllerClient::getParametersDefinition() const {
    return nlohmann::json();
}

crf::expected<bool> MotionControllerClient::isTrajectoryRunning() {
    if (!requestStatus()) return crf::Code::BadGateway;
    return isTrajectoryRunning_;
}

Signals MotionControllerClient::getSignals() {
    if (!requestStatus()) return Signals();
    return robotSignals_;
}

std::set<crf::Code> MotionControllerClient::getStatus() {
    if (!requestStatus()) return {crf::Code::BadRequest};
    return controllerStatus_;
}

std::shared_ptr<actuators::robot::RobotConfiguration> MotionControllerClient::getConfiguration() {
    return nullptr;
}

// Private

void MotionControllerClient::parseStatus(const nlohmann::json& json) {
    try {
        priorityUnderControl_ = json["priorityUnderControl"].get<uint32_t>();
        isTrajectoryRunning_ = json["isTrajectoryRunning"].get<crf::expected<bool>>();
        controllerStatus_ = json["controllerStatus"].get<std::set<crf::Code>>();
        robotSignals_.joints.positions = json["jointPositions"].
            get<crf::expected<crf::utility::types::JointPositions>>();
        robotSignals_.joints.velocities = json["jointVelocities"].
            get<crf::expected<crf::utility::types::JointVelocities>>();
        robotSignals_.joints.accelerations = json["jointAccelerations"].
            get<crf::expected<crf::utility::types::JointAccelerations>>();
        robotSignals_.joints.forceTorques = json["jointForceTorques"].
            get<crf::expected<crf::utility::types::JointForceTorques>>();
        robotSignals_.task.pose = json["taskPose"].
            get<crf::expected<crf::utility::types::TaskPose>>();
        robotSignals_.task.velocity = json["taskVelocity"].
            get<crf::expected<crf::utility::types::TaskVelocity>>();
        robotSignals_.task.acceleration = json["taskAcceleration"].
            get<crf::expected<crf::utility::types::TaskAcceleration>>();
        robotSignals_.task.forceTorque = json["taskForceTorque"].
            get<crf::expected<crf::utility::types::TaskForceTorque>>();
    } catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
    }
}

crf::expected<bool> MotionControllerClient::sendPacket(
    const communication::datapackets::JSONPacket& json,
    const Receiver<crf::expected<bool>>& receiver) {
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

    std::optional<crf::expected<bool>> result = receiver.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive answer for {}", json.data);
        return false;
    }
    unlockControl();
    return result.value();
}

}  // namespace motioncontroller
}  // namespace control
}  // namespace crf
