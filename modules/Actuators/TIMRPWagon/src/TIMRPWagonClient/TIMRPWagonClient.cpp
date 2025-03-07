/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "TIMRPWagon/TIMRPWagonClient/TIMRPWagonClient.hpp"

namespace crf::actuators::timrpwagon {

TIMRPWagonClient::TIMRPWagonClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds serverReplyTimeout,
    const float frequency,
    const uint32_t priority) :
    crf::utility::devicemanager::PriorityAccessClient(
        socket,
        serverReplyTimeout,
        frequency,
        priority),
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    priority_(priority),
    logger_("TIMRPWagonClient") {
    logger_->debug("CTor");

    receivers_.insert({"retractRPArm", &receiverRetractRPArm_});
    receivers_.insert({"deployRPArm", &receiverDeployRPArm_});
    receivers_.insert({"moveRPArmUp", &receiverMoveRPArmUp_});
    receivers_.insert({"moveRPArmDown", &receiverMoveRPArmDown_});
    receivers_.insert({"stopRPArm", &receiverStopRPArm_});
    receivers_.insert({"lockRPArm", &receiverLockRPArm_});
    receivers_.insert({"unlockRPArm", &receiverUnlockRPArm_});
    receivers_.insert({"resetRPArmDriver", &receiverResetRPArmDriver_});
    receivers_.insert({"acknowledgeErrors", &receiverAcknowledgeErrors_});
}

TIMRPWagonClient::~TIMRPWagonClient() {
    logger_->debug("DTor");
}

bool TIMRPWagonClient::initialize() {
    return PriorityAccessClient::initialize();
}

bool TIMRPWagonClient::deinitialize() {
    return PriorityAccessClient::deinitialize();
}

bool TIMRPWagonClient::isConnected() {
    logger_->debug("isConnected");
    if (!requestStatus()) return false;
    return statusIsConnected_.load();
}

RPArmPosition TIMRPWagonClient::getRPArmPosition() {
    logger_->debug("getRPArmPosition");
    if (!requestStatus()) return RPArmPosition::NotDefined;
    return statusRPArmPosition_.load();
}

std::optional<bool> TIMRPWagonClient::retractRPArm() {
    logger_->debug("retractRPArm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "retractRPArm";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverRetractRPArm_);
}

std::optional<bool> TIMRPWagonClient::deployRPArm() {
    logger_->debug("deployRPArm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "deployRPArm";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverDeployRPArm_);
}

std::optional<bool> TIMRPWagonClient::moveRPArmUp() {
    logger_->debug("moveRPArmUp");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "moveRPArmUp";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverRetractRPArm_);
}

std::optional<bool> TIMRPWagonClient::moveRPArmDown() {
    logger_->debug("moveRPArmDown");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "moveRPArmDown";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverMoveRPArmDown_);
}

std::optional<bool> TIMRPWagonClient::stopRPArm() {
    logger_->debug("stopRPArm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "stopRPArm";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverStopRPArm_);
}

std::optional<bool> TIMRPWagonClient::lockRPArm() {
    logger_->debug("lockRPArm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "lockRPArm";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverLockRPArm_);
}

std::optional<bool> TIMRPWagonClient::unlockRPArm() {
    logger_->debug("unlockRPArm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "unlockRPArm";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverUnlockRPArm_);
}

std::optional<bool> TIMRPWagonClient::isRPArmInError() {
    logger_->debug("isRPArmInError");
    if (!requestStatus()) return std::nullopt;
    return statusIsRPArmInError_.load();
}

std::optional<bool> TIMRPWagonClient::resetRPArmDriver() {
    logger_->debug("resetRPArmDriver");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "resetRPArmDriver";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverResetRPArmDriver_);
}

std::optional<bool> TIMRPWagonClient::acknowledgeErrors() {
    logger_->debug("acknowledgeErrors");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "acknowledgeErrors";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverAcknowledgeErrors_);
}

std::shared_ptr<TIMRPWagonConfiguration> TIMRPWagonClient::getConfiguration() {
    logger_->debug("getConfiguration");
    // Not implemented in the Comm Point
    return nullptr;
}

// Private

void TIMRPWagonClient::parseStatus(const nlohmann::json& json) {
    try {
        statusRPArmPosition_ = json["rpArmPosition"].get<RPArmPosition>();
        statusIsConnected_ = json["isConnected"].get<bool>();
        statusIsRPArmInError_ = json["isRPArmInError"].get<bool>();
    } catch (const std::exception& e) {
        logger_->error("Couldn't update status with message {}", json);
    }
}

bool TIMRPWagonClient::sendPacket(const communication::datapackets::JSONPacket& json,
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

}  // namespace crf::actuators::timrpwagon
