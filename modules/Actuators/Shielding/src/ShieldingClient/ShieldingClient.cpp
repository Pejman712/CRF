/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <map>
#include <string>

#include "Shielding/ShieldingClient/ShieldingClient.hpp"

namespace crf::actuators::shielding {

ShieldingClient::ShieldingClient(
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
    frequency_(frequency),
    priority_(priority),
    logger_("ShieldingClient") {
        logger_->debug("CTor");

        receivers_.insert({"open", &receiverOpen_});
        receivers_.insert({"close", &receiverClose_});
        receivers_.insert({"resetFaultState", &receiverResetFaultState_});
}

ShieldingClient::~ShieldingClient() {
    logger_->debug("DTor");
    deinitialize();
}

bool ShieldingClient::initialize() {
    return PriorityAccessClient::initialize();
}

bool ShieldingClient::deinitialize() {
    return PriorityAccessClient::deinitialize();
}

bool ShieldingClient::open() {
    logger_->debug("open");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "open";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverOpen_);
}

bool ShieldingClient::close() {
    logger_->debug("close");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "close";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverClose_);
}

std::optional<bool> ShieldingClient::isOpen() {
    logger_->debug("isOpen");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusIsOpen_;
}

std::optional<bool> ShieldingClient::isClosed() {
    logger_->debug("isClosed");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusIsClosed_;
}

bool ShieldingClient::resetFaultState() {
    logger_->debug("resetFaultState");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "resetFaultState";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverResetFaultState_);
}

std::optional<bool> ShieldingClient::isInFault() {
    logger_->debug("isInFault");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusIsInFault_;
}

void ShieldingClient::parseStatus(const nlohmann::json& json) {
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    try {
        std::string status = json["position"].get<std::string>();
        if (status == "open") {
            statusIsOpen_ = true;
            statusIsClosed_  = false;
        } else if (status == "closed") {
            statusIsOpen_ = false;
            statusIsClosed_  = true;
        } else {
            statusIsOpen_ = std::nullopt;
            statusIsClosed_  = std::nullopt;
        }
        if (json.contains("isInFault")) {
            statusIsInFault_ = json["isInFault"].get<bool>();
        } else {
            statusIsInFault_ = std::nullopt;
        }
    } catch (const std::exception& e) {
        logger_->error("Couldn't update status with message {}", json);
    }
}

bool ShieldingClient::sendPacket(const communication::datapackets::JSONPacket& json,
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

}  // namespace crf::actuators::shielding
