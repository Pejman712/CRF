/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>

#include <boost/optional/optional.hpp>
#include <nlohmann/json.hpp>

#include "MechanicalStabilizer/MechanicalStabilizerClient/MechanicalStabilizerClient.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf::actuators::mechanicalstabilizer {

MechanicalStabilizerClient::MechanicalStabilizerClient(
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
        logger_("MechanicalStabilizerClient") {
            logger_->debug("CTor");

            receivers_.insert({"activate", &receiverActivate_});
            receivers_.insert({"deactivate", &receiverDeactivate_});
            receivers_.insert({"resetFaultState", &receiverResetFaultState_});
}

MechanicalStabilizerClient::~MechanicalStabilizerClient() {
    logger_->debug("DTor");
    deinitialize();
}

bool MechanicalStabilizerClient::initialize() {
    return PriorityAccessClient::initialize();
}

bool MechanicalStabilizerClient::deinitialize() {
    return PriorityAccessClient::deinitialize();
}

bool MechanicalStabilizerClient::activate() {
    logger_->debug("open");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "activate";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverActivate_);
}

bool MechanicalStabilizerClient::deactivate() {
    logger_->debug("close");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "deactivate";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverDeactivate_);
}

std::optional<bool> MechanicalStabilizerClient::isActivated() {
    logger_->debug("isActivated");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusIsActivated_;
}

std::optional<bool> MechanicalStabilizerClient::isDeactivated() {
    logger_->debug("isDeactivated");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusIsDeactivated_;
}

bool MechanicalStabilizerClient::resetFaultState() {
    logger_->debug("resetFaultState");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "resetFaultState";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverResetFaultState_);
}

std::optional<bool> MechanicalStabilizerClient::isInFault() {
    logger_->debug("isInFault");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusIsInFault_;
}

void MechanicalStabilizerClient::parseStatus(const nlohmann::json& json) {
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    try {
        if (json.contains("active")) {
            statusIsActivated_ = json["active"].get<bool>();
            statusIsDeactivated_ = !json["active"].get<bool>();
        } else {
            statusIsActivated_ = std::nullopt;
            statusIsDeactivated_ = std::nullopt;
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

bool MechanicalStabilizerClient::sendPacket(const communication::datapackets::JSONPacket& json,
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

}  // namespace crf::actuators::mechanicalstabilizer
