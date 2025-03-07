/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <map>
#include <string>
#include <functional>
#include <atomic>

#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf::actuators::timrpwagon {

TIMRPWagonCommunicationPoint::TIMRPWagonCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::actuators::timrpwagon::TIMRPWagonManager> timRPWagonManager) :
    PriorityAccessCommunicationPoint(socket, timRPWagonManager),
    timRPWagonManager_(timRPWagonManager),
    logger_("TIMRPWagonCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "retractRPArm",
        std::bind(&TIMRPWagonCommunicationPoint::retractRPArmHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "deployRPArm",
        std::bind(&TIMRPWagonCommunicationPoint::deployRPArmHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "moveRPArmUp",
        std::bind(&TIMRPWagonCommunicationPoint::moveRPArmUpHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "moveRPArmDown",
        std::bind(&TIMRPWagonCommunicationPoint::moveRPArmDownHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stopRPArm",
        std::bind(&TIMRPWagonCommunicationPoint::stopRPArmHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "lockRPArm",
        std::bind(&TIMRPWagonCommunicationPoint::lockRPArmHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "unlockRPArm",
        std::bind(&TIMRPWagonCommunicationPoint::unlockRPArmHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "resetRPArmDriver",
        std::bind(&TIMRPWagonCommunicationPoint::resetRPArmDriverHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "acknowledgeErrors",
        std::bind(&TIMRPWagonCommunicationPoint::acknowledgeErrorsHandler,
            this, std::placeholders::_1)});
}

TIMRPWagonCommunicationPoint::~TIMRPWagonCommunicationPoint() {
}

void TIMRPWagonCommunicationPoint::retractRPArmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("retractRPArmHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->retractRPArm(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::deployRPArmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("deployRPArmHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->deployRPArm(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::moveRPArmUpHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("moveRPArmUpHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->moveRPArmUp(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::moveRPArmDownHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("moveRPArmDownHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->moveRPArmDown(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::stopRPArmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopRPArmHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->stopRPArm(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::lockRPArmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("lockRPArmHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->lockRPArm(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::unlockRPArmHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("unlockRPArmHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->unlockRPArm(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::resetRPArmDriverHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("resetRPArmDriverHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->resetRPArmDriver(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

void TIMRPWagonCommunicationPoint::acknowledgeErrorsHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("acknowledgeErrorsHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::optional<bool> result = timRPWagonManager_->acknowledgeErrors(priority);
    if (!result) {
        sendJSONError(packet, "Failed to send or execute");
    } else {
        sendJSONReply<bool>(packet, result.value());
    }
    return;
}

}  // namespace crf::actuators::timrpwagon
