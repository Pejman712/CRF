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

#include "Shielding/ShieldingCommunicationPoint/ShieldingCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "Shielding/ShieldingCommunicationPoint/ShieldingManager.hpp"

namespace crf::actuators::shielding {

ShieldingCommunicationPoint::ShieldingCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::actuators::shielding::ShieldingManager> manager) :
    crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        socket,
        manager),
    socket_(socket),
    manager_(manager),
    logger_("ShieldingCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "open",
        std::bind(&ShieldingCommunicationPoint::openRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "close",
        std::bind(&ShieldingCommunicationPoint::closeRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "resetFaultState",
        std::bind(&ShieldingCommunicationPoint::resetFaultStateRequestHandler,
            this, std::placeholders::_1)});
}

ShieldingCommunicationPoint::~ShieldingCommunicationPoint() {
    logger_->debug("DTor");
}

bool ShieldingCommunicationPoint::openRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("openRequestHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return false;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return false;
    }
    sendJSONReply<bool>(packet, manager_->open(priority));
    return false;
}

bool ShieldingCommunicationPoint::closeRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("closeRequestHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return false;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return false;
    }
    sendJSONReply<bool>(packet, manager_->close(priority));
    return false;
}

bool ShieldingCommunicationPoint::resetFaultStateRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("resetFaultStateRequestHandler");
    uint32_t priority = 0;
    try {
        priority = packet.data.at("priority").get<uint32_t>();
        if (priority == 0) {
            sendJSONError(packet, "Priority not valid");
            return false;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return false;
    }
    sendJSONReply<bool>(packet, manager_->resetFaultState(priority));
    return false;
}

}  // namespace crf::actuators::shielding
