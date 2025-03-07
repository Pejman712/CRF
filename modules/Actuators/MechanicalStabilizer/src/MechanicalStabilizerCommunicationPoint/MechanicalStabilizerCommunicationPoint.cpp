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

#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerManager.hpp"

namespace crf::actuators::mechanicalstabilizer {

MechanicalStabilizerCommunicationPoint::MechanicalStabilizerCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::actuators::mechanicalstabilizer::MechanicalStabilizerManager> manager) :
    crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        socket,
        manager),
    socket_(socket),
    manager_(manager),
    logger_("PriorityAccessCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "activate",
        std::bind(&MechanicalStabilizerCommunicationPoint::activateRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "deactivate",
        std::bind(&MechanicalStabilizerCommunicationPoint::deactivateRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "resetFaultState",
        std::bind(&MechanicalStabilizerCommunicationPoint::resetFaultStateRequestHandler,
            this, std::placeholders::_1)});
}

MechanicalStabilizerCommunicationPoint::~MechanicalStabilizerCommunicationPoint() {
    logger_->debug("DTor");
}

bool MechanicalStabilizerCommunicationPoint::activateRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("activateRequestHandler");
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
    sendJSONReply<bool>(packet, manager_->activate(priority));
    return false;
}

bool MechanicalStabilizerCommunicationPoint::deactivateRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("deactivateRequestHandler");
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
    sendJSONReply<bool>(packet, manager_->deactivate(priority));
    return false;
}

bool MechanicalStabilizerCommunicationPoint::resetFaultStateRequestHandler(
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

}  // namespace crf::actuators::mechanicalstabilizer
