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

#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DeviceManager/DeviceManagerWithPriorityAccess/DeviceManagerWithPriorityAccess.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

PriorityAccessCommunicationPoint::PriorityAccessCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::utility::devicemanager::DeviceManagerWithPriorityAccess> manager) :
    StatusStreamerCommunicationPoint(socket, manager),
    logger_("PriorityAccessCommunicationPoint"),
    manager_(manager) {
    jsonCommandHandlers_.insert({
        "lockControl",
        std::bind(&PriorityAccessCommunicationPoint::lockControlRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "unlockControl",
        std::bind(&PriorityAccessCommunicationPoint::unlockControlRequestHandler,
            this, std::placeholders::_1)});
}

PriorityAccessCommunicationPoint::~PriorityAccessCommunicationPoint() {
    StatusStreamerCommunicationPoint::deinitialize();
}

void PriorityAccessCommunicationPoint::lockControlRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("lockControlRequestHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority <= 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    sendJSONReply<bool>(packet, manager_->lockControl(priority));
    return;
}

void PriorityAccessCommunicationPoint::unlockControlRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("unlockControlRequestHandler");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
        if (priority <= 0) {
            sendJSONError(packet, "Priority not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    sendJSONReply<bool>(packet, manager_->unlockControl(priority));
    return;
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
