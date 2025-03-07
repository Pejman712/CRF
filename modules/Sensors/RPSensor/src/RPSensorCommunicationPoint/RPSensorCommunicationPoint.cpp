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

#include "CommunicationUtility/STLJSONConverter.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

RPSensorCommunicationPoint::RPSensorCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::sensors::rpsensor::RPSensorManager> sensorManager) :
    PriorityAccessCommunicationPoint(socket, sensorManager),
    socket_(socket),
    sensorManager_(sensorManager),
    logger_("RPSensorCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "resetCumulativeDose",
        std::bind(&RPSensorCommunicationPoint::resetCumulativeDoseHandler,
            this, std::placeholders::_1)});
}

RPSensorCommunicationPoint::~RPSensorCommunicationPoint() {
}

void RPSensorCommunicationPoint::resetCumulativeDoseHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("resetCumulativeDoseHandler");
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
    // Can't use "sendJSONPacket" since spdlog doesn't allow logging std::optional
    communication::datapackets::JSONPacket responseJSON;
    responseJSON.data["command"] = "reply";
    responseJSON.data["replyCommand"] = "resetCumulativeDose";
    responseJSON.data["message"] = sensorManager_->resetCumulativeDose(priority);
    socket_->write(responseJSON, responseJSON.getHeader(), true);
    return;
}

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
