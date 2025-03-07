/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <utility>
#include <nlohmann/json.hpp>

#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPoint.hpp"

namespace crf::actuators::linearactuator {

LinearActuatorCommunicationPoint::LinearActuatorCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<LinearActuatorManager> manager):
    crf::utility::devicemanager::PriorityAccessCommunicationPoint(socket, manager),
    socket_(socket),
    manager_(manager),
    logger_("LinearActuatorCommunicationPoint") {
    logger_->debug("CTor");

    jsonCommandHandlers_.insert({
        "setPosition",
        std::bind(&LinearActuatorCommunicationPoint::setPositionHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "setVelocity",
        std::bind(&LinearActuatorCommunicationPoint::setVelocityHandler,
            this, std::placeholders::_1)});
}

LinearActuatorCommunicationPoint::~LinearActuatorCommunicationPoint() {
    logger_->debug("DTor");
}

void LinearActuatorCommunicationPoint::setPositionHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setPositionHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        double position = packet.data.at("position").get<double>();
        sendJSONReply<crf::expected<bool>>(packet,
            manager_->setPosition(priority, position));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void LinearActuatorCommunicationPoint::setVelocityHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("setVelocityHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        double velocity = packet.data.at("velocity").get<double>();
        sendJSONReply<crf::expected<bool>>(packet,
            manager_->setVelocity(priority, velocity));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

}  // namespace crf::actuators::linearactuator
