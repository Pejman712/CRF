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
#include "Tool/ActiveToolCommunicationPoint/ActiveToolCommunicationPoint.hpp"

namespace crf::devices::tool {

ActiveToolCommunicationPoint::ActiveToolCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::devices::tool::ActiveToolManager> manager):
    crf::utility::devicemanager::PriorityAccessCommunicationPoint(
        socket,
        manager),
    socket_(socket),
    manager_(manager),
    logger_("ActiveToolCommunicationPoint") {
    logger_->debug("CTor");
    jsonCommandHandlers_.insert({
        "activate",
        std::bind(&ActiveToolCommunicationPoint::activateHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "deactivate",
        std::bind(&ActiveToolCommunicationPoint::deactivateHandler,
            this, std::placeholders::_1)});
}

ActiveToolCommunicationPoint::~ActiveToolCommunicationPoint() {
    logger_->debug("DTor");
}

void ActiveToolCommunicationPoint::activateHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("activateHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        sendJSONReply<crf::expected<bool>>(packet,
            manager_->activate(priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

void ActiveToolCommunicationPoint::deactivateHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("deactivateHandler");
    try {
        int priority = packet.data.at("priority").get<int>();
        sendJSONReply<crf::expected<bool>>(packet,
            manager_->deactivate(priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    return;
}

}  // namespace crf::devices::tool
