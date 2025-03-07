/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2024
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
#include "IMU/IMUCommunicationPoint/IMUCommunicationPoint.hpp"

namespace crf::sensors::imu {

IMUCommunicationPoint::IMUCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<IMUManager> manager):
    crf::utility::devicemanager::PriorityAccessCommunicationPoint(socket, manager),
    socket_(socket),
    manager_(manager),
    logger_("IMUCommunicationPoint") {
    logger_->debug("CTor");

    jsonCommandHandlers_.insert({
        "calibrate",
        std::bind(&IMUCommunicationPoint::calibrate,
            this, std::placeholders::_1)});
}

IMUCommunicationPoint::~IMUCommunicationPoint() {
    logger_->debug("DTor");
}

void IMUCommunicationPoint::calibrate(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("calibrate");
    int priority = 0;
    try {
        priority = packet.data.at("priority").get<int>();
    } catch (const std::exception& e) {
        sendJSONError(packet, crf::Code::BadRequest);
        return;
    }
    try {
        sendJSONReply<crf::expected<bool>>(packet,
            manager_->calibrate(priority));
    } catch (const std::exception& e) {
        sendJSONError(packet, "Any error code that might fit");
        return;
    }
}

}  // namespace crf::sensors::imu
