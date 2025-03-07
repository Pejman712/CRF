/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "EnvironmentalSensors/EnvironmentalSensorClient/EnvironmentalSensorClient.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

EnvironmentalSensorClient::EnvironmentalSensorClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& serverReplyTimeout,
    const float& frequency) :
    crf::utility::devicemanager::StatusStreamerClient(
        socket,
        serverReplyTimeout,
        frequency),
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    logger_("EnvironmentalSensorClient") {
    logger_->info("CTor");
}

EnvironmentalSensorClient::~EnvironmentalSensorClient() {
    logger_->info("DTor");
}

bool EnvironmentalSensorClient::initialize() {
    return StatusStreamerClient::initialize();
}

bool EnvironmentalSensorClient::deinitialize() {
    return StatusStreamerClient::deinitialize();
}

std::optional<float> EnvironmentalSensorClient::getMeasurement() {
    logger_->debug("getMeasurement");
    if (!requestStatus()) return std::nullopt;
    return statusMeasurement_;
}

void EnvironmentalSensorClient::parseStatus(const nlohmann::json& json) {
    try {
        statusMeasurement_ = json["rpArmPosition"].get<bool>();
    } catch (const std::exception& e) {
        logger_->error("Couldn't update status with message {}", json);
    }
}

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
