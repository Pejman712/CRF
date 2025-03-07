/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include "RPSensor/RPSensorClient/RPSensorClient.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

RPSensorClient::RPSensorClient(
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
    priority_(priority),
    logger_("RPSensorClient") {
    logger_->debug("CTor");

    receivers_.insert({"resetCumulativeDose", &receiverResetCumulativeDose_});
}

RPSensorClient::~RPSensorClient() {
    logger_->debug("DTor");
}

bool RPSensorClient::initialize() {
    return PriorityAccessClient::initialize();
}
bool RPSensorClient::deinitialize() {
    return PriorityAccessClient::deinitialize();
}

std::optional<float> RPSensorClient::getDoseRate() {
    logger_->debug("getDoseRate");
    if (!requestStatus()) return std::nullopt;
    return statusDoseRate_.load();
}

std::optional<float> RPSensorClient::getCumulativeDose() {
    logger_->debug("getCumulativeDose");
    if (!requestStatus()) return std::nullopt;
    return statusCumulativeDose_.load();
}

bool RPSensorClient::resetCumulativeDose() {
    logger_->debug("resetCumulativeDose");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "resetCumulativeDose";
    json.data["priority"] = priority_;
    return sendPacket(json, receiverResetCumulativeDose_);
}

// Private

void RPSensorClient::parseStatus(const nlohmann::json& json) {
    try {
        statusDoseRate_ = json["doseRate"].get<std::optional<float>>();
        statusCumulativeDose_ = json["cumulativeDose"].get<std::optional<float>>();
    } catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
    }
}

bool RPSensorClient::sendPacket(const communication::datapackets::JSONPacket& json,
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

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
