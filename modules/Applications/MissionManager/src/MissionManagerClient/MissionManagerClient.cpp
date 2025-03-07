/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <atomic>
#include <optional>
#include <mutex>

#include "MissionManager/MissionManagerClient/MissionManagerClient.hpp"

namespace crf::applications::missionmanager {

MissionManagerClient::MissionManagerClient(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds serverReplyTimeout,
    const float frequency):
    crf::utility::devicemanager::StatusStreamerClient(socket, serverReplyTimeout, frequency),
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    logger_("MissionManagerClient") {
        logger_->debug("CTor");

        receivers_.insert({"start", &receiverStart_});
        receivers_.insert({"next", &receiverNext_});
        receivers_.insert({"stop", &receiverStop_});
        receivers_.insert({"pause", &receiverPause_});
        receivers_.insert({"resume", &receiverResume_});
        receivers_.insert({"goHome", &receiverGoHome_});
        receivers_.insert({"recharge", &receiverRecharge_});
        receivers_.insert({"setStatus", &receiverSetStatus_});
        receivers_.insert({"emergency", &receiverEmergency_});
        receivers_.insert({"rearm", &receiverRearm_});
}

MissionManagerClient::~MissionManagerClient() {
    logger_->debug("DTor");
}

bool MissionManagerClient::start() {
    logger_->debug("start");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "start";
    return sendPacket(json, receiverStart_);
}

bool MissionManagerClient::next() {
    logger_->debug("next");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "next";
    return sendPacket(json, receiverNext_);
}

bool MissionManagerClient::stop() {
    logger_->debug("stop");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "stop";
    return sendPacket(json, receiverStop_);
}

bool MissionManagerClient::pause() {
    logger_->debug("pause");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "pause";
    return sendPacket(json, receiverPause_);
}

bool MissionManagerClient::resume() {
    logger_->debug("resume");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "resume";
    return sendPacket(json, receiverResume_);
}

bool MissionManagerClient::goHome() {
    logger_->debug("goHome");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "goHome";
    return sendPacket(json, receiverGoHome_);
}

bool MissionManagerClient::recharge() {
    logger_->debug("recharge");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "recharge";
    return sendPacket(json, receiverRecharge_);
}

bool MissionManagerClient::setStatus(const nlohmann::json& status) {
    logger_->debug("setStatus");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["message"] = status;
    return sendPacket(json, receiverSetStatus_);
}

bool MissionManagerClient::emergency() {
    logger_->debug("emergency");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "emergency";
    return sendPacket(json, receiverEmergency_);
}

bool MissionManagerClient::rearm() {
    logger_->debug("sendRearm");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "rearm";
    return sendPacket(json, receiverRearm_);
}

nlohmann::json MissionManagerClient::getStatus() {
    logger_->debug("sendGetStatus");
    requestStatus();
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    return statusUpdate_;
}

// Private

void MissionManagerClient::parseStatus(const nlohmann::json& json) {
    std::scoped_lock<std::mutex> statusLock(statusMutex_);
    try {
        statusUpdate_ = json;
    } catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
    }
}

bool MissionManagerClient::sendPacket(const communication::datapackets::JSONPacket& json,
    const Receiver<bool>& receiver) {
    logger_->debug("sendPacket: {}", json.data);
    if (!initialized_) {
        logger_->warn("Not initialized");
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
    return result.value();
}

}  // namespace crf::applications::missionmanager
