/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <string>
#include <functional>
#include <map>

#include "DeviceManager/DeviceManagerClient/StatusStreamerClient.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

StatusStreamerClient::StatusStreamerClient(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& serverReplyTimeout,
    const float& frequency):
    socket_(socket),
    serverReplyTimeout_(serverReplyTimeout),
    frequency_(frequency),
    logger_("StatusStreamerClient"),
    initialized_(false),
    streamActive_(false) {
        logger_->debug("CTor");

        packetHandlers_.insert({
            communication::datapackets::JSON_PACKET_TYPE,
            std::bind(&StatusStreamerClient::parseJSONPacket,
                this, std::placeholders::_1)
        });

        receivers_.insert({"getStatus", &receiverStatus_});
}

StatusStreamerClient::~StatusStreamerClient() {
    logger_->debug("DTor");
    if (initialized_) deinitialize();
}

bool StatusStreamerClient::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!socket_->open()) {
        logger_->warn("Failed to connect");
        return false;
    }
    stopThread_ = false;
    grabberThread_ = std::thread(&StatusStreamerClient::grabber, this);
    if (frequency_ > 0) startStream();
    initialized_ = true;
    return true;
}

bool StatusStreamerClient::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Already deinitialized");
        return false;
    }
    if (streamActive_) stopStream();

    if (socket_->isOpen()) {
        if (!socket_->close()) {
            logger_->error("Failed to close socket");
        }
    }
    stopThread_ = true;
    if (grabberThread_.joinable()) {
        grabberThread_.join();
    }
    initialized_ = false;
    return true;
}

// Private

void StatusStreamerClient::grabber() {
    while (!stopThread_) {
        if (!socket_->isOpen()) {
            stopThread_ = true;
            continue;
        }
        auto retval = socket_->read();
        if (!retval) {
            continue;
        }
        auto header = retval.value().second;
        auto search = packetHandlers_.find(header.type());
        if (search == packetHandlers_.end()) {
            logger_->error("Not supported packet type: {}", header.type());
            continue;
        }
        search->second(retval.value().first);
    }
}

void StatusStreamerClient::parseJSONPacket(const std::string& buffer) {
    communication::datapackets::JSONPacket json;
    if (!json.deserialize(buffer)) {
        logger_->error("Not valid JSON format");
        return;
    }
    if (!json.data.contains("command")) {
        logger_->error("Field command not found: {}", json.data);
        return;
    }
    std::string command = json.data["command"].get<std::string>();
    if (command != "reply" || !json.data.contains("replyCommand")) {
        logger_->error("Reply command not found, message: {}", json.data);
        return;
    }
    if (json.data["replyCommand"] == "error") {
        logger_->error("Error in server, message: {}", json.data["message"]);
        return;
    }
    if (json.data["replyCommand"] == "streamStatus") {
        parseStatus(json.data["message"]);
        return;
    }
    if (receivers_.find(json.data["replyCommand"]) == receivers_.end()) {
        logger_->error("Not receiver set for {}", json.data["replyCommand"]);
        return;
    }
    receivers_[json.data["replyCommand"]]->updateAndNotify(json);
}

bool StatusStreamerClient::startStream() {
    logger_->debug("startStream");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "startStreamStatus";
    json.data["frequency"] = frequency_;
    streamActive_ = true;
    std::scoped_lock<std::mutex> socketLock(socketMutex_);
    return socket_->write(json, json.getHeader(), false);
}

bool StatusStreamerClient::stopStream() {
    logger_->debug("stopStream");
    communication::datapackets::JSONPacket json;
    json.data["command"] = "stopStreamStatus";
    streamActive_ = false;
    std::scoped_lock<std::mutex> socketLock(socketMutex_);
    return socket_->write(json, json.getHeader(), false);
}

void StatusStreamerClient::parseStatus(const nlohmann::json& json) {
    logger_->warn("Not overwritten");
    return;
}

bool StatusStreamerClient::requestStatus() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    if (streamActive_) return true;
    communication::datapackets::JSONPacket json;
    json.data["command"] = "getStatus";
    std::unique_lock<std::mutex> socketLock(socketMutex_);
    socket_->write(json, json.getHeader());
    socketLock.unlock();

    std::optional<nlohmann::json> result = receiverStatus_.waitFor(serverReplyTimeout_);
    if (!result) {
        logger_->error("Could not receive status answer for {}", json.data);
        return false;
    }
    parseStatus(result.value());
    return true;
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
