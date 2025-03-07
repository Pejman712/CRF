/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary
 * software license. Any permission to use it shall be granted in writing. Requests shall be
 * addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ===============================================================================================
 */

#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DeviceManager/IDeviceManager.hpp"
#include "DeviceManager/StatusStreamerMode.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

BS::thread_pool StatusStreamerCommunicationPoint::threadPool_ =
    BS::thread_pool(std::thread::hardware_concurrency()/2);

unsigned int StatusStreamerCommunicationPoint::openSocketsCounter = 0;

StatusStreamerCommunicationPoint::StatusStreamerCommunicationPoint(
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<crf::utility::devicemanager::IDeviceManager> manager) :
    logger_("StatusStreamerCommunicationPoint"),
    socket_(socket),
    manager_(manager),
    receiverThread_(),
    frequency_(),
    streamerMode_(StatusStreamerMode::NotDefined),
    statusStreamerThread_(),
    stopHeartbeatThread_(true),
    streamActive_(false),
    stopStream_(true),
    stopThreads_(true),
    initialized_(false) {
    packetHandlers_.insert({
        communication::datapackets::JSON_PACKET_TYPE,
        std::bind(&StatusStreamerCommunicationPoint::parseJSONPacket,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "startHeartbeat",
        std::bind(&StatusStreamerCommunicationPoint::startHeartbeatRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stopHeartbeat",
        std::bind(&StatusStreamerCommunicationPoint::stopHeartbeatRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "heartbeat",
        std::bind(&StatusStreamerCommunicationPoint::heartbeatRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "getStatus",
        std::bind(&StatusStreamerCommunicationPoint::getStatusRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "startStreamStatus",
        std::bind(&StatusStreamerCommunicationPoint::startStreamStatusRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({
        "stopStreamStatus",
        std::bind(&StatusStreamerCommunicationPoint::stopStreamStatusRequestHandler,
            this, std::placeholders::_1)});

    openSocketsCounter++;
}

StatusStreamerCommunicationPoint::~StatusStreamerCommunicationPoint() {
    logger_->debug("DTor");
    deinitialize();
}

void StatusStreamerCommunicationPoint::deinitialiseDeviceManagerOnSocketClose() {
    logger_->info("Sockets closed.");
}

void StatusStreamerCommunicationPoint::heartbeatLost() {
    logger_->critical("Heartbeat has been lost!");
}

bool StatusStreamerCommunicationPoint::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->info("Already initialized");
        return true;
    }
    if (!socket_->isOpen()) {
        if (!socket_->open()) {
            logger_->error("Failed to open the socket");
            return false;
        }
    }
    stopThreads_ = false;
    receiverThread_ = std::thread(&StatusStreamerCommunicationPoint::receiver, this);
    for (const auto& pair : jsonCommandHandlers_) {
        jsonCommandExecuting_.insert({pair.first, false});
    }
    initialized_ = true;
    return true;
}

bool StatusStreamerCommunicationPoint::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->info("Already deinitialized");
        return true;
    }
    openSocketsCounter--;
    logger_->debug("Sockets remaining: {}", openSocketsCounter);
    if (openSocketsCounter == 0) {
        logger_->debug("Last client disconnected");
        deinitialiseDeviceManagerOnSocketClose();
    }
    threadPool_.wait_for_tasks();
    stopHeartbeatThread();
    stopThreads_ = true;
    stopStream_ = true;
    frequencyCV_.notify_one();
    if (socket_->isOpen()) {
        if (!socket_->close()) {
            logger_->error("Failed to close socket");
            return false;
        }
    }
    if (receiverThread_.joinable()) {
        receiverThread_.join();
    }
    stopStream_ = true;
    if (statusStreamerThread_.joinable()) {
        statusStreamerThread_.join();
    }
    initialized_ = false;
    return true;
}

void StatusStreamerCommunicationPoint::sendJSONError(
    const communication::datapackets::JSONPacket& request, const std::string& message,
    bool requiresAck) {
    communication::datapackets::JSONPacket responseJSON;
    responseJSON.data["command"] = "reply";
    responseJSON.data["replyCommand"] = "error";
    responseJSON.data["message"] = message;

    nlohmann::json::const_iterator id = request.data.find("id");
    if (id != request.data.end()) {
        responseJSON.data["id"] = *id;
    }
    logger_->warn(message);
    socket_->write(responseJSON, responseJSON.getHeader(), requiresAck);
}

void StatusStreamerCommunicationPoint::sendJSONError(
    const communication::datapackets::JSONPacket& request, const crf::Code& message,
    bool requiresAck) {
    communication::datapackets::JSONPacket responseJSON;
    responseJSON.data["command"] = "reply";
    responseJSON.data["replyCommand"] = "error";
    responseJSON.data["message"] = message;

    nlohmann::json::const_iterator id = request.data.find("id");
    if (id != request.data.end()) {
        responseJSON.data["id"] = *id;
    }
    logger_->warn(message);
    socket_->write(responseJSON, responseJSON.getHeader(), requiresAck);
}

void StatusStreamerCommunicationPoint::getStatusRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("getStatusRequestHandler");
    sendJSONReply<const nlohmann::json&>(packet, manager_->getStatus());
}

void StatusStreamerCommunicationPoint::startStreamStatusRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startStreamStatusRequestHandler");
    try {
        if (packet.data.contains("mode")) {
            streamerMode_ = packet.data.at("mode").get<StatusStreamerMode>();
            if (streamerMode_ == StatusStreamerMode::NotDefined) {
                sendJSONError(packet, "Mode not valid");
                return;
            }
        } else {
            streamerMode_ = StatusStreamerMode::Frequency;
        }
        frequency_ = packet.data.at("frequency").get<float>();
        if (frequency_ <= 0) {
            sendJSONError(packet, "Frequency not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    if (!streamActive_) {
        stopStream_ = false;
        statusStreamerThread_ =
            std::thread(&StatusStreamerCommunicationPoint::statusStreamer, this);
    }
    return;
}

void StatusStreamerCommunicationPoint::stopStreamStatusRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopStreamStatusRequestHandler");
    if (!streamActive_) {
        sendJSONError(packet, "Stream was not active");
        return;
    }
    stopStream_ = true;
    frequencyCV_.notify_one();
    statusStreamerThread_.join();
    return;
}

void StatusStreamerCommunicationPoint::startHeartbeatRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startHeartBeatRequestHandler");
    try {
        heartbeatPeriod_ = packet.data.at("period").get<float>();
        if (heartbeatPeriod_ <= 0) {
            sendJSONError(packet, "Heartbeat period not valid");
            return;
        }
    } catch (const std::exception&) {
        sendJSONError(packet, "Wrong parameters");
        return;
    }
    std::unique_lock<std::mutex> lck(heartbeatTimeMtx_);
    lastHeartbeat_ = std::chrono::high_resolution_clock::now();
    startHeartbeatThread();
    sendJSONReply<bool>(packet, true);
}

void StatusStreamerCommunicationPoint::stopHeartbeatRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("stopHeartBeatRequestHandler");
    if (stopHeartbeatThread_) {
        sendJSONError(packet, "Heartbeat has not been configured");
        return;
    }
    stopHeartbeatThread();
    sendJSONReply<bool>(packet, true);
}

void StatusStreamerCommunicationPoint::heartbeatRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    if (stopHeartbeatThread_) {
        sendJSONError(packet, "Heartbeat has not been configured");
        return;
    }
    std::unique_lock<std::mutex> lck(heartbeatTimeMtx_);
    lastHeartbeat_ = std::chrono::high_resolution_clock::now();
    sendJSONReply<bool>(packet, true);
}

void StatusStreamerCommunicationPoint::receiver() {
    logger_->debug("receiver()");
    std::string buffer;
    communication::datapackets::PacketHeader header;
    while (!stopThreads_) {
        if (!socket_->isOpen()) {
            stopThreads_ = true;
            stopStream_ = true;
            return;
        }
        auto message = socket_->read();
        if (!message) {
            continue;
        }
        auto header = message.value().second;
        auto search = packetHandlers_.find(header.type());
        if (search == packetHandlers_.end()) {
            const communication::datapackets::JSONPacket emptyPacket;
            sendJSONError(emptyPacket, "Not supported packet type");
            continue;
        }
        search->second(message.value().first);
    }
}

void StatusStreamerCommunicationPoint::statusStreamer() {
    logger_->debug("statusStreamer");
    streamActive_ = true;
    nlohmann::json prevStatus;
    while (!stopStream_) {
        auto start = std::chrono::high_resolution_clock::now();
        if (streamerMode_ == StatusStreamerMode::Frequency) {
            sendStatus(manager_->getStatus());
        } else if (streamerMode_ == StatusStreamerMode::Differential) {
            nlohmann::json status = manager_->getStatus();
            if (prevStatus != status) {
                prevStatus = status;
                sendStatus(status);
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto loopDuration = end - start;
        std::unique_lock<std::mutex> frequencyLck(frequencyMtx_);
        if (loopDuration < std::chrono::milliseconds(static_cast<int>(1000 / frequency_))) {
            frequencyCV_.wait_for(frequencyLck,
                std::chrono::milliseconds(static_cast<int>(1000 / frequency_)) - loopDuration);
        }
    }
    streamActive_ = false;
}

void StatusStreamerCommunicationPoint::sendStatus(const nlohmann::json& status) {
    sendJSONEvent<const nlohmann::json&>("streamStatus", status);
}

void StatusStreamerCommunicationPoint::parseJSONPacket(const std::string& buffer) {
    communication::datapackets::JSONPacket json;
    if (!json.deserialize(buffer)) {
        sendJSONError(json, "Not valid json format");
        return;
    }
    std::string command;

    try {
        command = json.data.at("command").get<std::string>();
    } catch (const std::exception& e) {
        sendJSONError(json, "Cannot get command field from received json");
        return;
    }
    if (jsonCommandHandlers_.find(command) == jsonCommandHandlers_.end()) {
        sendJSONError(json, "Unknown command: " + command);
        return;
    }
    if (json.data.contains("user")) {
        if (json.data.at("user").contains("name") && json.data.at("user").contains("role")) {
            logger_.userName = json.data.at("user").at("name").get<std::string>();;
            logger_.userRole = json.data.at("user").at("role").get<std::string>();;
            logger_.setFormat(crf::utility::logger::Format::UserDetails);
        } else {
            sendJSONError(json, "Unknown user fields: " + command);
            return;
        }
    } else {
        logger_.setFormat(crf::utility::logger::Format::Default);
    }

    /**
     * Wrapper over the functions inserted in the map to keep track of which functions
     * are being executed. This way we disregard repeated commands and only execute them
     * once.
     * (jplayang)
     */
    logger_->debug("command: {}", command);
    auto handlerWrapper = [command, json, this]() {
        jsonCommandExecuting_[command] = true;
        jsonCommandHandlers_[command](json);
        jsonCommandExecuting_[command] = false;
    };
    if (!jsonCommandExecuting_[command]) threadPool_.submit(handlerWrapper);
    return;
}

void StatusStreamerCommunicationPoint::startHeartbeatThread() {
    stopHeartbeatThread();
    stopHeartbeatThread_ = false;
    heartbeatThread_ = std::thread(
        &StatusStreamerCommunicationPoint::heartbeatThread, this);
}

void StatusStreamerCommunicationPoint::stopHeartbeatThread() {
    if (!heartbeatThread_.joinable()) return;
    stopHeartbeatThread_ = true;
    heartbeatCV_.notify_one();
    heartbeatThread_.join();
}

void StatusStreamerCommunicationPoint::heartbeatThread() {
    while (!stopHeartbeatThread_) {
        auto now = std::chrono::high_resolution_clock::now();
        std::unique_lock<std::mutex> lck(heartbeatTimeMtx_);
        auto timeSinceLastHeartbeat = now - lastHeartbeat_;
        lck.unlock();

        if (timeSinceLastHeartbeat > std::chrono::milliseconds(
            static_cast<int>(heartbeatPeriod_ * 1000))) {
            heartbeatLost();
            return;
        }

        std::unique_lock<std::mutex> heartbeatLck(heartbeatMtx_);
        // Sleep for heartbeat period divided by 10 (hb*1000/10)
        heartbeatCV_.wait_for(heartbeatLck, std::chrono::milliseconds(
            static_cast<int>(heartbeatPeriod_ * 100)));
    }
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
