/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary
 * software license. Any permission to use it shall be granted in writing. Requests shall be
 * addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "CommunicationUtility/ExpectedJSONConverter.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DeviceManager/IDeviceManager.hpp"
#include "DeviceManager/StatusStreamerMode.hpp"
#include "EventLogger/EventLogger.hpp"
#include "ThreadPool/BS_thread_pool.hpp"
#include "crf/ResponseDefinitions.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

/**
 * @ingroup status_streamer_communication_point
 * @brief Communication Point that can send once or stream given a desired frequency, the status of
 *        a device.
 */
class StatusStreamerCommunicationPoint
    : public communication::communicationpointserver::ICommunicationPoint {
 public:
    StatusStreamerCommunicationPoint() = delete;
    StatusStreamerCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::utility::devicemanager::IDeviceManager> manager);
    ~StatusStreamerCommunicationPoint() override;

    bool initialize() override;
    bool deinitialize() override;

 protected:
    std::map<uint16_t, std::function<void(const std::string&)>> packetHandlers_;
    std::map<std::string, std::function<void(const communication::datapackets::JSONPacket&)>>
        jsonCommandHandlers_;
    std::map<std::string, bool> jsonCommandExecuting_;

    /**
     * @brief reply to a JSon request
     *
     * @param[in] request the request the reply is associated with
     * @param[in] message "message" to send
     * @param requiresAck
     */
    template <typename T>
    void sendJSONReply(const communication::datapackets::JSONPacket& request, T message,
                       bool requiresAck = true);

    /**
     * @brief send a JSon event
     * @details sent message is not bound to any request
     * to be used only in very specific cases
     *
     * @param[in] replyCommand "replyCommand" field for the message
     * @param[in] message "message" to send
     * @param requiresAck
     */
    template <typename T>
    void sendJSONEvent(const std::string& replyCommand, T message, bool requiresAck = true);

    /**
     * @brief send an error message
     * @details use an empty request to send un-requested errors (asynchronuous)
     *
     * @param[in] request the request the message is associated with
     * @param[in] message "message" to send
     * @param requiresAck
     */
    void sendJSONError(const communication::datapackets::JSONPacket& request,
                       const std::string& message, bool requiresAck = true);

    /**
     * @brief send an error code
     * @details use an empty request to send un-requested errors (asynchronuous)
     *
     * @param[in] request the request the message is associated with
     * @param[in] message the error code to send
     * @param requiresAck
     */
    void sendJSONError(const communication::datapackets::JSONPacket& request,
                       const crf::Code& message, bool requiresAck = true);

    /**
     * @brief closes socket
     * @details this function can be overwritten in the class you use it in to change the behavior
     */
    virtual void deinitialiseDeviceManagerOnSocketClose();

    /**
     * @brief closes socket
     * @details this function can be overwritten in the class you use it in to change the behavior
     */
    virtual void heartbeatLost();


 private:
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::utility::devicemanager::IDeviceManager> manager_;
    std::thread receiverThread_;
    float frequency_;
    StatusStreamerMode streamerMode_;
    std::thread statusStreamerThread_;

    /**
     * @brief Static so that when several communication points are executed simultaneously,
     * several thread pools are not created with an N number of threads each
     * (jplayang)
     */
    static BS::thread_pool threadPool_;

    std::mutex frequencyMtx_;
    std::condition_variable frequencyCV_;

    std::atomic<bool> stopHeartbeatThread_;
    std::atomic<float> heartbeatPeriod_;
    std::mutex heartbeatTimeMtx_;
    std::chrono::high_resolution_clock::time_point lastHeartbeat_;
    std::thread heartbeatThread_;
    std::mutex heartbeatMtx_;
    std::condition_variable heartbeatCV_;


    std::atomic<bool> streamActive_;
    std::atomic<bool> stopStream_;
    std::atomic<bool> stopThreads_;
    bool initialized_;

    void getStatusRequestHandler(const communication::datapackets::JSONPacket& packet);
    void startStreamStatusRequestHandler(const communication::datapackets::JSONPacket& packet);
    void stopStreamStatusRequestHandler(const communication::datapackets::JSONPacket& packet);

    void startHeartbeatRequestHandler(const communication::datapackets::JSONPacket& packet);
    void stopHeartbeatRequestHandler(const communication::datapackets::JSONPacket& packet);
    void heartbeatRequestHandler(const communication::datapackets::JSONPacket& packet);

    void receiver();
    void statusStreamer();
    void sendStatus(const nlohmann::json& status);

    void parseJSONPacket(const std::string& buffer);

    void startHeartbeatThread();
    void stopHeartbeatThread();
    void heartbeatThread();

    static unsigned int openSocketsCounter;
};

template <typename T>
void StatusStreamerCommunicationPoint::sendJSONReply(
    const communication::datapackets::JSONPacket& request, T message, bool requiresAck) {
    communication::datapackets::JSONPacket responseJSON;
    responseJSON.data["command"] = "reply";
    responseJSON.data["replyCommand"] = request.data.value("command", "");
    responseJSON.data["message"] = message;

    nlohmann::json::const_iterator id = request.data.find("id");
    if (id != request.data.end()) {
        responseJSON.data["id"] = *id;
    }
    socket_->write(responseJSON, responseJSON.getHeader(), requiresAck);
}

template <typename T>
void StatusStreamerCommunicationPoint::sendJSONEvent(const std::string& replyCommand, T message,
                                                     bool requiresAck) {
    communication::datapackets::JSONPacket responseJSON;
    responseJSON.data["command"] = "reply";
    responseJSON.data["replyCommand"] = replyCommand;
    responseJSON.data["message"] = message;
    socket_->write(responseJSON, responseJSON.getHeader(), requiresAck);
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
