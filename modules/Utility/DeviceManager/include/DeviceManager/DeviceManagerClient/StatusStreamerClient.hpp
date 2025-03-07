/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <map>
#include <condition_variable>
#include <mutex>
#include <optional>

#include <nlohmann/json.hpp>

#include <CommonInterfaces/IInitializable.hpp>
#include "DataPacketSocket/PacketSocket.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

/**
 * @ingroup group_status_streamer_client
 * @brief Client that can communicate with a Device Manager Status Streamer
 *        communication point.
 */
class StatusStreamerClient {
 public:
    StatusStreamerClient() = delete;
    StatusStreamerClient(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds& serverReplyTimeout,
        const float& frequency);
    virtual ~StatusStreamerClient();

    bool initialize();
    bool deinitialize();

 protected:
    std::map<uint16_t, std::function<void(const std::string&)>> packetHandlers_;

 private:
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    const float frequency_;

    std::thread grabberThread_;
    std::atomic<bool> stopThread_;

    crf::utility::logger::EventLogger logger_;

    void grabber();
    void parseJSONPacket(const std::string& buffer);
    bool startStream();
    bool stopStream();

 protected:
    bool initialized_;
    bool streamActive_;
    std::mutex socketMutex_;
    std::mutex statusMutex_;

    virtual void parseStatus(const nlohmann::json& json);
    bool requestStatus();

    /**
     * @brief If we want to create an object that contains different types
     *        of template classes (ej: Receiver<bool>, Receiver<float>, ...)
     *        then we need to create an interface or base to properly manage it
     *        (jplayang)
     */

    class IReceiver {
     public:
        virtual ~IReceiver() = default;

        virtual void updateAndNotify(const communication::datapackets::JSONPacket& json) const = 0;
    };

    /**
     * @brief This Receiver class creates an object base approach to communication
     *        between client and server. This allows us to send a message and then
     *        create a receiver to wait for an answer that could be bool, float,
     *        string, etc...
     *        It is linked to the grabber so that when the user waits, once finished
     *        it will respond with the answer directly (or nullptr if nothing was
     *        received)
     *        (jplayang)
     */
    template <typename T>
    class Receiver: public IReceiver {
     private:
        /*
         * These are mutable so that we can actually declare methods as
         * const and call them from const T&.
         * (jplayang)
         */
        mutable std::condition_variable responseCV_;
        mutable std::mutex responseMtx_;
        mutable T result_;
        mutable std::atomic<bool> waiting_ = false;

     public:
        Receiver() = default;
        ~Receiver() override {}

        std::optional<T> waitFor(const std::chrono::milliseconds& serverReplyTimeout) const {
            std::unique_lock<std::mutex> lck(responseMtx_);
            waiting_ = true;
            if (responseCV_.wait_for(lck, serverReplyTimeout) ==
                std::cv_status::timeout) {
                    crf::utility::logger::EventLogger logger("Receiver");
                    logger->error("Timeout for server reply");
                    waiting_ = false;
                    return std::nullopt;
            }
            waiting_ = false;
            return result_;
        }

        T wait() const {
            std::unique_lock<std::mutex> lck(responseMtx_);
            waiting_ = true;
            responseCV_.wait(lck);
            waiting_ = false;
            return result_;
        }

        void updateAndNotify(const communication::datapackets::JSONPacket& json) const override {
            result_ = json.data["message"].get<T>();
            while (!waiting_) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            responseCV_.notify_one();
        }
    };

    /*
     * The map needs to be specified as a map of pointers. Pure virtual classes
     * cannot be placed into a map directly. If placed as non-pure virtual it
     * will produce object slicing. This will call the functions of the base
     * instead of the derivated class.
     * Also, they are placed as raw pointers as we only need to store the address.
     * This makes it more clear in the implementation of each client (and prettier).
     * (jplayang)
     */
    std::map<std::string, IReceiver*> receivers_;
    Receiver<nlohmann::json> receiverStatus_;
};

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
