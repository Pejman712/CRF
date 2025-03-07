/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Alejandro Diaz Rosales BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <optional>
#include <map>
#include <tuple>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "CommunicationUtility/STLJSONConverter.hpp"
#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "RPSensor/IRPSensor.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

class RPSensorClient:
    public crf::utility::devicemanager::PriorityAccessClient,
    public IRPSensor {
 public:
    explicit RPSensorClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency,
        const uint32_t priority);
    RPSensorClient() = delete;
    RPSensorClient(const RPSensorClient&) = delete;
    RPSensorClient(RPSensorClient&&) = delete;
    ~RPSensorClient() override;

    bool initialize() override;
    bool deinitialize() override;

    std::optional<float> getDoseRate() override;
    std::optional<float> getCumulativeDose() override;
    bool resetCumulativeDose() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    const uint32_t priority_;

    Receiver<bool> receiverResetCumulativeDose_;

    std::atomic<std::optional<float>> statusDoseRate_;
    std::atomic<std::optional<float>> statusCumulativeDose_;

    crf::utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;
    bool sendPacket(
        const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
