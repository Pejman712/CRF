/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <optional>
#include <memory>
#include <atomic>
#include <chrono>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "EnvironmentalSensors/IEnvironmentalSensor.hpp"
#include "DeviceManager/DeviceManagerClient/StatusStreamerClient.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

class EnvironmentalSensorClient :
    public crf::utility::devicemanager::StatusStreamerClient,
    public crf::sensors::environmentalsensors::IEnvironmentalSensor {
 public:
    explicit EnvironmentalSensorClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds& serverReplyTimeout,
        const float& frequency);
    ~EnvironmentalSensorClient() override;

    bool initialize() override;
    bool deinitialize() override;

    std::optional<float> getMeasurement() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::chrono::milliseconds serverReplyTimeout_;
    float frequency_;
    utility::logger::EventLogger logger_;

    std::atomic<float> statusMeasurement_;

    bool getStatus();
    void parseStatus(const nlohmann::json& json) override;
};

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
