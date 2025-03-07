/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <chrono>

#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EnvironmentalSensors/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorManager.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

class EnvironmentalSensorCommunicationPoint :
    public crf::utility::devicemanager::StatusStreamerCommunicationPoint {
 public:
    EnvironmentalSensorCommunicationPoint() = delete;
    EnvironmentalSensorCommunicationPoint(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<EnvironmentalSensorManager> sensorManager);
    ~EnvironmentalSensorCommunicationPoint() override;

 private:
    crf::utility::logger::EventLogger logger_;
};

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
