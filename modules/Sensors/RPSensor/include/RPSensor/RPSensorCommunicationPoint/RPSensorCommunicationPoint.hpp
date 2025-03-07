/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <chrono>

#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorManager.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

class RPSensorCommunicationPoint :
    public crf::utility::devicemanager::PriorityAccessCommunicationPoint {
 public:
    RPSensorCommunicationPoint() = delete;
    RPSensorCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::sensors::rpsensor::RPSensorManager> sensorManager);
    ~RPSensorCommunicationPoint() override;

 private:
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::sensors::rpsensor::RPSensorManager> sensorManager_;
    crf::utility::logger::EventLogger logger_;

    void resetCumulativeDoseHandler(const communication::datapackets::JSONPacket& packet);
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
