/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

class RPSensorCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    RPSensorCommunicationPointFactory() = delete;
    explicit RPSensorCommunicationPointFactory(
        std::shared_ptr<crf::sensors::rpsensor::RPSensorManager> sensorManager);
    ~RPSensorCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<crf::sensors::rpsensor::RPSensorManager> sensorManager_;
    utility::logger::EventLogger logger_;
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
