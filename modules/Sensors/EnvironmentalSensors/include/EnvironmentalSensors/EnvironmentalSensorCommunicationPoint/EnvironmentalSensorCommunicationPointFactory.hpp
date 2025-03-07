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
#include "EnvironmentalSensors/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorManager.hpp"
#include "EnvironmentalSensors/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorCommunicationPoint.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

class EnvironmentalSensorCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    EnvironmentalSensorCommunicationPointFactory() = delete;
    explicit EnvironmentalSensorCommunicationPointFactory(
        std::shared_ptr<EnvironmentalSensorManager> sensorManager);
    ~EnvironmentalSensorCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<EnvironmentalSensorManager> sensorManager_;
    utility::logger::EventLogger logger_;
};

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
