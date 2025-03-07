/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>

#include "EnvironmentalSensors/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorCommunicationPointFactory.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

EnvironmentalSensorCommunicationPointFactory::EnvironmentalSensorCommunicationPointFactory(
    std::shared_ptr<crf::sensors::environmentalsensors::EnvironmentalSensorManager> sensorManager) :
    sensorManager_(sensorManager),
    logger_("EnvironmentalSensorCommunicationPointFactory") {
    logger_->debug("CTor");
}

EnvironmentalSensorCommunicationPointFactory::~EnvironmentalSensorCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    EnvironmentalSensorCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<EnvironmentalSensorCommunicationPoint>(socket, sensorManager_);
    return commpoint;
}

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
