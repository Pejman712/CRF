/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>

#include "RPSensor/RPSensorCommunicationPoint/RPSensorManager.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorCommunicationPoint.hpp"
#include "RPSensor/RPSensorCommunicationPoint/RPSensorCommunicationPointFactory.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

RPSensorCommunicationPointFactory::RPSensorCommunicationPointFactory(
    std::shared_ptr<crf::sensors::rpsensor::RPSensorManager> sensorManager) :
    sensorManager_(sensorManager),
    logger_("RPSensorCommunicationPointFactory") {
    logger_->debug("CTor");
}

RPSensorCommunicationPointFactory::~RPSensorCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    RPSensorCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<RPSensorCommunicationPoint>(socket, sensorManager_);
    return commpoint;
}

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
