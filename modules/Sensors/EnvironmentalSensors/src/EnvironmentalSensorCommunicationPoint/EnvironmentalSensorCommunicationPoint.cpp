/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <map>
#include <string>
#include <functional>
#include <atomic>

#include "EnvironmentalSensors/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorCommunicationPoint.hpp"

namespace crf {
namespace sensors {
namespace environmentalsensors {

EnvironmentalSensorCommunicationPoint::EnvironmentalSensorCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<EnvironmentalSensorManager> sensorManager) :
    StatusStreamerCommunicationPoint(socket, sensorManager),
    logger_("EnvironmentalSensorCommunicationPoint") {
    logger_->debug("CTor");
}

EnvironmentalSensorCommunicationPoint::~EnvironmentalSensorCommunicationPoint() {
}

}  // namespace environmentalsensors
}  // namespace sensors
}  // namespace crf
