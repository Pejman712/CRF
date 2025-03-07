/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>

#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPoint.hpp"
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPointFactory.hpp"

namespace crf::actuators::linearactuator {

LinearActuatorCommunicationPointFactory::LinearActuatorCommunicationPointFactory(
    std::shared_ptr<LinearActuatorManager> manager) :
    manager_(manager),
    logger_("LinearActuatorCommunicationPointFactory") {
    logger_->debug("CTor");
}

LinearActuatorCommunicationPointFactory::~LinearActuatorCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    LinearActuatorCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared
            <crf::actuators::linearactuator::LinearActuatorCommunicationPoint>(socket, manager_);
    return commpoint;
}

}  // namespace crf::actuators::linearactuator
