/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>

#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPoint.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerCommunicationPointFactory.hpp"

namespace crf::control::motioncontroller {

MotionControllerCommunicationPointFactory::MotionControllerCommunicationPointFactory(
    std::shared_ptr<MotionControllerManager> manager) :
    manager_(manager),
    logger_("MotionControllerCommunicationPointFactory") {
    logger_->debug("CTor");
}

MotionControllerCommunicationPointFactory::~MotionControllerCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    MotionControllerCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<MotionControllerCommunicationPoint>(socket, manager_);
    return commpoint;
}

}  // namespace crf::control::motioncontroller
