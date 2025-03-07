/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>

#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPoint.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.hpp"

namespace crf::control::robotarmcontroller {

RobotArmControllerCommunicationPointFactory::RobotArmControllerCommunicationPointFactory(
    std::shared_ptr<RobotArmControllerManager> manager) :
    manager_(manager),
    logger_("RobotArmControllerCommunicationPointFactory") {
    logger_->debug("CTor");
}

RobotArmControllerCommunicationPointFactory::~RobotArmControllerCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    RobotArmControllerCommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<RobotArmControllerCommunicationPoint>(socket, manager_);
    return commpoint;
}

}  // namespace crf::control::robotarmcontroller
