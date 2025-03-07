/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>

#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPoint.hpp"
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPointFactory.hpp"

namespace crf::control::robotbasecontroller {

RobotBaseControllerCommunicationPointFactory::RobotBaseControllerCommunicationPointFactory(
    std::shared_ptr<RobotBaseControllerManager> manager) :
    manager_(manager),
    logger_("RobotBaseControllerCommunicationPointFactory") {
    logger_->debug("CTor");
}

RobotBaseControllerCommunicationPointFactory::~RobotBaseControllerCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>> RobotBaseControllerCommunicationPointFactory::create( // NOLINT
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<RobotBaseControllerCommunicationPoint>(socket, manager_);
    return commpoint;
}

}  // namespace crf::control::robotbasecontroller
