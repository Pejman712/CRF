/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>

#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402CommunicationPoint.hpp"
#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402CommunicationPointFactory.hpp"

namespace crf::devices::canopendrivers {

CiA402CommunicationPointFactory::CiA402CommunicationPointFactory(
    std::shared_ptr<CiA402Manager> manager) :
    manager_(manager),
    logger_("CiA402CommunicationPointFactory") {
    logger_->debug("CTor");
}

CiA402CommunicationPointFactory::~CiA402CommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    CiA402CommunicationPointFactory::create(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
        std::make_shared<CiA402CommunicationPoint>(socket, manager_);
    return commpoint;
}

}   // namespace crf::devices::canopendrivers
