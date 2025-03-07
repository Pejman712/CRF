/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPointFactory.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonManager.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPoint.hpp"

namespace crf::actuators::timrpwagon {

TIMRPWagonCommunicationPointFactory::TIMRPWagonCommunicationPointFactory(
    std::shared_ptr<TIMRPWagonManager> manager) :
    manager_(manager),
    logger_("TIMRPWagonCommunicationPointFactory") {
    logger_->debug("CTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    TIMRPWagonCommunicationPointFactory::create(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> point =
        std::make_shared<TIMRPWagonCommunicationPoint>(socket, manager_);
    return point;
}

}  // namespace crf::actuators::timrpwagon
