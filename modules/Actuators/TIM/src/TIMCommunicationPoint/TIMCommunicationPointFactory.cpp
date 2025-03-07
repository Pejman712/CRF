/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "TIM/TIMCommunicationPoint/TIMCommunicationPointFactory.hpp"
#include "TIM/TIMCommunicationPoint/TIMManager.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "TIM/TIMCommunicationPoint/TIMCommunicationPoint.hpp"

namespace crf::actuators::tim {

TIMCommunicationPointFactory::TIMCommunicationPointFactory(
    std::shared_ptr<TIMManager> manager) :
    manager_(manager),
    logger_("TIMCommunicationPointFactory") {
    logger_->debug("CTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    TIMCommunicationPointFactory::create(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> point =
        std::make_shared<TIMCommunicationPoint>(socket, manager_);
    return point;
}

}  // namespace crf::actuators::tim
