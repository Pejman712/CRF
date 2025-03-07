/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "Shielding/ShieldingCommunicationPoint/ShieldingCommunicationPointFactory.hpp"
#include "Shielding/ShieldingCommunicationPoint/ShieldingManager.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "Shielding/ShieldingCommunicationPoint/ShieldingCommunicationPoint.hpp"

namespace crf::actuators::shielding {

ShieldingCommunicationPointFactory::ShieldingCommunicationPointFactory(
    std::shared_ptr<ShieldingManager> manager) :
    manager_(manager),
    logger_("ShieldingCommunicationPointFactory") {
    logger_->debug("CTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    ShieldingCommunicationPointFactory::create(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> point =
        std::make_shared<ShieldingCommunicationPoint>(socket, manager_);
    return point;
}

}  // namespace crf::actuators::shielding
