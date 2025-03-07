/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerCommunicationPointFactory.hpp"
#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerManager.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerCommunicationPoint.hpp"

namespace crf::actuators::mechanicalstabilizer {

MechanicalStabilizerCommunicationPointFactory::MechanicalStabilizerCommunicationPointFactory(
    std::shared_ptr<MechanicalStabilizerManager> manager) :
    manager_(manager),
    logger_("MechanicalStabilizerCommunicationPointFactory") {
    logger_->debug("CTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    MechanicalStabilizerCommunicationPointFactory::create(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
    std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> point =
        std::make_shared<MechanicalStabilizerCommunicationPoint>(socket, manager_);
    return point;
}

}  // namespace crf::actuators::mechanicalstabilizer
