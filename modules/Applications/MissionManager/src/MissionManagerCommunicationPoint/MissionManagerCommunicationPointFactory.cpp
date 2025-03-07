 /* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>

#include "MissionManager/MissionManagerCommunicationPoint/MissionManagerCommunicationPointFactory.hpp"
#include "MissionManager/MissionManagerCommunicationPoint/MissionManagerCommunicationPoint.hpp"

namespace crf::applications::missionmanager {

MissionManagerCommunicationPointFactory::MissionManagerCommunicationPointFactory(
    std::shared_ptr<crf::applications::missionmanager::IMissionManager> mission):
        mission_(mission),
        logger_("MissionManagerCommunicationPointFactory") {
            logger_->debug("CTor");
}

MissionManagerCommunicationPointFactory::~MissionManagerCommunicationPointFactory() {
    logger_->debug("DTor");
}

std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
    MissionManagerCommunicationPointFactory::create(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket) {
        logger_->debug("create");
        std::shared_ptr<communication::communicationpointserver::ICommunicationPoint> commpoint =
            std::make_shared<MissionManagerCommunicationPoint>(socket, mission_);
            return commpoint;
}

}  // namespace crf::applications::missionmanager
