/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "MissionManager/IMissionManager.hpp"
#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::applications::missionmanager {

class MissionManagerCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    explicit MissionManagerCommunicationPointFactory(
        std::shared_ptr<crf::applications::missionmanager::IMissionManager>);
    ~MissionManagerCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<crf::applications::missionmanager::IMissionManager> mission_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::applications::missionmanager
