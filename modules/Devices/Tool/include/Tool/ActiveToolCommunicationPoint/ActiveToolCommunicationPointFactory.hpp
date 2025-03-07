/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "Tool/ActiveToolCommunicationPoint/ActiveToolManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::devices::tool {

/**
 * @ingroup group_active_tool
 * @brief
 */
class ActiveToolCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    ActiveToolCommunicationPointFactory() = delete;
    explicit ActiveToolCommunicationPointFactory(
        std::shared_ptr<crf::devices::tool::ActiveToolManager> manager);
    ~ActiveToolCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<crf::devices::tool::ActiveToolManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::devices::tool
