/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "TIM/TIMCommunicationPoint/TIMManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::tim {

class TIMCommunicationPointFactory :
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    TIMCommunicationPointFactory() = delete;
    explicit TIMCommunicationPointFactory(
        std::shared_ptr<TIMManager> manager);
    TIMCommunicationPointFactory(
        const TIMCommunicationPointFactory&) = default;
    TIMCommunicationPointFactory(
        TIMCommunicationPointFactory&&) = default;
    ~TIMCommunicationPointFactory() override = default;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<crf::actuators::tim::TIMManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::tim
