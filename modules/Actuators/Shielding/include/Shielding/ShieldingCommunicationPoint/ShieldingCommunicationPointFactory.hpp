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
#include "Shielding/ShieldingCommunicationPoint/ShieldingManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::shielding {

class ShieldingCommunicationPointFactory : public communication::communicationpointserver::ICommunicationPointFactory {  // NOLINT
 public:
    ShieldingCommunicationPointFactory() = delete;
    explicit ShieldingCommunicationPointFactory(
        std::shared_ptr<ShieldingManager> manager);
    ShieldingCommunicationPointFactory(
        const ShieldingCommunicationPointFactory&) = default;
    ShieldingCommunicationPointFactory(
        ShieldingCommunicationPointFactory&&) = default;
    ~ShieldingCommunicationPointFactory() override = default;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<ShieldingManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::shielding
