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
#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::mechanicalstabilizer {

class MechanicalStabilizerCommunicationPointFactory : public communication::communicationpointserver::ICommunicationPointFactory {  // NOLINT
 public:
    MechanicalStabilizerCommunicationPointFactory() = delete;
    explicit MechanicalStabilizerCommunicationPointFactory(
        std::shared_ptr<MechanicalStabilizerManager> manager);
    MechanicalStabilizerCommunicationPointFactory(
        const MechanicalStabilizerCommunicationPointFactory&) = default;
    MechanicalStabilizerCommunicationPointFactory(
        MechanicalStabilizerCommunicationPointFactory&&) = default;
    ~MechanicalStabilizerCommunicationPointFactory() override = default;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<MechanicalStabilizerManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::mechanicalstabilizer
