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
#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::linearactuator {

/**
 * @ingroup group_linear_actuator_communication_point
 * @brief
 */
class LinearActuatorCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    LinearActuatorCommunicationPointFactory() = delete;
    explicit LinearActuatorCommunicationPointFactory(
        std::shared_ptr<crf::actuators::linearactuator::LinearActuatorManager> manager);
    ~LinearActuatorCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<crf::actuators::linearactuator::LinearActuatorManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::linearactuator
