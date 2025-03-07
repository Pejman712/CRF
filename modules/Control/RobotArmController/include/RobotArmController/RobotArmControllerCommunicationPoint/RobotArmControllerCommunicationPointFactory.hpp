/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::control::robotarmcontroller {

class RobotArmControllerCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    RobotArmControllerCommunicationPointFactory() = delete;
    explicit RobotArmControllerCommunicationPointFactory(
        std::shared_ptr<RobotArmControllerManager> manager);
    ~RobotArmControllerCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<RobotArmControllerManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::control::robotarmcontroller
