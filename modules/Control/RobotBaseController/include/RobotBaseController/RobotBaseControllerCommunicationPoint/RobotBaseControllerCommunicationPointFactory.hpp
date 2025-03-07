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
#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::control::robotbasecontroller {

class RobotBaseControllerCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    RobotBaseControllerCommunicationPointFactory() = delete;
    explicit RobotBaseControllerCommunicationPointFactory(
        std::shared_ptr<RobotBaseControllerManager> manager);
    ~RobotBaseControllerCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<RobotBaseControllerManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::control::robotbasecontroller
