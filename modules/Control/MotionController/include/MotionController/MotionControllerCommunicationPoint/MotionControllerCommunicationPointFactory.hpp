/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "CommunicationPointServer/ICommunicationPointFactory.hpp"
#include "MotionController/MotionControllerCommunicationPoint/MotionControllerManager.hpp"
#include "CommunicationPointServer/ICommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::control::motioncontroller {

/**
 * @ingroup group_motion_controller_communication_point
 * @brief Class to generate Motion controller communication points for each client
 * connected to the server.
 *
 */
class MotionControllerCommunicationPointFactory:
    public communication::communicationpointserver::ICommunicationPointFactory {
 public:
    MotionControllerCommunicationPointFactory() = delete;
    explicit MotionControllerCommunicationPointFactory(
        std::shared_ptr<MotionControllerManager> manager);
    ~MotionControllerCommunicationPointFactory() override;

    std::optional<std::shared_ptr<communication::communicationpointserver::ICommunicationPoint>>
        create(std::shared_ptr<communication::datapacketsocket::PacketSocket>) override;

 private:
    std::shared_ptr<MotionControllerManager> manager_;
    utility::logger::EventLogger logger_;
};

}  // namespace crf::control::motioncontroller
