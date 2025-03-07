/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "LinearStage/LinearActuatorCommunicationPoint/LinearActuatorManager.hpp"
#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::actuators::linearactuator {

/**
 * @ingroup group_linear_actuator_communication_point
 * @brief TODO
 */
class LinearActuatorCommunicationPoint: public crf::utility::devicemanager::PriorityAccessCommunicationPoint {  // NOLINT
 public:
    LinearActuatorCommunicationPoint() = delete;
    explicit LinearActuatorCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::actuators::linearactuator::LinearActuatorManager> manager);
    ~LinearActuatorCommunicationPoint() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::actuators::linearactuator::LinearActuatorManager> manager_;

    utility::logger::EventLogger logger_;

    void setPositionHandler(const communication::datapackets::JSONPacket&);
    void setVelocityHandler(const communication::datapackets::JSONPacket&);
};

}  // namespace crf::actuators::linearactuator
