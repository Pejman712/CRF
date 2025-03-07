/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
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

#include "MotionController/MotionControllerCommunicationPoint/MotionControllerManager.hpp"
#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::control::motioncontroller {

/**
 * @ingroup group_motion_controller_communication_point
 * @brief Communication point for the Motion Controller. Class that manages the
 * messages received byt the server and parses them into function calls.
 *
 */
class MotionControllerCommunicationPoint: public crf::utility::devicemanager::PriorityAccessCommunicationPoint {  // NOLINT
 public:
    MotionControllerCommunicationPoint() = delete;
    explicit MotionControllerCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::control::motioncontroller::MotionControllerManager> manager);
    ~MotionControllerCommunicationPoint() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::control::motioncontroller::MotionControllerManager> manager_;

    utility::logger::EventLogger logger_;

    void startJointsControlLoopHandler(const communication::datapackets::JSONPacket& packet);
    void startTaskControlLoopHandler(const communication::datapackets::JSONPacket& packet);

    void appendPathHandler(const communication::datapackets::JSONPacket& packet);
    void setVelocityHandler(const communication::datapackets::JSONPacket& packet);
    void setTorqueHandler(const communication::datapackets::JSONPacket& packet);

    void setProfileVelocityHandler(const communication::datapackets::JSONPacket& packet);
    void setProfileAccelerationHandler(const communication::datapackets::JSONPacket& packet);

    void softStopHandler(const communication::datapackets::JSONPacket& packet);
    void hardStopHandler(const communication::datapackets::JSONPacket& packet);

    void setParametersHandler(const communication::datapackets::JSONPacket& packet);
    void getConfigurationHandler(const communication::datapackets::JSONPacket& packet);
};

}  // namespace crf::control::motioncontroller
