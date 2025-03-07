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

#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"
#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::control::robotarmcontroller {

class RobotArmControllerCommunicationPoint: public utility::devicemanager::StatusStreamerCommunicationPoint {  // NOLINT
 public:
    RobotArmControllerCommunicationPoint() = delete;
    explicit RobotArmControllerCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerManager> manager);
    ~RobotArmControllerCommunicationPoint() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::control::robotarmcontroller::RobotArmControllerManager> manager_;

    bool initialized_;
    std::mutex mtx_;
    std::condition_variable cvTrajectory_;
    std::mutex controllerMutex_;
    std::atomic<bool> stopThreads_;
    std::thread trajectoryResultThread_;
    std::future<bool> trajectoryResult_;
    std::mutex trajectoryMtx_;

    utility::logger::EventLogger logger_;

    void resultCheck();

    void lockControlRequestHandler(const communication::datapackets::JSONPacket&);
    void unlockControlRequestHandler(const communication::datapackets::JSONPacket&);
    void setControllerModeRequestHandler(const communication::datapackets::JSONPacket&);

    void setPositionHandler(const communication::datapackets::JSONPacket&);
    void setVelocityHandler(const communication::datapackets::JSONPacket&);
    void setAccelerationHandler(const communication::datapackets::JSONPacket&);
    void interruptTrajectoryHandler(const communication::datapackets::JSONPacket&);
    void setJointsMaximumVelocityHandler(const communication::datapackets::JSONPacket&);
    void setJointsMaximumAccelerationHandler(const communication::datapackets::JSONPacket&);
    void setTaskMaximumVelocityHandler(const communication::datapackets::JSONPacket&);
    void setTaskMaximumAccelerationHandler(const communication::datapackets::JSONPacket&);
    void setGripperPositionHandler(const communication::datapackets::JSONPacket&);
    void setGripperVelocityHandler(const communication::datapackets::JSONPacket&);
};

}  // namespace crf::control::robotarmcontroller
