/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2021
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
#include <condition_variable>

#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerManager.hpp"
#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::control::robotbasecontroller {

class RobotBaseControllerCommunicationPoint: public utility::devicemanager::StatusStreamerCommunicationPoint {  // NOLINT
 public:
    RobotBaseControllerCommunicationPoint() = delete;
    explicit RobotBaseControllerCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::control::robotbasecontroller::RobotBaseControllerManager> manager);
    ~RobotBaseControllerCommunicationPoint() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::control::robotbasecontroller::RobotBaseControllerManager> manager_;

    bool initialized_;
    std::mutex mtx_;
    std::condition_variable cvTrajectory_;
    std::atomic<bool> stopThreads_;
    std::thread trajectoryResultThread_;
    std::future<bool> trajectoryResult_;
    std::mutex trajectoryMtx_;

    utility::logger::EventLogger logger_;

    void resultCheck();

    void lockControlRequestHandler(const communication::datapackets::JSONPacket& packet);
    void unlockControlRequestHandler(const communication::datapackets::JSONPacket& packet);
    void setControllerModeRequestHandler(const communication::datapackets::JSONPacket& packet);

    void setPositionHandler(const communication::datapackets::JSONPacket&);
    void setVelocityHandler(const communication::datapackets::JSONPacket&);
    void interruptTrajectoryHandler(const communication::datapackets::JSONPacket&);
    void setMaximumVelocityHandler(const communication::datapackets::JSONPacket&);
    void setMaximumAccelerationHandler(const communication::datapackets::JSONPacket&);
    void setStageVelocityHandler(const communication::datapackets::JSONPacket&);
    void setStagePositionHandler(const communication::datapackets::JSONPacket&);
};

}  // namespace crf::control::robotbasecontroller
