/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <chrono>

#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "TIMRPWagon/TIMRPWagonCommunicationPoint/TIMRPWagonManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf::actuators::timrpwagon {

class TIMRPWagonCommunicationPoint :
    public utility::devicemanager::PriorityAccessCommunicationPoint {
 public:
    TIMRPWagonCommunicationPoint() = delete;
    TIMRPWagonCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::actuators::timrpwagon::TIMRPWagonManager> timRPWagonManager);
    ~TIMRPWagonCommunicationPoint() override;

 private:
    std::shared_ptr<crf::actuators::timrpwagon::TIMRPWagonManager> timRPWagonManager_;
    crf::utility::logger::EventLogger logger_;

    void retractRPArmHandler(const communication::datapackets::JSONPacket& packet);
    void deployRPArmHandler(const communication::datapackets::JSONPacket& packet);
    void moveRPArmUpHandler(const communication::datapackets::JSONPacket& packet);
    void moveRPArmDownHandler(const communication::datapackets::JSONPacket& packet);
    void stopRPArmHandler(const communication::datapackets::JSONPacket& packet);
    void lockRPArmHandler(const communication::datapackets::JSONPacket& packet);
    void unlockRPArmHandler(const communication::datapackets::JSONPacket& packet);
    void resetRPArmDriverHandler(const communication::datapackets::JSONPacket& packet);
    void acknowledgeErrorsHandler(const communication::datapackets::JSONPacket& packet);
};

}  // namespace crf::actuators::timrpwagon
