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
#include "TIM/TIMCommunicationPoint/TIMManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf::actuators::tim {

class TIMCommunicationPoint : public utility::devicemanager::PriorityAccessCommunicationPoint {
 public:
    TIMCommunicationPoint() = delete;
    TIMCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::actuators::tim::TIMManager> timManager);
    ~TIMCommunicationPoint() override;

 private:
    std::shared_ptr<crf::actuators::tim::TIMManager> timManager_;
    crf::utility::logger::EventLogger logger_;

    void setCurrentPositionHandler(const communication::datapackets::JSONPacket& packet);
    void setTargetPositionHandler(const communication::datapackets::JSONPacket& packet);
    void setTargetVelocityHandler(const communication::datapackets::JSONPacket& packet);
    void moveToTargetHandler(const communication::datapackets::JSONPacket& packet);
    void jogHandler(const communication::datapackets::JSONPacket& packet);
    void stopHandler(const communication::datapackets::JSONPacket& packet);
    void emergencyStopHandler(const communication::datapackets::JSONPacket& packet);
    void extendChargingArmHandler(const communication::datapackets::JSONPacket& packet);
    void retractChargingArmHandler(const communication::datapackets::JSONPacket& packet);
    void startChargingHandler(const communication::datapackets::JSONPacket& packet);
    void stopChargingHandler(const communication::datapackets::JSONPacket& packet);
    void enableEconomyModeHandler(const communication::datapackets::JSONPacket& packet);
    void disableEconomyModeHandler(const communication::datapackets::JSONPacket& packet);
    void rebootRobotArmWagonHandler(const communication::datapackets::JSONPacket& packet);
    void setObstacleAreaHandler(const communication::datapackets::JSONPacket& packet);
    void devicesRetractedHandler(const communication::datapackets::JSONPacket& packet);
    void allowMovementHandler(const communication::datapackets::JSONPacket& packet);
    void acknowledgeAlarmsHandler(const communication::datapackets::JSONPacket& packet);
};

}  // namespace crf::actuators::tim
