/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN BE/CEM/MRO 2024
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

#include "CANopenDrivers/CiA402/CiA402CommunicationPoint/CiA402Manager.hpp"
#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two_communication_point
 * @brief Communication point for the CiA402Driver associated with the "Device Profile for Drives and Motion Control."
 * One of the key aspects of this profile are the below modes of operation
 */
class CiA402CommunicationPoint: public crf::utility::devicemanager::PriorityAccessCommunicationPoint {  // NOLINT
 public:
    CiA402CommunicationPoint() = delete;
    explicit CiA402CommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::devices::canopendrivers::CiA402Manager> manager);
    ~CiA402CommunicationPoint() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::devices::canopendrivers::CiA402Manager> manager_;

    utility::logger::EventLogger logger_;

    void deinitialiseDeviceManagerOnSocketClose() override;
    void heartbeatLost() override;

    void setProfilePositionHandler(const communication::datapackets::JSONPacket&);
    void setProfileVelocityHandler(const communication::datapackets::JSONPacket&);
    void setProfileTorqueHandler(const communication::datapackets::JSONPacket&);

    void setVelocityHandler(const communication::datapackets::JSONPacket&);
    void setMaximumTorqueHandler(const communication::datapackets::JSONPacket&);

    void setInterpolatedPositionHandler(const communication::datapackets::JSONPacket&);
    void setModeOfOperationHandler(const communication::datapackets::JSONPacket&);

    void setCyclicPositionHandler(const communication::datapackets::JSONPacket&);
    void setCyclicVelocityHandler(const communication::datapackets::JSONPacket&);
    void setCyclicTorqueHandler(const communication::datapackets::JSONPacket&);

    void quickStopHandler(const communication::datapackets::JSONPacket&);
    void stopHandler(const communication::datapackets::JSONPacket&);
    void resetFaultHandler(const communication::datapackets::JSONPacket&);
    void resetQuickStopHandler(const communication::datapackets::JSONPacket&);
};

}  // namespace crf::devices::canopendrivers
