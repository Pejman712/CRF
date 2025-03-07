/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>

#include "DeviceManager/DeviceManagerCommunicationPoint/StatusStreamerCommunicationPoint.hpp"
#include "MissionManager/IMissionManager.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::applications::missionmanager {

class MissionManagerCommunicationPoint:
    public crf::utility::devicemanager::StatusStreamerCommunicationPoint {
 public:
    MissionManagerCommunicationPoint() = delete;
    explicit MissionManagerCommunicationPoint(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::applications::missionmanager::IMissionManager> mission);
    ~MissionManagerCommunicationPoint();

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::applications::missionmanager::IMissionManager> mission_;
    crf::utility::logger::EventLogger logger_;

    void startHandler(const communication::datapackets::JSONPacket& packet);
    void nextHandler(const communication::datapackets::JSONPacket& packet);
    void stopHandler(const communication::datapackets::JSONPacket& packet);
    void pauseHandler(const communication::datapackets::JSONPacket& packet);
    void resumeHandler(const communication::datapackets::JSONPacket& packet);
    void goHomeHandler(const communication::datapackets::JSONPacket& packet);
    void rechargeHandler(const communication::datapackets::JSONPacket& packet);
    void getStatusHandler(const communication::datapackets::JSONPacket& packet);
    void setStatusHandler(const communication::datapackets::JSONPacket& packet);
    void emergencyHandler(const communication::datapackets::JSONPacket& packet);
    void rearmHandler(const communication::datapackets::JSONPacket& packet);
};

}  // namespace crf::applications::missionmanager
