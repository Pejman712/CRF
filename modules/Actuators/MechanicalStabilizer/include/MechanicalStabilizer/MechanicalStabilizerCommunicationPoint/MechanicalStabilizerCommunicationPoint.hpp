/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <map>
#include <string>
#include <functional>
#include <atomic>

#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "MechanicalStabilizer/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"

namespace crf::actuators::mechanicalstabilizer {

class MechanicalStabilizerCommunicationPoint : public crf::utility::devicemanager::PriorityAccessCommunicationPoint { //  NOLINT
 public:
    MechanicalStabilizerCommunicationPoint() = delete;
    MechanicalStabilizerCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::actuators::mechanicalstabilizer::MechanicalStabilizerManager> manager);
    ~MechanicalStabilizerCommunicationPoint() override;

 private:
    std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::actuators::mechanicalstabilizer::MechanicalStabilizerManager> manager_;
    crf::utility::logger::EventLogger logger_;

    bool activateRequestHandler(const communication::datapackets::JSONPacket& packet);
    bool deactivateRequestHandler(const communication::datapackets::JSONPacket& packet);
    bool resetFaultStateRequestHandler(const communication::datapackets::JSONPacket& packet);
};

}  // namespace crf::actuators::mechanicalstabilizer
