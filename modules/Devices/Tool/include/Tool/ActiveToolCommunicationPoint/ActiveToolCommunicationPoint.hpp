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

#include "Tool/ActiveToolCommunicationPoint/ActiveToolManager.hpp"
#include "DeviceManager/DeviceManagerCommunicationPoint/PriorityAccessCommunicationPoint.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::devices::tool {

/**
 * @ingroup group_active_tool
 * @brief
 */
class ActiveToolCommunicationPoint: public crf::utility::devicemanager::PriorityAccessCommunicationPoint {  // NOLINT
 public:
    ActiveToolCommunicationPoint() = delete;
    explicit ActiveToolCommunicationPoint(
        std::shared_ptr<crf::communication::datapacketsocket::PacketSocket> socket,
        std::shared_ptr<crf::devices::tool::ActiveToolManager> manager);
    ~ActiveToolCommunicationPoint() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    std::shared_ptr<crf::devices::tool::ActiveToolManager> manager_;

    utility::logger::EventLogger logger_;

    void activateHandler(const communication::datapackets::JSONPacket&);
    void deactivateHandler(const communication::datapackets::JSONPacket&);
};

}  // namespace crf::devices::tool
