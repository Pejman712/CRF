/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */
#pragma once

#include <memory>
#include <map>
#include <string>
#include <mutex>

#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "Shielding/IShielding.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::shielding {

class ShieldingClient:
    public crf::utility::devicemanager::PriorityAccessClient,
    public IShielding {
 public:
    explicit ShieldingClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency,
        const uint32_t priority);
    ~ShieldingClient();
    bool initialize() override;
    bool deinitialize() override;

    bool open() override;
    bool close() override;
    std::optional<bool> isOpen() override;
    std::optional<bool> isClosed() override;
    bool resetFaultState() override;
    std::optional<bool> isInFault() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds& serverReplyTimeout_;
    const float& frequency_;
    const uint32_t& priority_;

    Receiver<bool> receiverOpen_;
    Receiver<bool> receiverClose_;
    Receiver<bool> receiverResetFaultState_;

    std::optional<bool> statusIsOpen_;
    std::optional<bool> statusIsClosed_;
    std::optional<bool> statusIsInFault_;

    utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;
    bool sendPacket(
        const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);
};

}  // namespace crf::actuators::shielding
