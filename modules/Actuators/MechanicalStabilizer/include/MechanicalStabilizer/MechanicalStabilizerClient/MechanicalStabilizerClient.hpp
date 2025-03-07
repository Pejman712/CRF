/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <map>

#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "MechanicalStabilizer/IMechanicalStabilizer.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::mechanicalstabilizer {

class MechanicalStabilizerClient:
    public crf::utility::devicemanager::PriorityAccessClient,
    public IMechanicalStabilizer {
 public:
    MechanicalStabilizerClient() = delete;
    explicit MechanicalStabilizerClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency,
        const uint32_t priority);
    MechanicalStabilizerClient(const MechanicalStabilizerClient&) = delete;
    MechanicalStabilizerClient(MechanicalStabilizerClient&&) = delete;
    ~MechanicalStabilizerClient() override;

    bool initialize() override;
    bool deinitialize() override;

    bool activate() override;
    bool deactivate() override;
    std::optional<bool> isActivated() override;
    std::optional<bool> isDeactivated() override;
    bool resetFaultState() override;
    std::optional<bool> isInFault() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    const float frequency_;
    const uint32_t priority_;

    Receiver<bool> receiverActivate_;
    Receiver<bool> receiverDeactivate_;
    Receiver<bool> receiverResetFaultState_;

    std::optional<bool> statusIsActivated_;
    std::optional<bool> statusIsDeactivated_;
    std::optional<bool> statusIsInFault_;

    crf::utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;
    bool sendPacket(
        const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);
};

}  // namespace crf::actuators::mechanicalstabilizer
