/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "TIMRPWagon/ITIMRPWagon.hpp"
#include "TIMRPWagon/TIMRPWagonConfiguration.hpp"
#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"

namespace crf::actuators::timrpwagon {

class TIMRPWagonClient:
    public crf::utility::devicemanager::PriorityAccessClient,
    public ITIMRPWagon {
 public:
    explicit TIMRPWagonClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency,
        const uint32_t priority);
    ~TIMRPWagonClient() override;

    bool initialize() override;
    bool deinitialize() override;

    bool isConnected() override;

    RPArmPosition getRPArmPosition() override;
    std::optional<bool> retractRPArm() override;
    std::optional<bool> deployRPArm() override;
    std::optional<bool> moveRPArmUp() override;
    std::optional<bool> moveRPArmDown() override;
    std::optional<bool> stopRPArm() override;
    std::optional<bool> lockRPArm() override;
    std::optional<bool> unlockRPArm() override;
    std::optional<bool> isRPArmInError() override;
    std::optional<bool> resetRPArmDriver() override;
    std::optional<bool> acknowledgeErrors() override;
    std::shared_ptr<TIMRPWagonConfiguration> getConfiguration() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    const int priority_;

    Receiver<bool> receiverRetractRPArm_;
    Receiver<bool> receiverDeployRPArm_;
    Receiver<bool> receiverMoveRPArmUp_;
    Receiver<bool> receiverMoveRPArmDown_;
    Receiver<bool> receiverStopRPArm_;
    Receiver<bool> receiverLockRPArm_;
    Receiver<bool> receiverUnlockRPArm_;
    Receiver<bool> receiverResetRPArmDriver_;
    Receiver<bool> receiverAcknowledgeErrors_;

    std::atomic<RPArmPosition> statusRPArmPosition_;
    std::atomic<bool> statusIsRPArmInError_;
    std::atomic<bool> statusIsConnected_;

    utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;
    bool sendPacket(const communication::datapackets::JSONPacket& json,
        const Receiver<bool>& receiver);
};

}  // namespace crf::actuators::timrpwagon
