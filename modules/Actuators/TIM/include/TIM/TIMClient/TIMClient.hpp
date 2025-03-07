/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <optional>
#include <map>
#include <tuple>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include <nlohmann/json.hpp>
#include "CommunicationUtility/STLJSONConverter.hpp"

#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "DataPacketSocket/PacketSocket.hpp"
#include "EventLogger/EventLogger.hpp"

#include "DeviceManager/DeviceManagerClient/PriorityAccessClient.hpp"
#include "TIM/ITIM.hpp"
#include "TIM/TIMAlarms.hpp"
#include "TIM/TIMCommands.hpp"
#include "TIM/TIMSettings.hpp"
#include "TIM/TIMStatus.hpp"
#include "TIM/TIMConfiguration.hpp"

namespace crf::actuators::tim {

class TIMClient:
    public crf::utility::devicemanager::PriorityAccessClient,
    public ITIM {
 public:
    explicit TIMClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds serverReplyTimeout,
        const float frequency,
        const uint32_t priority);
    ~TIMClient() override;

    bool initialize() override;
    bool deinitialize() override;

    bool isConnected() override;

    std::optional<bool> setCurrentPosition(const float &position) override;
    std::optional<bool> setTargetPosition(const float &position) override;
    std::optional<bool> setTargetVelocity(const float &velocity) override;
    std::optional<bool> moveToTarget(const float &position, const float &velocity) override;
    std::optional<bool> jog(const float &velocity) override;

    std::optional<float> getCurrentPosition() override;
    std::optional<float> getTargetPosition() override;
    std::optional<float> getCurrentVelocity() override;
    std::optional<float> getTargetVelocity() override;
    std::optional<float> getCurrentMaximumVelocity() override;
    std::optional<bool> isMoving() override;
    std::optional<bool> isTargetReached() override;

    std::optional<bool> stop() override;
    std::optional<bool> emergencyStop() override;

    std::optional<bool> extendChargingArm() override;
    std::optional<bool> retractChargingArm() override;
    std::optional<bool> startCharging() override;
    std::optional<bool> stopCharging() override;
    std::optional<bool> isCharging() override;
    std::optional<float> getChargingCurrent() override;
    std::optional<float> getBatteryVoltage() override;
    std::optional<float> getBatteryCurrent() override;
    std::optional<bool> enableEconomyMode() override;
    std::optional<bool> disableEconomyMode() override;
    std::optional<bool> isInEconomy() override;
    std::optional<bool> rebootRobotArmWagon() override;


    std::array<LHCObstacle, 2> getClosestObstacleAreas() override;
    LHCObstacle getCurrentObstacleArea() override;
    std::optional<bool> setObstacleArea(const LHCObstacle &obstacle) override;
    std::optional<bool> devicesRetracted(bool deviceStatus) override;
    std::optional<bool> devicesRetracted() override;

    std::optional<bool> isFrontWarningFieldActive() override;
    std::optional<bool> isBackWarningFieldActive() override;
    std::optional<bool> allowMovement(bool allow) override;
    std::optional<bool> isSafeToMove() override;

    crf::actuators::tim::TIMAlarms getAlarms() override;
    std::optional<bool> acknowledgeAlarms() override;
    std::shared_ptr<TIMConfiguration> getConfiguration() override;

 private:
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket_;
    const std::chrono::milliseconds serverReplyTimeout_;
    uint32_t priority_;

    Receiver<std::optional<bool>> receiverSetCurrentPosition_;
    Receiver<std::optional<bool>> receiverSetTargetPosition_;
    Receiver<std::optional<bool>> receiverSetTargetVelocity_;
    Receiver<std::optional<bool>> receiverMoveToTarget_;
    Receiver<std::optional<bool>> receiverJog_;
    Receiver<std::optional<bool>> receiverStop_;
    Receiver<std::optional<bool>> receiverEmergencyStop_;
    Receiver<std::optional<bool>> receiverExtendChargingArm_;
    Receiver<std::optional<bool>> receiverRetractChargingArm_;
    Receiver<std::optional<bool>> receiverStartCharging_;
    Receiver<std::optional<bool>> receiverStopCharging_;
    Receiver<std::optional<bool>> receiverEnableEconomyMode_;
    Receiver<std::optional<bool>> receiverDisableEconomyMode_;
    Receiver<std::optional<bool>> receiverRebootRobotArmWagon_;
    Receiver<std::optional<bool>> receiverSetObstacleArea_;
    Receiver<std::optional<bool>> receiverDevicesRetracted_;
    Receiver<std::optional<bool>> receiverAllowMovement_;
    Receiver<std::optional<bool>> receiverAcknowledgeAlarms_;

    bool statusIsConnected_;
    std::atomic<std::optional<float>> statusCurrentPosition_;
    std::atomic<std::optional<float>> statusTargetPosition_;
    std::atomic<std::optional<float>> statusCurrentVelocity_;
    std::atomic<std::optional<float>> statusTargetVelocity_;
    std::atomic<std::optional<float>> statusCurrentMaximumVelocity_;
    std::atomic<std::optional<bool>> statusIsMoving_;
    std::atomic<std::optional<bool>> statusIsTargetReached_;
    std::atomic<std::optional<bool>> statusDevicesRetracted_;
    std::atomic<std::optional<bool>> statusIsCharging_;
    std::atomic<std::optional<float>> statusChargingCurrent_;
    std::atomic<std::optional<float>> statusBatteryVoltage_;
    std::atomic<std::optional<float>> statusBatteryCurrent_;
    std::atomic<std::optional<bool>> statusIsInEconomy_;
    std::atomic<std::optional<bool>> statusIsFrontWarningFieldActive_;
    std::atomic<std::optional<bool>> statusIsBackWarningFieldActive_;
    std::atomic<std::optional<bool>> statusIsSafeToMove_;

    // Can't use atomic for these custom classes since they are not trivially copy CTor
    // Also, since it's only a few variables I don't use the statusMtx_ inherited to not
    // block everything (jplayang)
    std::mutex mtx_;
    std::array<LHCObstacle, 2> statusClosestObstacleAreas_;
    LHCObstacle statusCurrentObstacleArea_;
    TIMAlarms statusAlarms_;
    std::shared_ptr<TIMConfiguration> statusConfiguration_;

    crf::utility::logger::EventLogger logger_;

    void parseStatus(const nlohmann::json& json) override;
    std::optional<bool> sendPacket(const communication::datapackets::JSONPacket& json,
        const Receiver<std::optional<bool>>& receiver);
};

}  // namespace crf::actuators::tim
