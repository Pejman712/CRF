/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
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
#include <condition_variable>
#include <vector>
#include <set>

#include <nlohmann/json.hpp>

#include "TIM/ITIM.hpp"
#include "SiemensPLC/ISiemensPLC.hpp"
#include "TIM/TIMAlarms.hpp"
#include "TIM/TIMCommands.hpp"
#include "TIM/TIMSettings.hpp"
#include "TIM/TIMStatus.hpp"
#include "TIM/TIMConfiguration.hpp"

namespace crf::actuators::tim {

class TIMS300: public ITIM {
 public:
    explicit TIMS300(const nlohmann::json& timConfigFile,
        std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc = nullptr);
    TIMS300(const std::string& timConfigFile,
        std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc) = delete;
    TIMS300(const TIMS300&) = delete;
    TIMS300(TIMS300&&) = delete;
    ~TIMS300() override;

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
    nlohmann::json timConfigFile_;
    std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc_;
    crf::utility::logger::EventLogger logger_;
    bool isInitialized_;
    std::shared_ptr<crf::actuators::tim::TIMConfiguration> config_;
    std::chrono::microseconds commandTimeout_;
    crf::actuators::tim::TIMAlarms alarmsDB_;
    crf::actuators::tim::TIMCommands commandsDB_;
    crf::actuators::tim::TIMSettings settingsDB_;
    crf::actuators::tim::TIMStatus statusDB_;
    std::set<LHCObstacle, CompareLHCObstacle> obstaclesList_;

    std::thread updatePLCValuesThread_;
    std::atomic<bool> stopUpdatePCLValuesThread_;
    std::atomic<bool> isTIMAlive_;
    std::mutex firstGrabberLoopMutex_;
    std::condition_variable firstGrabberLoop_;

    bool saveObstaclesInformation();
    bool updatePLCValues();
    void grabber();

    bool setBooleanCommand(const std::string &name, bool value);
    bool setFloatSetting(const std::string &name, float value);
};

}  // namespace crf::actuators::tim
