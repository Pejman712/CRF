/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/
#pragma once

#include <memory>

#include "TIM/ITIM.hpp"

namespace crf::actuators::tim {

class TIMMock : public ITIM {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(bool, isConnected, (), (override));

    MOCK_METHOD(std::optional<bool>, setCurrentPosition, (const float &position), (override));
    MOCK_METHOD(std::optional<bool>, setTargetPosition, (const float &position), (override));
    MOCK_METHOD(std::optional<bool>, setTargetVelocity, (const float &velocity), (override));
    MOCK_METHOD(std::optional<bool>, moveToTarget, (const float &position, const float &velocity),
        (override));
    MOCK_METHOD(std::optional<bool>, jog, (const float &velocity), (override));

    MOCK_METHOD(std::optional<float>, getCurrentPosition, (), (override));
    MOCK_METHOD(std::optional<float>, getTargetPosition, (), (override));
    MOCK_METHOD(std::optional<float>, getCurrentVelocity, (), (override));
    MOCK_METHOD(std::optional<float>, getTargetVelocity, (), (override));
    MOCK_METHOD(std::optional<float>, getCurrentMaximumVelocity, (), (override));

    MOCK_METHOD(std::optional<bool>, isMoving, (), (override));
    MOCK_METHOD(std::optional<bool>, isTargetReached, (), (override));
    MOCK_METHOD(std::optional<bool>, stop, (), (override));
    MOCK_METHOD(std::optional<bool>, emergencyStop, (), (override));
    MOCK_METHOD(std::optional<bool>, extendChargingArm, (), (override));
    MOCK_METHOD(std::optional<bool>, retractChargingArm, (), (override));
    MOCK_METHOD(std::optional<bool>, startCharging, (), (override));
    MOCK_METHOD(std::optional<bool>, stopCharging, (), (override));
    MOCK_METHOD(std::optional<bool>, isCharging, (), (override));

    MOCK_METHOD(std::optional<float>, getChargingCurrent, (), (override));
    MOCK_METHOD(std::optional<float>, getBatteryVoltage, (), (override));
    MOCK_METHOD(std::optional<float>, getBatteryCurrent, (), (override));

    MOCK_METHOD(std::optional<bool>, enableEconomyMode, (), (override));
    MOCK_METHOD(std::optional<bool>, disableEconomyMode, (), (override));
    MOCK_METHOD(std::optional<bool>, isInEconomy, (), (override));
    MOCK_METHOD(std::optional<bool>, rebootRobotArmWagon, (), (override));

    MOCK_METHOD((std::array<crf::actuators::tim::LHCObstacle, 2>), getClosestObstacleAreas, (),
        (override));
    MOCK_METHOD(std::optional<bool>, setObstacleArea, (const LHCObstacle &obstacle), (override));
    MOCK_METHOD(crf::actuators::tim::LHCObstacle, getCurrentObstacleArea, (), (override));
    MOCK_METHOD(std::optional<bool>, devicesRetracted, (bool deviceStatus), (override));
    MOCK_METHOD(std::optional<bool>, devicesRetracted, (), (override));
    MOCK_METHOD(std::optional<bool>, isFrontWarningFieldActive, (), (override));
    MOCK_METHOD(std::optional<bool>, isBackWarningFieldActive, (), (override));
    MOCK_METHOD(std::optional<bool>, allowMovement, (bool allow), (override));
    MOCK_METHOD(std::optional<bool>, isSafeToMove, (), (override));

    MOCK_METHOD(crf::actuators::tim::TIMAlarms, getAlarms, (), (override));
    MOCK_METHOD(std::optional<bool>, acknowledgeAlarms, (), (override));
    MOCK_METHOD(std::shared_ptr<TIMConfiguration>, getConfiguration, (), (override));
};

}  // namespace crf::actuators::tim
