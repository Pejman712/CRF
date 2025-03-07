/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "EtherCATDevices/IEtherCATMotor.hpp"

#include <utility>
#include <optional>

namespace crf::devices::ethercatdevices {

class EtherCATMotorMock : public IEtherCATMotor {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(int, getID, (), (override));
    MOCK_METHOD(bool, bindPDOs, (), (override));
    MOCK_METHOD(std::optional<uint16_t>, getEtherCatState, (), (override));
    MOCK_METHOD(bool, isAlive, (), (override));
    MOCK_METHOD(std::optional<bool>, inFault, (), (override));
    MOCK_METHOD(std::optional<bool>, inQuickStop, (), (override));
    MOCK_METHOD(std::optional<bool>, isEnabled, (), (override));
    MOCK_METHOD(std::optional<bool>, isReadyToSwitchOn, (), (override));
    MOCK_METHOD(std::optional<bool>, isSwitchOnDisabled, (), (override));
    MOCK_METHOD(std::optional<bool>, isSwitchedOn, (), (override));

    MOCK_METHOD(bool, enableOperation, (), (override));
    MOCK_METHOD(bool, disableOperation, (), (override));
    MOCK_METHOD(bool, disableVoltage, (), (override));
    MOCK_METHOD(bool, stop, (), (override));
    MOCK_METHOD(bool, quickStop, (), (override));
    MOCK_METHOD(bool, shutdown, (), (override));
    MOCK_METHOD(bool, faultReset, (), (override));
    MOCK_METHOD(std::optional<bool>, targetReached, (), (override));
    MOCK_METHOD(std::optional<bool>, internalLimitActive, (), (override));

    MOCK_METHOD(std::optional<int32_t>, getVelocity, (), (override));
    MOCK_METHOD(std::optional<int32_t>, getPosition, (), (override));
    MOCK_METHOD(std::optional<int32_t>, getCurrent, (), (override));

    MOCK_METHOD(std::optional<uint16_t>, getStatusWord, (), (override));
    MOCK_METHOD(std::optional<int8_t>, getModeOfOperation, (), (override));
    MOCK_METHOD(std::optional<bool>, getDigitalInput, (uint32_t bit), (override));
    MOCK_METHOD(std::optional<int16_t>, getTorque, (), (override));
    MOCK_METHOD(std::optional<int16_t>, getAnalogInput, (), (override));

    MOCK_METHOD(bool, setModeOfOperation, (int8_t modeOfOperation), (override));
    MOCK_METHOD(bool, setDigitalOutput, (uint32_t bit), (override));
    MOCK_METHOD(bool, resetDigitalOutput, (uint32_t bit), (override));

    MOCK_METHOD(bool, setPosition, (int32_t position, bool relative), (override));
    MOCK_METHOD(bool, setPosition, (int32_t position, uint32_t velocity,
        bool relative), (override));
    MOCK_METHOD(bool, setPosition, (int32_t position, uint32_t velocity,
        uint32_t acceleration, bool relative), (override));
    MOCK_METHOD(bool, setPosition, (int32_t position, uint32_t velocity,
        uint32_t acceleration, uint32_t deceleration, bool relative), (override));

    MOCK_METHOD(bool, setVelocity, (int32_t velocity), (override));
    MOCK_METHOD(bool, setVelocity, (int32_t velocity, uint32_t acceleration), (override));
    MOCK_METHOD(bool, setVelocity, (int32_t velocity, uint32_t acceleration,
        uint32_t deceleration), (override));

    MOCK_METHOD(bool, setTorque, (int16_t torque), (override));
    MOCK_METHOD(bool, setMaxTorque, (uint16_t MaxTorque), (override));
    MOCK_METHOD(bool, setProfileVelocity, (uint32_t profileVelocity), (override));
    MOCK_METHOD(bool, setProfileAcceleration, (uint32_t profileAcceleration), (override));
    MOCK_METHOD(bool, setProfileDeceleration, (uint32_t profileDeceleration), (override));
    MOCK_METHOD(bool, setMaxVelocity, (uint32_t maxVelocity), (override));
    MOCK_METHOD(bool, setMaxAcceleration, (uint32_t maxAcceleration), (override));
    MOCK_METHOD(bool, setMaxDeceleration, (uint32_t maxDeceleration), (override));
    MOCK_METHOD(bool, setQuickstopDeceleration, (uint32_t quickstopDeceleration), (override));
    MOCK_METHOD(bool, setPositionLimits, ((std::pair<int32_t, int32_t> limits)), (override));
    MOCK_METHOD(bool, setMaxCurrent, (uint16_t MaxCurrent), (override));
    MOCK_METHOD(bool, setMotorRatedCurrent, (uint32_t motorcurrent), (override));
    MOCK_METHOD(bool, setMotorRatedTorque, (uint32_t motortorque), (override));

    MOCK_METHOD(std::optional<uint32_t>, getProfileVelocity, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getProfileAcceleration, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getProfileDeceleration, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getQuickstopDeceleration, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getMaxVelocity, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getMaxAcceleration, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getMaxDeceleration, (), (override));
    MOCK_METHOD((std::optional<std::pair<int32_t, int32_t>>), getPositionLimits, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getMotorRatedCurrent, (), (override));
    MOCK_METHOD(std::optional<uint32_t>, getMotorRatedTorque, (), (override));
};

}  // namespace crf::devices::ethercatdevices
