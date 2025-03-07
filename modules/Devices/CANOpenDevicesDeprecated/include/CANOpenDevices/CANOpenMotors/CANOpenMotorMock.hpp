/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/STI/ECE 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <utility>

#include "CANOpenDevices/CANOpenMotors/ICANOpenMotor.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class CANOpenMotorMock : public ICANOpenMotor {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());
    MOCK_METHOD0(getCANID,
        int());
    MOCK_METHOD0(isAlive,
        bool());
    MOCK_METHOD0(inFault,
        bool());
    MOCK_METHOD0(inQuickStop,
        bool());
    MOCK_METHOD0(isEnabled,
        bool());
    MOCK_METHOD0(isReadyToSwitchOn,
        bool());
    MOCK_METHOD0(getNMTState,
        std::optional<uint8_t>());
    MOCK_METHOD0(getVelocity,
        std::optional<int32_t>());
    MOCK_METHOD0(getPosition,
        std::optional<int32_t>());
    MOCK_METHOD0(getCurrent,
        std::optional<int32_t>());
    MOCK_METHOD0(getStatusWord,
        std::optional<uint16_t>());
    MOCK_METHOD0(getModeOfOperation,
        std::optional<uint8_t>());
    MOCK_METHOD0(getDigitalInput,
        std::optional<uint32_t>());
    MOCK_METHOD1(setDigitalOutput,
        bool(uint32_t));
    MOCK_METHOD1(resetDigitalOutput,
        bool(uint32_t));
    MOCK_METHOD0(enableOperation,
        bool());
    MOCK_METHOD0(disableOperation,
        bool());
    MOCK_METHOD0(stop,
        bool());
    MOCK_METHOD0(quickStop,
        bool());
    MOCK_METHOD0(shutdown,
        bool());
    MOCK_METHOD0(faultReset,
        bool());
    MOCK_METHOD2(setPosition,
        bool(int32_t position, bool relative));
    MOCK_METHOD3(setPosition,
        bool(int32_t position, uint32_t velocity, bool relative));
    MOCK_METHOD4(setPosition,
        bool(int32_t position, uint32_t velocity, uint32_t acceleration, bool relative));
    MOCK_METHOD5(setPosition,
        bool(int32_t position, uint32_t velocity, uint32_t acceleration, uint32_t deceleration, bool relative));  // NOLINT
    MOCK_METHOD0(positionReached,
        bool());
    MOCK_METHOD1(setVelocity,
        bool(int32_t velocity));
    MOCK_METHOD2(setVelocity,
        bool(int32_t velocity, uint32_t acceleration));
    MOCK_METHOD3(setVelocity,
        bool(int32_t velocity, uint32_t acceleration, uint32_t deceleration));
    MOCK_METHOD1(setTorque,
        bool(int16_t));
    MOCK_METHOD1(setCurrent,
        bool(int16_t));
    MOCK_METHOD1(setProfileVelocity,
        bool(uint32_t));
    MOCK_METHOD1(setProfileAcceleration,
        bool(uint32_t));
    MOCK_METHOD1(setProfileDeceleration,
        bool(uint32_t));
    MOCK_METHOD1(setQuickstopDeceleration,
        bool(uint32_t));
    MOCK_METHOD1(setMaxAcceleration,
        bool(uint32_t));
    MOCK_METHOD1(setPositionLimits,
        bool(std::pair<int32_t, int32_t>));
    MOCK_METHOD0(getProfileVelocity,
        uint32_t());
    MOCK_METHOD0(getProfileAcceleration,
        uint32_t());
    MOCK_METHOD0(getProfileDeceleration,
        uint32_t());
    MOCK_METHOD0(getQuickstopDeceleration,
        uint32_t());
    MOCK_METHOD0(getMaximumVelocity,
        uint32_t());
    MOCK_METHOD0(getMaximumAcceleration,
        uint32_t());
    MOCK_METHOD0(getPositionLimits,
        std::pair<int32_t, int32_t>());
    MOCK_METHOD0(getObjectDictionary,
        std::shared_ptr<ObjectDictionary>());
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
