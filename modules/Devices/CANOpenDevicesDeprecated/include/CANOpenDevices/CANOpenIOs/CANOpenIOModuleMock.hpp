/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <optional>

#include "CANOpenDevices/CANOpenIOs/ICANOpenIOModule.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class CANOpenIOModuleMock : public ICANOpenIOModule {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());
    MOCK_METHOD0(getCANID,
        int());
    MOCK_METHOD0(isAlive,
        bool());
    MOCK_METHOD0(getObjectDictionary,
        std::shared_ptr<ObjectDictionary>());
    MOCK_METHOD0(hasDigitalInputs,
        bool());
    MOCK_METHOD0(getDigitalInputCount,
        uint16_t());
    MOCK_METHOD1(getDigitalInputState,
        std::optional<bool>(uint16_t index));
    MOCK_METHOD1(getDigitalInputPolarity,
        std::optional<bool>(uint16_t index));
    MOCK_METHOD2(setDigitalInputPolarity,
        bool(uint16_t index, bool polarity));
    MOCK_METHOD0(hasDigitalOutputs,
        bool());
    MOCK_METHOD0(getDigitalOutputCount,
        uint16_t());
    MOCK_METHOD2(setDigitalOutputState,
        bool(uint16_t index, bool state));
    MOCK_METHOD1(getDigitalOutputState,
        std::optional<bool>(uint16_t index));
    MOCK_METHOD2(setDigitalOutputPolarity,
        bool(uint16_t index, bool polarity));
    MOCK_METHOD1(getDigitalOutputPolarity,
        std::optional<bool>(uint16_t index));
    MOCK_METHOD0(hasAnalogueInputs,
        bool());
    MOCK_METHOD0(getAnalogueInputCount,
        uint8_t());
    MOCK_METHOD1(getAnalogueInput,
        std::optional<int32_t>(uint8_t index));
    MOCK_METHOD0(hasAnalogueOutputs,
        bool());
    MOCK_METHOD0(getAnalogueOutputCount,
        uint8_t());
    MOCK_METHOD2(setAnalogueOutput,
        bool(uint8_t index, int32_t value));
    MOCK_METHOD1(getAnalogueOutput,
        std::optional<int32_t>(uint8_t index));
};


}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
