/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>
#include <memory>
#include <utility>

#include "CANOpenDevices/ICANOpenDevice.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class ICANOpenIOModule : public ICANOpenDevice {
 public:
    ~ICANOpenIOModule() override = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    int getCANID() override = 0;
    bool isAlive() override = 0;
    std::shared_ptr<ObjectDictionary> getObjectDictionary() override = 0;

    virtual bool hasDigitalInputs() = 0;
    virtual uint16_t getDigitalInputCount() = 0;
    virtual std::optional<bool> getDigitalInputState(uint16_t index) = 0;
    virtual std::optional<bool> getDigitalInputPolarity(uint16_t index) = 0;
    virtual bool setDigitalInputPolarity(uint16_t index, bool polarity) = 0;

    virtual bool hasDigitalOutputs() = 0;
    virtual uint16_t getDigitalOutputCount() = 0;
    virtual bool setDigitalOutputState(uint16_t index, bool state) = 0;
    virtual std::optional<bool> getDigitalOutputState(uint16_t index) = 0;
    virtual bool setDigitalOutputPolarity(uint16_t index, bool polarity) = 0;
    virtual std::optional<bool> getDigitalOutputPolarity(uint16_t index) = 0;

    virtual bool hasAnalogueInputs() = 0;
    virtual uint8_t getAnalogueInputCount() = 0;
    virtual std::optional<int32_t> getAnalogueInput(uint8_t index) = 0;

    virtual bool hasAnalogueOutputs() = 0;
    virtual uint8_t getAnalogueOutputCount() = 0;
    virtual bool setAnalogueOutput(uint8_t index, int32_t value) = 0;
    virtual std::optional<int32_t> getAnalogueOutput(uint8_t index) = 0;
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
