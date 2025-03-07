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

#include "CANOpenDevices/CANOpenIOs/ICANOpenIOModule.hpp"
#include "CANOpenDevices/CANOpenSdoManager.hpp"
#include "CANSocket/ICANSocket.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class CANOpenIOModule : public ICANOpenIOModule {
 public:
    CANOpenIOModule() = delete;
    CANOpenIOModule(uint8_t id, std::shared_ptr<communication::cansocket::ICANSocket> socket);
    CANOpenIOModule(const CANOpenIOModule&) = delete;
    CANOpenIOModule(CANOpenIOModule&&) = delete;
    ~CANOpenIOModule() override;

    bool initialize() override;
    bool deinitialize() override;

    int getCANID() override;
    bool isAlive() override;
    std::shared_ptr<ObjectDictionary> getObjectDictionary() override;

    bool hasDigitalInputs() override;
    uint16_t getDigitalInputCount() override;
    std::optional<bool> getDigitalInputState(uint16_t index) override;
    std::optional<bool> getDigitalInputPolarity(uint16_t index) override;
    bool setDigitalInputPolarity(uint16_t index, bool polarity) override;

    bool hasDigitalOutputs() override;
    uint16_t getDigitalOutputCount() override;
    bool setDigitalOutputState(uint16_t index, bool state) override;
    std::optional<bool> getDigitalOutputState(uint16_t index) override;
    bool setDigitalOutputPolarity(uint16_t index, bool polarity) override;
    std::optional<bool> getDigitalOutputPolarity(uint16_t index) override;

    bool hasAnalogueInputs() override;
    uint8_t getAnalogueInputCount() override;
    std::optional<int32_t> getAnalogueInput(uint8_t index) override;

    bool hasAnalogueOutputs() override;
    uint8_t getAnalogueOutputCount() override;
    bool setAnalogueOutput(uint8_t index, int32_t value) override;
    std::optional<int32_t> getAnalogueOutput(uint8_t index) override;

 private:
    utility::logger::EventLogger logger_;
    bool initialized_;

    const uint16_t heartbeatTime_;
    const uint16_t heartbeatConsumerId_;

    uint8_t id_;
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    std::shared_ptr<ObjectDictionary> objectDictionary_;

    const std::chrono::milliseconds defaultSdoResponseTimeout_;
    const std::chrono::milliseconds nmtIsAliveThreshold_;

    std::unique_ptr<CANOpenSdoManager> sdoManager_;

    bool setNMTState(uint8_t);
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
