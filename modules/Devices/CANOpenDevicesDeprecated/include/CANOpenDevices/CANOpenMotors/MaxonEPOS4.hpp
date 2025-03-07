/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>
#include <memory>
#include <string>
#include <utility>

#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/CANOpenSdoManager.hpp"
#include "CANOpenDevices/CANOpenMotors/ICANOpenMotor.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class MaxonEPOS4 : public ICANOpenMotor {
 public:
    MaxonEPOS4() = delete;
    MaxonEPOS4(uint8_t id, std::shared_ptr<communication::cansocket::ICANSocket> socket);
    MaxonEPOS4(const MaxonEPOS4&) = delete;
    MaxonEPOS4(MaxonEPOS4&&) = delete;
    ~MaxonEPOS4() override;

    bool initialize() override;
    bool deinitialize() override;

    int getCANID() override;
    bool isAlive() override;
    bool inFault() override;
    bool inQuickStop() override;
    bool isEnabled() override;
    bool isReadyToSwitchOn() override;

    std::optional<uint8_t> getNMTState() override;
    std::optional<int32_t> getVelocity() override;
    std::optional<int32_t> getPosition() override;
    std::optional<int32_t> getCurrent() override;
    std::optional<uint16_t> getStatusWord() override;
    std::optional<uint8_t> getModeOfOperation() override;
    std::optional<uint32_t> getDigitalInput() override;

    bool setDigitalOutput(uint32_t) override;
    bool resetDigitalOutput(uint32_t) override;
    bool enableOperation() override;
    bool disableOperation() override;
    bool stop() override;
    bool quickStop() override;
    bool shutdown() override;
    bool faultReset() override;
    bool setPosition(int32_t position, bool relative = false) override;
    bool setPosition(int32_t position, uint32_t velocity, bool relative = false) override;
    bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
        bool relative = false) override;
    bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
        uint32_t deceleration, bool relative = false) override;
    bool positionReached() override;
    bool setVelocity(int32_t velocity) override;
    bool setVelocity(int32_t velocity, uint32_t acceleration) override;
    bool setVelocity(int32_t velocity, uint32_t acceleration, uint32_t deceleration) override;
    bool setTorque(int16_t) override;
    bool setCurrent(int16_t) override;

    bool setProfileVelocity(uint32_t) override;
    bool setProfileAcceleration(uint32_t) override;
    bool setProfileDeceleration(uint32_t) override;
    bool setQuickstopDeceleration(uint32_t) override;
    bool setMaxAcceleration(uint32_t) override;
    bool setPositionLimits(std::pair<int32_t, int32_t>) override;

    uint32_t getProfileVelocity() override;
    uint32_t getProfileAcceleration() override;
    uint32_t getProfileDeceleration() override;
    uint32_t getQuickstopDeceleration() override;
    uint32_t getMaximumVelocity() override;
    uint32_t getMaximumAcceleration() override;
    std::pair<int32_t, int32_t> getPositionLimits() override;

    std::shared_ptr<ObjectDictionary> getObjectDictionary() override;

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
    bool quickStopActive_;

    bool setupPDOs();
    std::unique_ptr<CANOpenSdoManager> sdoManager_;

    bool setNMTState(uint8_t);
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
