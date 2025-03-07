/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thomas Breant CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#pragma once

#include <memory>
#include <string>
#include <utility>

#include "CANOpenDevices/CANOpenMotors/ICANOpenMotor.hpp"
#include "CANSocket/ICANSocket.hpp"
#include "CANOpenDevices/CANOpenDefinitions.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"
#include "CANOpenDevices/CANOpenSdoManager.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class ERB : public ICANOpenMotor {
 public:
    ERB() = delete;
    ERB(const uint8_t& id, std::shared_ptr<communication::cansocket::ICANSocket> socket,
        const uint8_t& positionMode);
    ERB(const ERB&) = delete;
    ERB(ERB&&) = delete;
    ~ERB() override;

    bool initialize() override;
    bool deinitialize() override;

    int getCANID() override;
    bool isAlive() override;
    bool inFault() override;
    bool inQuickStop() override;
    bool isSwitchOnDisabled();
    bool isSwitchedOn();
    bool isEnabled() override;
    bool isReadyToSwitchOn() override;

    std::optional<uint8_t> getNMTState() override;
    std::optional<int32_t> getVelocity() override;
    std::optional<int32_t> getPosition() override;
    std::optional<int32_t> getCurrent() override;
    std::optional<int32_t> getTorque();
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
    bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration, bool relative = false) override;  // NOLINT
    bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration, uint32_t deceleration, bool relative = false) override;  // NOLINT
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
    uint8_t id_;
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;
    uint8_t positionMode_;

    std::shared_ptr<ObjectDictionary> objectDictionary_;
    std::unique_ptr<CANOpenSdoManager> sdoManager_;
    bool quickStopActive_;
    bool initialized_;
    crf::utility::logger::EventLogger logger_;

    static uint16_t const NMT_ID    = 0x000;
    const float maxDeceleration_ = 68754.93542;
    const uint16_t heartbeatTime_ = 250;
    const uint16_t heartbeatConsumerId_ = 0x25;
    const std::chrono::milliseconds defaultSdoResponseTimeout_ = std::chrono::milliseconds(100);
    const std::chrono::milliseconds nmtIsAliveThreshold_ = std::chrono::seconds(1);

    bool setupPDOs();
    bool setNMTState(uint8_t);
    bool setModeOfOperation(const uint8_t);
    bool setRights(std::string rights);

    enum eNMT_Command {
        NMT_STARTREMOTENODE = 0x01,
        NMT_STOPREMOTENODE = 0x02,
        NMT_ENTERPREOPERATIONAL = 0x80,
        NMT_RESETNODE = 0x81,
        NMT_RESETCOMMUNICATION = 0x82
    };
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
