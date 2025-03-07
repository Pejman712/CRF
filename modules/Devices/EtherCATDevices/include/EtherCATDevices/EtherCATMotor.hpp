/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <chrono>
#include <utility>
#include <string>
#include <functional>
#include <optional>

#include "EtherCATDevices/IEtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace ethercatdevices {

class EtherCATMotor : public IEtherCATMotor {
 public:
    EtherCATMotor() = delete;
    EtherCATMotor(uint16_t id, std::shared_ptr<EtherCATManager> ECM);
    EtherCATMotor(const EtherCATMotor&) = delete;
    EtherCATMotor(EtherCATMotor&&) = delete;
    ~EtherCATMotor() override;

    bool initialize() override;
    bool deinitialize() override;
    int getID() override;
    bool bindPDOs() override;
    std::optional<uint16_t> getEtherCatState() override;
    bool isAlive() override;

    // Device Internal State Machine
    std::optional<bool> inFault() override;
    std::optional<bool> inQuickStop() override;
    std::optional<bool> isEnabled() override;
    std::optional<bool> isReadyToSwitchOn() override;
    std::optional<bool> isSwitchOnDisabled() override;
    std::optional<bool> isSwitchedOn() override;
    bool enableOperation() override;
    bool disableOperation() override;
    bool disableVoltage() override;
    bool stop() override;
    bool quickStop() override;
    bool shutdown() override;
    bool faultReset() override;
    std::optional<bool> targetReached() override;
    std::optional<bool> internalLimitActive() override;

    // Motor Data - PDOs
    std::optional<int32_t> getVelocity() override;
    std::optional<int32_t> getPosition() override;
    std::optional<int32_t> getCurrent() override;
    std::optional<uint16_t> getStatusWord() override;
    std::optional<int8_t> getModeOfOperation() override;
    std::optional<bool> getDigitalInput(uint32_t bit) override;
    std::optional<int16_t> getTorque() override;
    std::optional<int16_t> getAnalogInput() override;

    bool setModeOfOperation(int8_t modeOfOperation) override;
    bool setDigitalOutput(uint32_t bit) override;
    bool resetDigitalOutput(uint32_t bit) override;
    bool setPosition(int32_t position, bool relative) override;
    bool setPosition(int32_t position, uint32_t velocity, bool relative) override;
    bool setPosition(int32_t position, uint32_t velocity,
    uint32_t acceleration, bool relative) override;
    bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
    uint32_t deceleration, bool relative) override;
    bool setVelocity(int32_t velocity) override;
    bool setVelocity(int32_t velocity, uint32_t acceleration) override;
    bool setVelocity(int32_t velocity, uint32_t acceleration, uint32_t deceleration) override;
    bool setTorque(int16_t current) override;
    bool setMaxTorque(uint16_t maxTorque) override;

    // Motor Parameters - SDO
    bool setProfileVelocity(uint32_t profileVelocity) override;
    bool setProfileAcceleration(uint32_t profileAcceleration) override;
    bool setProfileDeceleration(uint32_t profileDeceleration) override;
    bool setMaxVelocity(uint32_t maxVelocity) override;
    bool setMaxAcceleration(uint32_t maxAcceleration) override;
    bool setMaxDeceleration(uint32_t maxDeceleration) override;
    bool setQuickstopDeceleration(uint32_t quickstopDeceleration) override;
    bool setPositionLimits(std::pair<int32_t, int32_t>) override;
    bool setMaxCurrent(uint16_t maxCurrent) override;
    bool setMotorRatedCurrent(uint32_t motorcurrent) override;
    bool setMotorRatedTorque(uint32_t motortorque) override;
    std::optional<uint32_t> getProfileVelocity() override;
    std::optional<uint32_t> getProfileAcceleration() override;
    std::optional<uint32_t> getProfileDeceleration() override;
    std::optional<uint32_t> getQuickstopDeceleration() override;
    std::optional<uint32_t> getMaxVelocity() override;
    std::optional<uint32_t> getMaxAcceleration() override;
    std::optional<uint32_t> getMaxDeceleration() override;
    std::optional<std::pair<int32_t, int32_t>> getPositionLimits() override;
    std::optional<uint32_t> getMotorRatedCurrent() override;
    std::optional<uint32_t> getMotorRatedTorque() override;

 private:
    struct EC_output {
        int32_t targetPosition;
        int32_t targetVelocity;
        int16_t targetTorque;
        uint16_t maxTorque;
        uint16_t controlWord;
        int8_t modeOfOperation;
        uint32_t digitalOutputs;
    };
    struct EC_input {
        int32_t positionActualValue;
        uint32_t digitalInputs;
        int32_t velocityActualValue;
        uint16_t statusWord;
        int8_t modeOfOperationDisplay;
        int16_t torqueActualValue;
        int16_t analogInput;
        int16_t currentActualValue;
    };

    EC_output* motorOut_;
    EC_input* motorIn_;
    std::atomic<bool> pdoLinked_;
    utility::logger::EventLogger logger_;
    std::atomic<bool> initialized_;
    uint16_t id_;
    std::shared_ptr<EtherCATManager> ECManager_;

    bool timedCheck_function(std::function<std::optional<bool>()>);
    template<typename T>
    bool timedCheck_withMask(const T& valueFrom, const T& valueCheck,
        int timeTot = constants::totalTime_timedCheck,
        int timeCycle = constants::cycleTime_timedCheck);
    template<typename T>
    bool timedCheck(const T& value1, const T& value2,
        int timeTot = constants::totalTime_timedCheck,
        int timeCycle = constants::cycleTime_timedCheck);
};

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
