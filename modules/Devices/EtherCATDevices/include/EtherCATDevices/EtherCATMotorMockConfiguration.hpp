/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "EtherCATDevices/EtherCATMotorMock.hpp"

#include <utility>
#include <optional>
#include <atomic>

namespace crf::devices::ethercatdevices {

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::_;

class EtherCATMotorMockConfiguration : public EtherCATMotorMock {
 public:
    explicit EtherCATMotorMockConfiguration(const int& id):
        id_(id),
        faultActive_(false),
        quickStopActive_(false),
        actualPosition_(0),
        actualVelocity_(0),
        actualCurrent_(0),
        initialized_(false) {
    }

    ~EtherCATMotorMockConfiguration() override {}

    void setFault(const bool& inFault) {
        faultActive_ = inFault;
    }

    void setQuickStop(const bool& inQuickStop) {
        quickStopActive_ = inQuickStop;
    }

    void configureMock() {
        ON_CALL(*this, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) return false;
                initialized_ = true;
                return true;
            }));
        ON_CALL(*this, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                initialized_ = false;
                return true;
            }));
        ON_CALL(*this, getID()).WillByDefault(Invoke(
            [this] () {
                return id_;
            }));

        ON_CALL(*this, bindPDOs()).WillByDefault(Return(true));
        ON_CALL(*this, getEtherCatState()).WillByDefault(Invoke(
            [this] () -> std::optional<uint16_t> {
                if (!initialized_) return std::nullopt;
                return 0;
            }));
        ON_CALL(*this, isAlive()).WillByDefault(Return(true));

        // Drive State Machine
        ON_CALL(*this, inFault()).WillByDefault(Invoke(
            [this] () -> std::optional<bool> {
                if (!initialized_) return std::nullopt;
                return faultActive_;
            }));
        ON_CALL(*this, inQuickStop()).WillByDefault(Invoke(
            [this] () -> std::optional<bool> {
                if (!initialized_) return std::nullopt;
                return quickStopActive_;
            }));
        ON_CALL(*this, isEnabled()).WillByDefault(Invoke(
            [this] () -> std::optional<bool> {
                if (!initialized_) return std::nullopt;
                if (quickStopActive_ || faultActive_) return false;
                return true;
            }));
        ON_CALL(*this, isReadyToSwitchOn()).WillByDefault(Invoke(
            [this] () -> std::optional<bool> {
                if (!initialized_) return std::nullopt;
                if (quickStopActive_ || faultActive_) return false;
                return true;
            }));
        ON_CALL(*this, isSwitchOnDisabled()).WillByDefault(Invoke(
            [this] () -> std::optional<bool> {
                if (!initialized_) return std::nullopt;
                if (quickStopActive_ || faultActive_) return false;
                return true;
            }));
        ON_CALL(*this, isSwitchedOn()).WillByDefault(Invoke(
            [this] () -> std::optional<bool> {
                if (!initialized_) return std::nullopt;
                if (quickStopActive_ || faultActive_) return false;
                return true;
            }));

        // Trigger transitions
        ON_CALL(*this, enableOperation()).WillByDefault(Return(true));
        ON_CALL(*this, disableOperation()).WillByDefault(Return(true));
        ON_CALL(*this, disableVoltage()).WillByDefault(Return(true));
        ON_CALL(*this, stop()).WillByDefault(Return(true));
        ON_CALL(*this, shutdown()).WillByDefault(Return(true));
        ON_CALL(*this, faultReset()).WillByDefault(Return(true));
        ON_CALL(*this, quickStop()).WillByDefault(Invoke(
            [this] () {
                quickStopActive_ = true;
                return true;
            }));

        // Getters
        ON_CALL(*this, getPosition()).WillByDefault(Invoke(
            [this] () -> std::optional<int32_t> {
                if (!initialized_) return std::nullopt;
                return actualPosition_;
            }));
        ON_CALL(*this, getVelocity()).WillByDefault(Invoke(
            [this] () -> std::optional<int32_t> {
                if (!initialized_) return std::nullopt;
                return actualVelocity_;
            }));
        ON_CALL(*this, getCurrent()).WillByDefault(Invoke(
            [this] () -> std::optional<int32_t> {
                if (!initialized_) return std::nullopt;
                return actualCurrent_;
            }));
        ON_CALL(*this, getTorque()).WillByDefault(Invoke(
            [this] () -> std::optional<int16_t> {
                if (!initialized_) return std::nullopt;
                return actualTorque_;
            }));

        // Other getters not used, if used they need an implementation
        ON_CALL(*this, getStatusWord()).WillByDefault(Return(0));
        ON_CALL(*this, getModeOfOperation()).WillByDefault(Return(0));
        ON_CALL(*this, getDigitalInput(_)).WillByDefault(Return(false));
        ON_CALL(*this, getAnalogInput()).WillByDefault(Return(0));
        ON_CALL(*this, targetReached()).WillByDefault(Return(false));
        ON_CALL(*this, internalLimitActive()).WillByDefault(Return(true));
        ON_CALL(*this, getProfileVelocity()).WillByDefault(Return(0));

        ON_CALL(*this, getProfileAcceleration()).WillByDefault(Return(0));
        ON_CALL(*this, getProfileDeceleration()).WillByDefault(Return(0));
        ON_CALL(*this, getQuickstopDeceleration()).WillByDefault(Return(0));
        ON_CALL(*this, getMaxVelocity()).WillByDefault(Return(0));
        ON_CALL(*this, getMaxAcceleration()).WillByDefault(Return(0));
        ON_CALL(*this, getMaxDeceleration()).WillByDefault(Return(0));
        ON_CALL(*this, getMotorRatedCurrent()).WillByDefault(Return(0));
        ON_CALL(*this, getMotorRatedTorque()).WillByDefault(Return(0));

        // Position limits not implemented for now
        // ON_CALL(*this, getPositionLimits()).WillByDefault(Return(0));


        // Setters
        ON_CALL(*this, setModeOfOperation(_)).WillByDefault(Return(true));
        ON_CALL(*this, setDigitalOutput(_)).WillByDefault(Return(true));
        ON_CALL(*this, resetDigitalOutput(_)).WillByDefault(Return(true));

        // Position
        ON_CALL(*this, setPosition(_, _)).WillByDefault(Invoke(
            [this] (int32_t position, bool relative) {
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualPosition_ = position;
                return true;
            }));
        ON_CALL(*this, setPosition(_, _, _)).WillByDefault(Invoke(
            [this] (int32_t position, uint32_t velocity, bool relative) {
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualPosition_ = position;
                return true;
            }));
        ON_CALL(*this, setPosition(_, _, _, _)).WillByDefault(Invoke(
            [this] (int32_t position, uint32_t velocity, uint32_t acceleration, bool relative) {
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualPosition_ = position;
                return true;
            }));
        ON_CALL(*this, setPosition(_, _, _, _, _)).WillByDefault(Invoke(
            [this] (int32_t position, uint32_t velocity, uint32_t acceleration, uint32_t deceleration, bool relative) {  // NOLINT
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualPosition_ = position;
                return true;
            }));

        // Velocity
        ON_CALL(*this, setVelocity(_)).WillByDefault(Invoke(
            [this] (int32_t velocity) {
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualVelocity_ = velocity;
                return true;
            }));
        ON_CALL(*this, setVelocity(_, _)).WillByDefault(Invoke(
            [this] (int32_t velocity, uint32_t acceleration) {
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualVelocity_ = velocity;
                return true;
            }));
        ON_CALL(*this, setVelocity(_, _, _)).WillByDefault(Invoke(
            [this] (int32_t velocity, uint32_t acceleration, uint32_t deceleration) {
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualVelocity_ = velocity;
                return true;
            }));

        // Torque
        ON_CALL(*this, setVelocity(_)).WillByDefault(Invoke(
            [this] (int16_t torque) {
                if (!initialized_) return false;
                if (quickStopActive_ || faultActive_) return false;
                actualTorque_ = torque;
                return true;
            }));

        // Other setters
        ON_CALL(*this, setMaxTorque(_)).WillByDefault(Return(true));
        ON_CALL(*this, setProfileVelocity(_)).WillByDefault(Return(true));
        ON_CALL(*this, setProfileAcceleration(_)).WillByDefault(Return(true));
        ON_CALL(*this, setProfileDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*this, setMaxVelocity(_)).WillByDefault(Return(true));
        ON_CALL(*this, setMaxAcceleration(_)).WillByDefault(Return(true));
        ON_CALL(*this, setMaxDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*this, setQuickstopDeceleration(_)).WillByDefault(Return(true));
        ON_CALL(*this, setPositionLimits(_)).WillByDefault(Return(true));
        ON_CALL(*this, setMaxCurrent(_)).WillByDefault(Return(true));
        ON_CALL(*this, setMotorRatedCurrent(_)).WillByDefault(Return(true));
        ON_CALL(*this, setMotorRatedTorque(_)).WillByDefault(Return(true));
    }

 private:
    int id_;
    std::atomic<bool> faultActive_;
    std::atomic<bool> quickStopActive_;

    std::atomic<int32_t> actualPosition_;
    std::atomic<int32_t> actualVelocity_;
    std::atomic<int32_t> actualCurrent_;
    std::atomic<int16_t> actualTorque_;

    bool initialized_;
};

}  // namespace crf::devices::ethercatdevices
