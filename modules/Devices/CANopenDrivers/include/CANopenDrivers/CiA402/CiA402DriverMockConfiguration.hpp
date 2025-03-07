/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *         Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "CANopenDrivers/CiA402/CiA402DriverMock.hpp"

#include "EventLogger/EventLogger.hpp"
#include "crf/expected.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::An;

namespace crf::devices::canopendrivers {

class CiA402DriverMockConfiguration : public CiA402DriverMock {
 public:
    CiA402DriverMockConfiguration():
        initialized_(false),
        position_(0.0),
        velocity_(0.0),
        torque_(0.0),
        maximumTorque_(0.0),
        mode_(ModeOfOperation::NoOperatingMode),
        logger_("CiA402DriverMockConfiguration") {
        logger_->debug("CTor");
    }

    ~CiA402DriverMockConfiguration() {
        logger_->info("DTor");
    }

    void configureMock() {
        ON_CALL(*this, initialize()).WillByDefault(Invoke([this] {
            if (initialized_) return false;
            initialized_ = true;
            return true;
        }));
        ON_CALL(*this, deinitialize()).WillByDefault(Invoke([this] {
            if (!initialized_) return false;
            initialized_ = false;
            return true;
        }));

        ON_CALL(*this, inFault()).WillByDefault(Return(false));
        ON_CALL(*this, inQuickStop()).WillByDefault(Return(false));
        ON_CALL(*this, resetFault()).WillByDefault(Invoke([this] {
            if (!initialized_) return false;
            return true;
        }));
        ON_CALL(*this, quickStop()).WillByDefault(Invoke([this] {
            if (!initialized_) return false;
            return true;
        }));
        ON_CALL(*this, stop()).WillByDefault(Invoke([this] {
            return;
        }));
        ON_CALL(*this, resetQuickStop()).WillByDefault(Invoke([this] {
            if (!initialized_) return false;
            return true;
        }));
        ON_CALL(*this, getMotorStatus()).WillByDefault(Invoke([this] {
            return std::vector<crf::ResponseCode>({crf::ResponseCode(crf::Code::OK)});
        }));
        ON_CALL(*this, getStatusWord()).WillByDefault(Invoke([this] {
            if (!initialized_) return StatusWord::SwitchONDisabled;
            return StatusWord::OperationEnabled;
        }));
        ON_CALL(*this, setModeOfOperation(_)).WillByDefault(Invoke(
            [this] (const ModeOfOperation& mode) -> crf::expected<bool> {
            if (!initialized_) return crf::Code::NotInitialized;
            return true;
        }));
        ON_CALL(*this, setMaximumTorque(_)).WillByDefault(Invoke(
            [this] (const double& maximumTorque) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                maximumTorque_ = maximumTorque;
                return true;
            }));
        ON_CALL(*this, getPosition()).WillByDefault(Invoke(
            [this] () -> crf::expected<double> {
            if (!initialized_) return crf::Code::NotInitialized;
            return position_.load();
        }));
        ON_CALL(*this, getVelocity()).WillByDefault(Invoke(
            [this] () -> crf::expected<double> {
            if (!initialized_) return crf::Code::NotInitialized;
            return velocity_.load();
        }));
        ON_CALL(*this, getTorque()).WillByDefault(Invoke(
            [this] () -> crf::expected<double> {
            if (!initialized_) return crf::Code::NotInitialized;
            return torque_.load();
        }));
        ON_CALL(*this, getModeOfOperation()).WillByDefault(Invoke([this] {
            if (!initialized_) return ModeOfOperation::ProfilePositionMode;
            return mode_.load();
        }));
        ON_CALL(*this, getMaximumTorque()).WillByDefault(Invoke(
            [this] () -> crf::expected<double> {
            if (!initialized_) return crf::Code::NotInitialized;
            return maximumTorque_.load();
        }));

        ON_CALL(*this, setProfilePosition(_, _, _, _, _)).WillByDefault(Invoke(
            [this] (const double& position,
                    const double& velocity,
                    const double& acceleration,
                    const double& deceleration,
                    const PositionReference& positionReference) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                position_ = position;
                mode_ = ModeOfOperation::ProfilePositionMode;
                return true;
        }));
        ON_CALL(*this, setVelocity(_, _, _, _, _)).WillByDefault(Invoke(
            [this] (double velocity,
                    double deltaSpeedAcc,
                    double deltaTimeAcc,
                    double deltaSpeedDec,
                    double delatTimeDec) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                velocity_ = velocity;
                mode_ = ModeOfOperation::VelocityMode;
                return true;
        }));
        ON_CALL(*this, setProfileVelocity(_, _, _)).WillByDefault(Invoke(
            [this] (const double& velocity,
                    const double& acceleration,
                    const double& deceleration) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                velocity_ = velocity;
                mode_ = ModeOfOperation::ProfileVelocityMode;
                return true;
        }));
        ON_CALL(*this, setProfileTorque(_)).WillByDefault(Invoke(
            [this] (double torque) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                torque_ = torque;
                mode_ = ModeOfOperation::ProfileTorqueMode;
                return true;
        }));
        ON_CALL(*this, setInterpolatedPosition(_, _, _, _)).WillByDefault(Invoke(
            [this] (const double& position,
                    const double& velocity,
                    const double& acceleration,
                    const double& deceleration) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                position_ = position;
                mode_ = ModeOfOperation::InterpolatedPositionMode;
                return true;
        }));
        ON_CALL(*this, setCyclicPosition(_, _, _, _)).WillByDefault(Invoke(
            [this] (const double& position,
                    const double& posOffset,
                    const double& velOffset,
                    const double& torOffset) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                position_ = position + posOffset;
                mode_ = ModeOfOperation::CyclicSyncPositionMode;
                return true;
        }));
        ON_CALL(*this, setCyclicVelocity(_, _, _)).WillByDefault(Invoke(
            [this] (const double& velocity,
                    const double& velOffset,
                    const double& torOffset) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                velocity_ = velocity + velOffset;
                mode_ = ModeOfOperation::CyclicSyncVelocityMode;
                return true;
        }));
        ON_CALL(*this, setCyclicTorque(_, _)).WillByDefault(Invoke(
            [this] (const double& torque,
                    const double& torOffset) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                torque_ = torque + torOffset;
                mode_ = ModeOfOperation::CyclicSyncTorqueMode;
                return true;
        }));
    }

 private:
    std::atomic<bool> initialized_;
    std::atomic<double> position_;
    std::atomic<double> velocity_;
    std::atomic<double> torque_;
    std::atomic<double> maximumTorque_;
    std::atomic<ModeOfOperation> mode_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::devices::canopendrivers
