/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include "LinearStage/LinearActuatorMock.hpp"

using testing::Invoke;
using testing::_;

namespace crf::actuators::linearactuator {

class LinearActuatorMockConfiguration : public LinearActuatorMock {
 public:
    LinearActuatorMockConfiguration():
        initialized_(false),
        currentPosition_(0.0),
        currentVelocity_(0.0) {
    }

    ~LinearActuatorMockConfiguration() override {}

    void setStartingPosition(const double& position) {
        currentPosition_ = position;
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

        ON_CALL(*this, setPosition(_)).WillByDefault(Invoke(
            [this] (const double& position) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                currentPosition_ = position;
                return true;
            }));
        ON_CALL(*this, setVelocity(_)).WillByDefault(Invoke(
            [this] (const double& velocity) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                currentVelocity_ = velocity;
                return true;
            }));

        ON_CALL(*this, getPosition()).WillByDefault(Invoke(
            [this] () -> crf::expected<double> {
                if (!initialized_) return crf::Code::NotInitialized;
                return currentPosition_.load();
            }));
        ON_CALL(*this, getVelocity()).WillByDefault(Invoke(
            [this] () -> crf::expected<double> {
                if (!initialized_) return crf::Code::NotInitialized;
                return currentVelocity_.load();
            }));
    }

 private:
    std::atomic<bool> initialized_;
    std::atomic<double> currentPosition_;
    std::atomic<double> currentVelocity_;
};

}  // namespace crf::actuators::linearactuator
