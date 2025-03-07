/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sebastien Collomb CERN EN/SMM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <thread>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <optional>
#include <mutex>
#include <random>
#include <atomic>

#include <gmock/gmock.h>

#include "IMU/IMUMock.hpp"

using testing::Invoke;
using testing::_;

namespace crf::sensors::imu {

class IMUMockConfiguration : public IMUMock {
 public:
    IMUMockConfiguration():
        initialized_(false) {
    }

    ~IMUMockConfiguration() override {}

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

        ON_CALL(*this, calibrate()).WillByDefault(Invoke(
            [this] () -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                return true;
            }));

        ON_CALL(*this, getSignal()).WillByDefault(Invoke(
            [this] () -> crf::sensors::imu::IMUSignals {
                crf::sensors::imu::IMUSignals sig;
                sig.position = std::array<double, 3>({3, 3, 3});
                sig.quaternion = std::array<double, 4>({3, 3, 3, 3});
                sig.eulerZYX = std::array<double, 3>({3, 3, 3});
                sig.linearVelocity = std::array<double, 3>({3, 3, 3});;
                sig.angularVelocity = std::array<double, 3>({3, 3, 3});;
                sig.linearAcceleration = std::array<double, 3>({3, 3, 3});;
                sig.angularAcceleration = std::array<double, 3>({3, 3, 3});;
                sig.magneticField = std::array<double, 3>({3, 3, 3});;
                return sig;
            }));
    }

 private:
    std::atomic<bool> initialized_;
    std::thread measurement_;
};

}  // namespace crf::sensors::imu
