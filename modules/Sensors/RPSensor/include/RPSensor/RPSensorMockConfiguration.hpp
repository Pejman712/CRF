/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
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
#include <gtest/gtest.h>

#include "RPSensor/RPSensorMock.hpp"

namespace crf {
namespace sensors {
namespace rpsensor {

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::_;

class RPSensorMockConfiguration : public RPSensorMock {
 public:
    RPSensorMockConfiguration():
        initialized_(false),
        dose_(0),
        cumulativeDose_(0),
        stopThreads_(true) {
    }

    ~RPSensorMockConfiguration() {
        stopThreads_ = true;
        if (measurement_.joinable()) measurement_.join();
    }

    void configureMock() {
        ON_CALL(*this, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) return false;
                stopThreads_ = false;
                measurement_ = std::thread(
                    &crf::sensors::rpsensor::RPSensorMockConfiguration::rpMeasures, this);
                initialized_ = true;
                return true;
            }));
        ON_CALL(*this, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                stopThreads_ = true;
                if (measurement_.joinable()) measurement_.join();
                initialized_ = false;
                return true;
            }));

        ON_CALL(*this, getDoseRate()).WillByDefault(Invoke(
            [this] () -> std::optional<float> {
                if (!initialized_) return std::nullopt;
                return dose_;
            }));

        ON_CALL(*this, getCumulativeDose()).WillByDefault(Invoke(
            [this] () -> std::optional<float> {
                if (!initialized_) return std::nullopt;
                return cumulativeDose_;
            }));

        ON_CALL(*this, resetCumulativeDose()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                cumulativeDose_ = 0;
                return true;
            }));
    }

 private:
    bool initialized_;
    std::atomic<float> dose_;
    std::atomic<float> cumulativeDose_;
    std::atomic<bool> stopThreads_;

    std::thread measurement_;

    void rpMeasures() {
        while (!stopThreads_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::random_device ran;
            std::mt19937 mt(ran());
            std::uniform_real_distribution<float> dist(0, 0.05);
            dose_ = dist(mt);
            cumulativeDose_ = cumulativeDose_ + dose_;
        }
    }
};

}  // namespace rpsensor
}  // namespace sensors
}  // namespace crf
