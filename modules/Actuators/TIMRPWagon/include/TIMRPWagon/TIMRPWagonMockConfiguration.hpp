/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/
#pragma once

#include <thread>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <optional>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "TIMRPWagon/TIMRPWagonMock.hpp"

namespace crf::actuators::timrpwagon {

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::_;

class TIMRPWagonMockConfiguration {
 public:
    TIMRPWagonMockConfiguration():
        rpWagonMock_(new NiceMock<TIMRPWagonMock>()),
        initialized_(false),
        timeout_(10),
        percent_(0) {
    }

    ~TIMRPWagonMockConfiguration() {
        cv_.notify_all();
    }

    std::shared_ptr<NiceMock<TIMRPWagonMock>> getMock() {
        return rpWagonMock_;
    }

    void setMovementTime(std::chrono::milliseconds time) {
        timeout_ = time;
    }

    void setPosition(RPArmPosition position) {
        if (position == RPArmPosition::Deployed) percent_ = 0;
        if (position == RPArmPosition::Retracted) percent_ = 10;
        if (position == RPArmPosition::InTheMiddle) percent_ = 5;
    }

    void configureTIMRPWagonMock() {
        ON_CALL(*rpWagonMock_, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) {
                    return false;
                }
                initialized_ = true;
                return true;
            }));

        ON_CALL(*rpWagonMock_, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) {
                    return false;
                }
                initialized_ = false;
                return true;
            }));

        ON_CALL(*rpWagonMock_, isConnected()).WillByDefault(Return(true));

        ON_CALL(*rpWagonMock_, getRPArmPosition()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) {
                    return RPArmPosition::NotDefined;
                }
                if (percent_ == 0) return RPArmPosition::Deployed;
                if (percent_ == 10) return RPArmPosition::Retracted;
                return RPArmPosition::InTheMiddle;
            }));

        ON_CALL(*rpWagonMock_, retractRPArm()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                if (percent_ == 10) return true;
                percent_ = 5;
                std::unique_lock<std::mutex> lck(mtx_);
                if (cv_.wait_for(lck, timeout_) != std::cv_status::timeout) {
                    return false;
                }
                percent_ = 10;
                return true;
            }));

        ON_CALL(*rpWagonMock_, deployRPArm()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                if (percent_ == 0) return true;
                percent_ = 5;
                std::unique_lock<std::mutex> lck(mtx_);
                if (cv_.wait_for(lck, timeout_) != std::cv_status::timeout) {
                    return false;
                }
                percent_ = 0;
                return true;
            }));

        ON_CALL(*rpWagonMock_, moveRPArmUp()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                if (percent_ == 10) return true;
                std::unique_lock<std::mutex> lck(mtx_);
                if (cv_.wait_for(lck, timeout_/10) != std::cv_status::timeout) {
                    return false;
                }
                percent_++;
                return true;
            }));

        ON_CALL(*rpWagonMock_, moveRPArmDown()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                if (percent_ == 0) return true;
                std::unique_lock<std::mutex> lck(mtx_);
                if (cv_.wait_for(lck, timeout_/10) != std::cv_status::timeout) {
                    return false;
                }
                percent_--;
                return true;
            }));

        ON_CALL(*rpWagonMock_, stopRPArm()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                cv_.notify_one();
                return true;
            }));

        ON_CALL(*rpWagonMock_, lockRPArm()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                return true;
            }));

        ON_CALL(*rpWagonMock_, unlockRPArm()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                return true;
            }));

        ON_CALL(*rpWagonMock_, isRPArmInError()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                return false;
            }));

        ON_CALL(*rpWagonMock_, resetRPArmDriver()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                return true;
            }));

        ON_CALL(*rpWagonMock_, acknowledgeErrors()).WillByDefault(Invoke(
            [this]() -> std::optional<bool> {
                if (!initialized_) {
                    return std::nullopt;
                }
                return true;
            }));

        // TODO(jplayang): Return a properly filled configuration
        ON_CALL(*rpWagonMock_, getConfiguration()).WillByDefault(Return(config_));
    }

 private:
    std::shared_ptr<NiceMock<TIMRPWagonMock>> rpWagonMock_;
    bool initialized_;
    std::chrono::milliseconds timeout_;
    // If percent is 0 then it's deployed, and if it's 10 it's retracted
    std::atomic<uint8_t> percent_;
    std::shared_ptr<TIMRPWagonConfiguration> config_;
    std::condition_variable cv_;
    std::mutex mtx_;
};

}  // namespace crf::actuators::timrpwagon
