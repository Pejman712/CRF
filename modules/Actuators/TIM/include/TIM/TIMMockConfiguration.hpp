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
#include <mutex>
#include <vector>
#include <limits>
#include <set>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "TIM/TIMMock.hpp"
#include "TIM/LHCObstacle.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::tim {

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::_;

class TIMMockConfiguration {
 public:
    TIMMockConfiguration():
        logger_("TIMMockConfiguration"),
        currentPos_(0),
        velocity_(0),
        initialized_(false),
        economyMode_(false),
        moving_(false),
        devicesRetracted_(false),
        obstacleMtx_() {
            timMock_.reset(new NiceMock<TIMMock>());
    }
    ~TIMMockConfiguration() {
        moving_ = false;
        if (movement_.joinable()) movement_.join();
    }

    std::shared_ptr<NiceMock<TIMMock>> getMock() {
        return timMock_;
    }

    void setTIMStartingPosition(const float& position) {
        currentPos_ = position;
    }

    void TIMAddObstacle(const crf::actuators::tim::LHCObstacle& obstacle) {
        obstaclesList_.insert(obstacle);
    }

    void configureTIMMock() {
        ON_CALL(*timMock_, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) {
                    return false;
                }
                std::scoped_lock<std::mutex> lck(obstacleMtx_);
                obstaclesList_.insert(LHCObstacle(0, LHCObstacleType::OuterLimits,
                    0, 0, 0.3, true));
                obstaclesList_.insert(LHCObstacle(std::numeric_limits<int8_t>::max(),
                    LHCObstacleType::OuterLimits, std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max(), 0.3, true));
                initialized_ = true;
                return true;
            }));
        ON_CALL(*timMock_, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) {
                    return false;
                }
                moving_ = false;
                if (movement_.joinable()) {
                    movement_.join();
                }
                initialized_ = false;
                return true;
            }));
        ON_CALL(*timMock_, isConnected()).WillByDefault(Return(true));
        ON_CALL(*timMock_, moveToTarget(_, _)).WillByDefault(Invoke(
            [this](float dcum, float vel) {
                if (!initialized_) {
                    return false;
                }
                finalPoint_ = dcum;
                if (finalPoint_ < currentPos_) {
                    velocity_ = std::fabs(vel)*-1;
                } else {
                    velocity_ = std::fabs(vel);
                }
                if (movement_.joinable()) {
                    moving_ = false;
                    movement_.join();
                }
                moving_ = true;
                movement_ = std::thread(&TIMMockConfiguration::moveTrain, this);
                return true;
            }));
        ON_CALL(*timMock_, emergencyStop()).WillByDefault(Invoke(
            [this] {
                stop_ = true;
                moving_ = false;
                return true;
            }));
        ON_CALL(*timMock_, stop()).WillByDefault(Invoke(
            [this] {
                stop_ = true;
                moving_ = false;
                return true;
            }));
        ON_CALL(*timMock_, startCharging()).WillByDefault(Return(true));
        ON_CALL(*timMock_, stopCharging()).WillByDefault(Return(true));
        ON_CALL(*timMock_, setTargetVelocity(_)).WillByDefault(Invoke(
            [this] (const float &velocity) {
                if (finalPoint_ < currentPos_) {
                    velocity_ = std::fabs(velocity)*-1;
                } else {
                    velocity_ = std::fabs(velocity);
                }
                logger_->info("Velocity set to {}", velocity_);
                return true;
            }));
        ON_CALL(*timMock_, getCurrentPosition()).WillByDefault(Invoke(
            [this] {
                return currentPos_.load();
            }));
        ON_CALL(*timMock_, getCurrentVelocity()).WillByDefault(Invoke(
            [this] {
                return velocity_.load();
            }));
        ON_CALL(*timMock_, devicesRetracted(_)).WillByDefault(Invoke(
            [this] (bool deviceStatus) {
                devicesRetracted_ = deviceStatus;
                return true;
            }));
        ON_CALL(*timMock_, devicesRetracted()).WillByDefault(Invoke(
            [this] {
                return devicesRetracted_.load();
            }));
        ON_CALL(*timMock_, enableEconomyMode()).WillByDefault(Invoke(
            [this] {
                economyMode_ = true;
                return true;
            }));
        ON_CALL(*timMock_, disableEconomyMode()).WillByDefault(Invoke(
            [this] {
                economyMode_ = false;
                return true;
            }));
        ON_CALL(*timMock_, isMoving()).WillByDefault(Invoke(
            [this] {
                return moving_.load();
            }));
        ON_CALL(*timMock_, getBatteryVoltage()).WillByDefault(Return(batteryLevel_));
        ON_CALL(*timMock_, getCurrentObstacleArea()).WillByDefault(Invoke(
            [this] {
                std::scoped_lock<std::mutex> lck(obstacleMtx_);
                for (const LHCObstacle &obstacle : obstaclesList_) {
                    if (currentPos_ >= obstacle.startPosition() &&
                        currentPos_ <= obstacle.endPosition()) {
                        return obstacle;
                    }
                }
                return crf::actuators::tim::LHCObstacle();
            }));
        ON_CALL(*timMock_, getClosestObstacleAreas()).WillByDefault(Invoke(
            [this] {
                crf::actuators::tim::LHCObstacle obstacleBehind;
                crf::actuators::tim::LHCObstacle obstacleInFront;
                std::scoped_lock<std::mutex> lck(obstacleMtx_);
                for (const LHCObstacle &obstacle : obstaclesList_) {
                    if (currentPos_ > obstacle.endPosition()) {
                        obstacleBehind = obstacle;
                    }
                    if (currentPos_ < obstacle.startPosition()) {
                        obstacleInFront = obstacle;
                        break;
                    }
                }
                if (velocity_ < 0) {
                    return std::array<LHCObstacle, 2>{obstacleInFront, obstacleBehind};
                }
                return std::array<LHCObstacle, 2>{obstacleBehind, obstacleInFront};
            }));
    }

 private:
    crf::utility::logger::EventLogger logger_;
    std::atomic<float> currentPos_;
    std::atomic<float> velocity_;
    std::atomic<float> finalPoint_;
    std::atomic<bool> stop_;
    std::atomic<bool> economyMode_;
    std::atomic<bool> moving_;
    std::atomic<bool> devicesRetracted_;
    bool initialized_;
    std::mutex obstacleMtx_;
    std::set<LHCObstacle, CompareLHCObstacle> obstaclesList_;
    std::thread movement_;
    const float batteryLevel_ = 26;
    const float timMovementThreshold_ = 0.15;
    const float obstacleThreshold_ = 10;

    std::shared_ptr<NiceMock<TIMMock>> timMock_;

    // TODO(adiazros): Stop train when it goes in obstacle area and devices are not retracted.
    void moveTrain() {
        logger_->debug("moveTrain");
        stop_ = false;
        moving_ = true;
        while (moving_) {
            std::mutex mtx;
            std::unique_lock lk(mtx);
            std::condition_variable cv;
            cv.wait_for(lk, std::chrono::milliseconds(static_cast<int>(100/std::fabs(velocity_))));
            logger_->debug("Current train position is {}", currentPos_);
            currentPos_ = currentPos_ + 0.1 * (velocity_/std::fabs(velocity_));
            if (stop_ || economyMode_) break;
            if (std::fabs(currentPos_-finalPoint_) < timMovementThreshold_) {
                moving_ = false;
                velocity_ = 0;
            }
        }
    }
};

}  // namespace crf::actuators::tim
