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
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "RobotArmController/RobotArmControllerMock.hpp"
#include "Types/Types.hpp"

#include "EventLogger/EventLogger.hpp"

namespace crf::control::robotarmcontroller {

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::Invoke;
using ::testing::_;
using ::testing::An;

class RobotArmControllerMockConfiguration {
 public:
    RobotArmControllerMockConfiguration() = delete;
    explicit RobotArmControllerMockConfiguration(int numberJoints):
        numberJoints_(numberJoints),
        currentJointPos_(numberJoints_),
        currentJointVel_(numberJoints_),
        currentJointAcc_(numberJoints_),
        initialized_(false),
        moving_(false),
        stop_(false),
        logger_("RobotArmControllerMockConfiguration") {
            robotArmControllerMock_.reset(new NiceMock<RobotArmControllerMock>());
    }
    ~RobotArmControllerMockConfiguration() {
        moving_ = false;
        stop_ = true;
    }

    std::shared_ptr<NiceMock<RobotArmControllerMock>> getMock() {
        return robotArmControllerMock_;
    }

    void setPosition(const crf::utility::types::JointPositions& pos) {
        currentJointPos_ = pos;
    }

    void configureRobotArmControllerMock() {
        ON_CALL(*robotArmControllerMock_, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) return false;
                moving_ = false;
                initialized_ = true;
                return true;
            }));

        ON_CALL(*robotArmControllerMock_, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                moving_ = false;
                stop_ = true;
                initialized_ = false;
                return true;
            }));

        ON_CALL(*robotArmControllerMock_, setPosition(
            An<const crf::utility::types::JointPositions&>())).WillByDefault(Invoke(
            [this](const crf::utility::types::JointPositions& position) {
                if (!initialized_) return std::future<bool>();
                stop_ = false;
                std::vector<crf::utility::types::JointPositions> goal;
                goal.push_back(position);
                return std::async(std::launch::async,
                    [this, goal]() {
                        return moveToPosition(goal);
                    });
            }));

        ON_CALL(*robotArmControllerMock_, setPosition(
            An<const std::vector<crf::utility::types::JointPositions>&>())).WillByDefault(Invoke(
            [this](const std::vector<crf::utility::types::JointPositions>& positions) {
                if (!initialized_) return std::future<bool>();
                stop_ = false;
                return std::async(std::launch::async,
                    [this, positions]() {
                        return moveToPosition(positions);
                    });
            }));

        ON_CALL(*robotArmControllerMock_, interruptTrajectory()).WillByDefault(Invoke(
            [this] {
                stop_ = true;
                return true;
            }));

        ON_CALL(*robotArmControllerMock_, getJointPositions()).WillByDefault(Invoke(
            [this] {
                return currentJointPos_;
            }));

        ON_CALL(*robotArmControllerMock_, getJointVelocities()).WillByDefault(Invoke(
            [this] {
                return currentJointVel_;
            }));

        ON_CALL(*robotArmControllerMock_, getJointAccelerations()).WillByDefault(Invoke(
            [this] {
                return currentJointAcc_;
            }));

        ON_CALL(*robotArmControllerMock_, getTaskPose()).WillByDefault(Invoke(
            [this] {
                return currentTaskPos_;
            }));

        ON_CALL(*robotArmControllerMock_, getTaskVelocity()).WillByDefault(Invoke(
            [this] {
                return currentTaskVel_;
            }));

        ON_CALL(*robotArmControllerMock_, getTaskAcceleration()).WillByDefault(Invoke(
            [this] {
                return currentTaskAcc_;
            }));

        ON_CALL(*robotArmControllerMock_, setJointsMaximumVelocity(_)).WillByDefault(Return(true));
    }

 private:
    int numberJoints_;

    crf::utility::types::JointPositions currentJointPos_;
    crf::utility::types::JointVelocities currentJointVel_;
    crf::utility::types::JointAccelerations currentJointAcc_;

    crf::utility::types::TaskPose currentTaskPos_;
    crf::utility::types::TaskVelocity currentTaskVel_;
    crf::utility::types::TaskAcceleration currentTaskAcc_;

    std::shared_ptr<NiceMock<RobotArmControllerMock>> robotArmControllerMock_;

    bool initialized_ = false;;
    bool moving_ = false;
    std::atomic<bool> stop_ = false;

    crf::utility::logger::EventLogger logger_;

    bool moveToPosition(
        const std::vector<crf::utility::types::JointPositions>& goalPositionsJoints) {
        for (uint8_t i = 0; i < numberJoints_; i++) {
            currentJointVel_[i] = 0.3;
        }
        for (uint8_t j = 0; j < goalPositionsJoints.size(); j++) {
            auto target = goalPositionsJoints.at(j);
            float maxTime = 0;
            int idx = 0;
            for (uint8_t i = 0; i < numberJoints_; i++) {
                float time = (target[i] - currentJointPos_[i]) / currentJointVel_[i];
                if (std::fabs(time) > maxTime) {
                    maxTime = std::fabs(time);
                    idx = i;
                }
            }
            if (maxTime <= 0.1 && j == goalPositionsJoints.size()-1) {
                moving_ = false;
                return true;
            }
            if (maxTime <= 0.1) continue;
            logger_->info("Goal is {}, to reach in {}", target, maxTime);
            for (uint8_t i = 0; i < numberJoints_; i++) {
                float time = (target[i] - currentJointPos_[i]) / currentJointVel_[i];
                currentJointVel_[i] = currentJointVel_[i] * (time/maxTime);
            }
            stop_ = false;
            moving_ = true;
            while (moving_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                logger_->debug("Current position {}", currentJointPos_);
                for (uint8_t i = 0; i < numberJoints_; i++) {
                    currentJointPos_[i] = currentJointPos_[i] + currentJointVel_[i] * 0.1;
                }
                if (stop_) {
                    logger_->warn("Trajectory interrupted");
                    return false;
                }
                if (crf::utility::types::areAlmostEqual(currentJointPos_, target, 0.015)) {
                    moving_ = false;
                    logger_->info("Position reached");
                }
            }
        }
        return true;
    }
};

}  // namespace crf::control::robotarmcontroller
