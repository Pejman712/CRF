/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
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

#include "RobotArm/RobotArmMock.hpp"

#include "Types/Types.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::An;

using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointForceTorques;
using crf::utility::types::areAlmostEqual;

using crf::actuators::robotarm::RobotArmMock;

namespace crf::actuators::robotarm {

class RobotArmMockConfiguration {
 public:
    RobotArmMockConfiguration() = delete;
    explicit RobotArmMockConfiguration(int numberJoints):
        numberJoints_(numberJoints),
        currentJointPos_(numberJoints_),
        currentJointVel_(numberJoints_),
        currentJointAcc_(numberJoints_),
        currentJointTrq_(numberJoints_),
        defaultVelocity_(numberJoints_),
        initialized_(false),
        stop_(false),
        logger_("RobotArmMockConfiguration") {
            for (uint16_t i = 0; i < numberJoints_; i++) {
                defaultVelocity_[i] = defaultJointVelocities_;
            }
            std::string testDirName = __FILE__;
            testDirName = testDirName.substr(0,
                testDirName.find("cpproboticframework"));

            // Random config file to return
            robotArmConfiguration_.reset(new RobotArmConfiguration);
            std::ifstream robotData(testDirName +
                "cpproboticframework/modules/Actuators/TIMArm/config/TIM81RobotArm.json");
            nlohmann::json robotJSON = nlohmann::json::parse(robotData);
            robotArmConfiguration_->parse(robotJSON);
            controlLoopTime_ = robotArmConfiguration_->getRTLoopTime();

            robotArmMock_.reset(new NiceMock<RobotArmMock>());
        }
    ~RobotArmMockConfiguration() = default;

    std::shared_ptr<NiceMock<RobotArmMock>> getMock() {
        return robotArmMock_;
    }

    void setPosition(const crf::utility::types::JointPositions& pos) {
        currentJointPos_ = pos;
    }

    void setControlLoopTimeMs(std::chrono::milliseconds loopTime) {
        controlLoopTime_ = loopTime;
    }

    void configureRobotArmMock() {
        ON_CALL(*robotArmMock_, initialize()).WillByDefault(Invoke(
            [this] {
                if (initialized_) return false;
                stop_ = false;
                initialized_ = true;
                return true;
            }));
        ON_CALL(*robotArmMock_, deinitialize()).WillByDefault(Invoke(
            [this] {
                if (!initialized_) return false;
                stop_ = true;
                initialized_ = false;
                return true;
            }));

        ON_CALL(*robotArmMock_, setJointPositions(_)).WillByDefault(Invoke(
            [this](const crf::utility::types::JointPositions& position) {
                if (!initialized_) return false;
                stop_ = false;
                float maxTime = 0;
                int idx = 0;
                for (uint8_t i = 0; i < numberJoints_; i++) {
                    float time = (position[i] - currentJointPos_[i]) / defaultVelocity_[i];
                    if (std::fabs(time) > maxTime) {
                        maxTime = std::fabs(time);
                        idx = i;
                    }
                }
                if (maxTime <= 0.1) return true;
                logger_->info("Goal is {}, to reach in {}", position, maxTime);
                for (uint8_t i = 0; i < numberJoints_; i++) {
                    float time = (position[i] - currentJointPos_[i]) / defaultVelocity_[i];
                    defaultVelocity_[i] = defaultVelocity_[i] * (time/maxTime);
                }
                stop_ = false;
                bool moving = true;
                while (moving) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    logger_->debug("Current position {}", currentJointPos_);
                    for (uint8_t i = 0; i < numberJoints_; i++) {
                        currentJointPos_[i] = currentJointPos_[i] + defaultVelocity_[i] * 0.1;
                    }
                    if (stop_) {
                        logger_->warn("Trajectory interrupted");
                        return false;
                    }
                    if (crf::utility::types::areAlmostEqual(currentJointPos_, position, 0.015)) {
                        moving = false;
                        logger_->info("Position reached");
                    }
                }
                return true;
            }));

        ON_CALL(*robotArmMock_, setJointVelocities(_)).WillByDefault(Invoke(
            [this](const crf::utility::types::JointVelocities& jointVelocities) {
                if (!initialized_) return false;
                logger_->debug("Current position {}", currentJointPos_);
                for (int i = 0; i < numberJoints_; i++) {
                    currentJointPos_[i] += jointVelocities[i]*controlLoopTime_.count();
                }
                currentJointVel_ = jointVelocities;
                return true;
            }));

        ON_CALL(*robotArmMock_, stopArm()).WillByDefault(Invoke(
            [this] {stop_ = true; return true;}));

        ON_CALL(*robotArmMock_, getJointPositions()).WillByDefault(Invoke(
            [this] {return currentJointPos_;}));
        ON_CALL(*robotArmMock_, getJointVelocities()).WillByDefault(Invoke(
            [this] {return currentJointVel_;}));
        ON_CALL(*robotArmMock_, getJointForceTorques()).WillByDefault(Invoke(
            [this] {return currentJointTrq_;}));

        ON_CALL(*robotArmMock_, getTaskPose()).WillByDefault(Invoke(
            [this] {return currentTaskPos_;}));
        ON_CALL(*robotArmMock_, getTaskVelocity()).WillByDefault(Invoke(
            [this] {return currentTaskVel_;}));

        ON_CALL(*robotArmMock_, getConfiguration()).WillByDefault(Invoke(
            [this] {return robotArmConfiguration_;}));

        ON_CALL(*robotArmMock_, enableBrakes()).WillByDefault(Invoke(
            [this] {return true;}));

        ON_CALL(*robotArmMock_, disableBrakes()).WillByDefault(Invoke(
            [this] {return true;}));
    }

 private:
    int numberJoints_;

    crf::utility::types::JointPositions currentJointPos_;
    crf::utility::types::JointVelocities currentJointVel_;
    crf::utility::types::JointAccelerations currentJointAcc_;
    crf::utility::types::JointForceTorques currentJointTrq_;
    crf::utility::types::TaskPose currentTaskPos_;
    crf::utility::types::TaskVelocity currentTaskVel_;
    crf::utility::types::TaskAcceleration currentTaskAcc_;
    crf::utility::types::TaskForceTorque currentTaskTrq_;

    crf::utility::types::JointVelocities defaultVelocity_;
    std::chrono::milliseconds controlLoopTime_;

    std::shared_ptr<NiceMock<RobotArmMock>> robotArmMock_;

    bool initialized_;
    bool stop_;

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<RobotArmConfiguration> robotArmConfiguration_;

    const float defaultJointVelocities_ = 0.3;  // rad/s
};

}  // namespace crf::actuators::robotarm
