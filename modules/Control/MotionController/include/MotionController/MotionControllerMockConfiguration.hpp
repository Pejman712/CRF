/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <shared_mutex>
#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "MotionController/MotionControllerMock.hpp"

#include "Types/Types.hpp"

#include "EventLogger/EventLogger.hpp"
#include "crf/expected.hpp"

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

namespace crf::control::motioncontroller {

class MotionControllerMockConfiguration : public MotionControllerMock {
 public:
    MotionControllerMockConfiguration() = delete;
    explicit MotionControllerMockConfiguration(int numberJoints):
        numberJoints_(numberJoints),
        currentJointPos_(numberJoints_),
        currentJointVel_(numberJoints_),
        currentJointAcc_(numberJoints_),
        currentJointTrq_(numberJoints_),
        moving_(false),
        stop_(false),
        logger_("MotionControllerMockConfiguration") {
        for (uint16_t i = 0; i < numberJoints_; i++) {
            // Random starting position
            currentJointPos_[i] = 0.3;
            currentJointVel_[i] = 0.7;
            currentJointAcc_[i] = 3.0;
            currentJointTrq_[i] = 1.4;
        }
    }
    ~MotionControllerMockConfiguration() {
        logger_->info("DTor");
    }

    void setInitialPosition(const crf::utility::types::JointPositions& pos) {
        currentJointPos_ = pos;
    }

    void configureMock() {
        ON_CALL(*this, initialize()).WillByDefault(Invoke(
            [this] {
                return true;
            }));
        ON_CALL(*this, deinitialize()).WillByDefault(Invoke(
            [this] {
                return true;
            }));
        ON_CALL(*this, appendPath(_)).WillByDefault(Invoke(
            [this](const std::vector<JointPositions>& path) {
                for (int i = 0; i < path.size(); i++) {
                    std::vector<crf::utility::types::JointPositions> goal;
                    goal.push_back(path[i]);
                    if (!moveToPosition(goal)) {
                        return false;
                    }
                }
                return true;
            }));
        ON_CALL(*this, appendPath(_, _, _)).WillByDefault(Return(true));
        ON_CALL(*this, setVelocity(_)).WillByDefault(Return(true));
        ON_CALL(*this, setVelocity(_, _)).WillByDefault(Return(true));
        ON_CALL(*this, setTorque(_)).WillByDefault(Return(true));
        ON_CALL(*this, setTorque(_, _)).WillByDefault(Return(true));
        ON_CALL(*this, setProfileVelocity(
            An<const crf::utility::types::JointVelocities&>())).WillByDefault(Return(true));
        ON_CALL(*this, setProfileVelocity(
            An<const crf::utility::types::TaskVelocity&>())).WillByDefault(Return(true));
        ON_CALL(*this, setProfileAcceleration(
            An<const crf::utility::types::JointAccelerations&>())).WillByDefault(Return(true));
        ON_CALL(*this, setProfileAcceleration(
            An<const crf::utility::types::TaskAcceleration&>())).WillByDefault(Return(true));
        ON_CALL(*this, softStop()).WillByDefault(Invoke(
            [this] {
                return;
            }));
        ON_CALL(*this, hardStop()).WillByDefault(Invoke(
            [this] {
                return;
            }));
        ON_CALL(*this, setParameters(_)).WillByDefault(Return(true));
        ON_CALL(*this, getCurrentParameters()).WillByDefault(Invoke(
            [this] {
                nlohmann::json json;
                return json;
            }));
        ON_CALL(*this, getParametersDefinition()).WillByDefault(Invoke(
            [this] {
                nlohmann::json json;
                return json;
            }));
        ON_CALL(*this, getSignals()).WillByDefault(Invoke(
            [this] () -> crf::utility::types::Signals {
                crf::utility::types::Signals sig;
                sig.joints.positions = currentJointPos_;
                sig.joints.velocities = currentJointVel_;
                sig.joints.accelerations = currentJointAcc_;
                sig.joints.forceTorques = currentJointTrq_;
                return sig;
            }));
        ON_CALL(*this, isTrajectoryRunning()).WillByDefault(Return(moving_));
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
    bool moving_;
    std::atomic<bool> stop_;
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<MotionControllerMock>> motionControllerMock_;

    bool moveToPosition(const std::vector<crf::utility::types::JointPositions>& positions) {
        for (uint8_t i = 0; i < numberJoints_; i++) {
            currentJointVel_[i] = 0.3;
        }
        for (uint8_t j = 0; j < positions.size(); j++) {
            auto target = positions.at(j);
            float maxTime = 0;
            int idx = 0;
            for (uint8_t i = 0; i < numberJoints_; i++) {
                float time = (target[i] - currentJointPos_[i]) / currentJointVel_[i];
                if (std::fabs(time) > maxTime) {
                    maxTime = std::fabs(time);
                    idx = i;
                }
            }
            if (maxTime <= 0.1 && j == positions.size()-1) {
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

}  // namespace crf::control::motioncontroller
