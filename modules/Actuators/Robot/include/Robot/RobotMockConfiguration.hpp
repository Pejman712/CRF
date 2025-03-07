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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Robot/RobotMock.hpp"
#include "Robot/RobotConfiguration.hpp"

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

using crf::actuators::robot::RobotMock;

namespace crf::actuators::robot {

class RobotMockConfiguration : public RobotMock {
 public:
    RobotMockConfiguration() = delete;
    explicit RobotMockConfiguration(int numberJoints, std::string configFilePath =
        "cpproboticframework/modules/Actuators/Robot/config/UniversalRobot/UR10e.json"):
        numberJoints_(numberJoints),
        mockConfigured_(false),
        currentJointPos_(numberJoints_),
        currentJointVel_(numberJoints_),
        currentJointAcc_(numberJoints_),
        currentJointTrq_(numberJoints_),
        defaultVelocity_(numberJoints_),
        profileJointVel_(numberJoints_),
        profileJointAcc_(numberJoints_),
        currentTaskPos_(Eigen::Vector3d({0, 0, 0}),
                        crf::math::rotation::CardanXYZ({1, 0, 0})),
        currentBrakeStatus_(numberJoints_),
        initialized_(false),
        stop_(false),
        logger_("RobotMockConfiguration") {
        for (uint16_t i = 0; i < numberJoints_; i++) {
            currentJointPos_[i] = 0.0;
            currentJointVel_[i] = 0.0;
            currentJointAcc_[i] = 0.0;
            currentJointTrq_[i] = 0.0;
            defaultVelocity_[i] = defaultJointVelocities_;
        }
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("cpproboticframework"));
        std::ifstream robotData(testDirName + configFilePath);
        robotConfiguration_ = std::make_shared<RobotConfiguration>(
            nlohmann::json::parse(robotData));
        controlLoopTime_ = robotConfiguration_->getRobotControllerLoopTime();

        robot_.reset(new NiceMock<RobotMock>());
    }

    ~RobotMockConfiguration() {
        logger_->info("DTor");
    }

    std::atomic<bool> getTaskObjectsImplemented {false};

    void setPosition(const crf::utility::types::JointPositions& pos) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        currentJointPos_ = pos;
    }

    void setControlLoopTimeMs(std::chrono::milliseconds loopTime) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        controlLoopTime_ = loopTime;
    }

    static bool returnSimulatedSetGravityResponse(std::array<double, 3> g) {
        double gravityMagnitude = sqrt(std::pow(g[0], 2) + std::pow(g[1], 2) + std::pow(g[2], 2));
        if (std::abs(gravityMagnitude - 9.81) > 0.1) {
            return false;
        }
        return true;
    }

    void configureMock() {
        mockConfigured_ = true;
        ON_CALL(*this, initialize()).WillByDefault(Invoke([this] {
            if (initialized_) return false;
            stop_ = false;
            initialized_ = true;
            return true;
        }));
        ON_CALL(*this, deinitialize()).WillByDefault(Invoke([this] {
            if (!initialized_) return false;
            stop_ = true;
            initialized_ = false;
            return true;
        }));

        ON_CALL(*this, getJointPositions()).WillByDefault(Invoke([this] {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return currentJointPos_;
        }));
        ON_CALL(*this, getJointVelocities()).WillByDefault(Invoke([this] {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return currentJointVel_;
            }));
        ON_CALL(*this, getJointAccelerations()).WillByDefault(Invoke([this] {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return currentJointAcc_;
        }));
        ON_CALL(*this, getJointForceTorques()).WillByDefault(Invoke([this] {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return currentJointTrq_;
        }));

        ON_CALL(*this, getTaskPose()).WillByDefault(Invoke(
            [this] () -> crf::expected<TaskPose> {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            if (getTaskObjectsImplemented) {
                return currentTaskPos_;
            }
            return crf::Code::MethodNotAllowed;
        }));

        ON_CALL(*this, getTaskVelocity()).WillByDefault(Invoke(
            [this] () -> crf::expected<TaskVelocity> {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            if (getTaskObjectsImplemented) {
                return currentTaskVel_;
            }
            return crf::Code::MethodNotAllowed;
        }));
        ON_CALL(*this, getTaskAcceleration()).WillByDefault(Invoke(
            [this] () -> crf::expected<TaskAcceleration> {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            if (getTaskObjectsImplemented) {
                return currentTaskAcc_;
            }
            return crf::Code::MethodNotAllowed;
        }));
        ON_CALL(*this, getTaskForceTorque()).WillByDefault(Invoke(
            [this] () -> crf::expected<TaskForceTorque> {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            if (getTaskObjectsImplemented) {
                return currentTaskTrq_;
            }
            return crf::Code::MethodNotAllowed;
        }));

        ON_CALL(*this, setJointPositions(_, _, _, _)).WillByDefault(Invoke([this](
            const bool& isSmoothTrajectory,
            const utility::types::JointPositions& jointPositions,
            const utility::types::JointVelocities& jointVelocities,
            const utility::types::JointAccelerations& jointAccelerations)->expected<bool> {
            if (!initialized_) return false;
            std::unique_lock<std::shared_mutex> lock(mutex_);
            if (!jointPositions.size() == numberJoints_) return false;
            currentJointPos_ = jointPositions;
            return true;
        }));

        ON_CALL(*this, setJointVelocities(_, _, _)).WillByDefault(Invoke([this](
            const bool& isSmoothTrajectory,
            const utility::types::JointVelocities& jointVelocities,
            const utility::types::JointAccelerations& jointAccelerations)->expected<bool> {
            if (!initialized_) return false;
            stop_ = false;
            std::unique_lock<std::shared_mutex> lock(mutex_);
            for (int i = 0; i < numberJoints_; i++) {
                currentJointPos_[i] += jointVelocities[i]*controlLoopTime_.count()/1000;
            }
            logger_->debug("Current pos: {}", currentJointPos_);
            currentJointVel_ = jointVelocities;
            return true;
        }));

        ON_CALL(*this, setJointForceTorques(_, _)).WillByDefault(Invoke([this](
            const bool& isSmoothTrajectory,
            const crf::utility::types::JointForceTorques& jointForceTorques)->expected<bool> {
            if (!initialized_) return false;
            stop_ = false;
            std::unique_lock<std::shared_mutex> lock(mutex_);
            logger_->debug("Current pos: {}", currentJointTrq_);
            currentJointTrq_ = jointForceTorques;
            return true;
        }));

        ON_CALL(*this, setProfileJointVelocities(_)).WillByDefault(Invoke([this](
            const utility::types::JointVelocities& jointVelocities)->expected<bool> {
            if (!initialized_) return false;
            stop_ = false;
            std::unique_lock<std::shared_mutex> lock(mutex_);
            logger_->debug("Current pos: {}", profileJointVel_);
            profileJointVel_ = jointVelocities;
            return true;
        }));

        ON_CALL(*this, getProfileJointVelocities()).WillByDefault(Invoke([this] {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return profileJointVel_;
        }));

        ON_CALL(*this, setProfileJointAccelerations(_)).WillByDefault(Invoke([this](
            const utility::types::JointAccelerations& jointAccelerations)->expected<bool> {
            if (!initialized_) return false;
            stop_ = false;
            std::unique_lock<std::shared_mutex> lock(mutex_);
            logger_->debug("Current pos: {}", profileJointAcc_);
            profileJointAcc_ = jointAccelerations;
            return true;
        }));

        ON_CALL(*this, getProfileJointAccelerations()).WillByDefault(Invoke([this] {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return profileJointAcc_;
        }));

        ON_CALL(*this, setGravity(_)).
            WillByDefault(Invoke(returnSimulatedSetGravityResponse));

        ON_CALL(*this, softStop()).WillByDefault(Invoke([this] {
            stop_ = true;
            return true;
        }));
        ON_CALL(*this, hardStop()).WillByDefault(Invoke([this] {
            stop_ = true;
            return true;
        }));

        ON_CALL(*this, setBrakes(_)).WillByDefault(Invoke([this](
            const std::vector<bool>& brakesStatus)->expected<bool> {
            if (!initialized_) return false;
            stop_ = false;
            std::unique_lock<std::shared_mutex> lock(mutex_);
            currentBrakeStatus_ = brakesStatus;
            return true;
        }));

        ON_CALL(*this, getBrakes()).WillByDefault(Invoke([this] {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return currentBrakeStatus_;
        }));

        ON_CALL(*this, getConfiguration()).WillByDefault(Invoke([this] {
            return robotConfiguration_;
        }));
    }

 private:
    int numberJoints_;
    std::atomic<bool> mockConfigured_;

    crf::utility::types::JointPositions currentJointPos_;
    crf::utility::types::JointVelocities currentJointVel_;
    crf::utility::types::JointAccelerations currentJointAcc_;
    crf::utility::types::JointForceTorques currentJointTrq_;
    crf::utility::types::TaskPose currentTaskPos_;
    crf::utility::types::TaskVelocity currentTaskVel_;
    crf::utility::types::TaskAcceleration currentTaskAcc_;
    crf::utility::types::TaskForceTorque currentTaskTrq_;

    crf::utility::types::JointVelocities profileJointVel_;
    crf::utility::types::JointAccelerations profileJointAcc_;

    std::shared_mutex mutex_;

    std::vector<bool> currentBrakeStatus_;

    crf::utility::types::JointVelocities defaultVelocity_;
    std::chrono::milliseconds controlLoopTime_;

    std::shared_ptr<NiceMock<RobotMock>> robot_;

    std::atomic<bool> initialized_;
    std::atomic<bool> stop_;

    crf::utility::logger::EventLogger logger_;

    std::shared_ptr<RobotConfiguration> robotConfiguration_;

    const float defaultJointVelocities_ = 0.3;  // rad/s
};

}  // namespace crf::actuators::robot
