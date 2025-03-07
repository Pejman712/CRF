/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>
#include <set>

#include "Robot/IRobot.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::An;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;

namespace crf::actuators::robot {

class RobotMock : public IRobot {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(crf::expected<JointPositions>, getJointPositions, (), (override));
    MOCK_METHOD(crf::expected<JointVelocities>, getJointVelocities, (), (override));
    MOCK_METHOD(crf::expected<JointAccelerations>, getJointAccelerations, (), (override));
    MOCK_METHOD(crf::expected<JointForceTorques>, getJointForceTorques, (), (override));

    MOCK_METHOD(crf::expected<TaskPose>, getTaskPose, (), (override));
    MOCK_METHOD(crf::expected<TaskVelocity>, getTaskVelocity, (), (override));
    MOCK_METHOD(crf::expected<TaskAcceleration>, getTaskAcceleration, (), (override));
    MOCK_METHOD(crf::expected<TaskForceTorque>, getTaskForceTorque, (), (override));

    MOCK_METHOD(crf::expected<bool>, setJointPositions, (
        const bool&,
        const JointPositions&,
        const JointVelocities&,
        const JointAccelerations&), (override));
    MOCK_METHOD(crf::expected<bool>, setJointVelocities, (
        const bool&,
        const JointVelocities&,
        const JointAccelerations&), (override));
    MOCK_METHOD(crf::expected<bool>, setJointForceTorques, (
        const bool&,
        const JointForceTorques&), (override));

    MOCK_METHOD(crf::expected<bool>, setTaskPose, (
        const bool&,
        const TaskPose&,
        const TaskVelocity&,
        const TaskAcceleration&), (override));
    MOCK_METHOD(crf::expected<bool>, setTaskVelocity, (
        const bool&,
        const TaskVelocity&,
        const TaskAcceleration&), (override));
    MOCK_METHOD(crf::expected<bool>, setTaskForceTorque, (
        const bool&,
        const TaskForceTorque&), (override));

    MOCK_METHOD(crf::expected<JointVelocities>, getProfileJointVelocities, (), (override));
    MOCK_METHOD(crf::expected<JointAccelerations>, getProfileJointAccelerations, (), (override));

    MOCK_METHOD(crf::expected<TaskVelocity>, getProfileTaskVelocity, (), (override));
    MOCK_METHOD(crf::expected<TaskAcceleration>, getProfileTaskAcceleration, (), (override));

    MOCK_METHOD(crf::expected<bool>, setProfileJointVelocities, (
        const JointVelocities&), (override));
    MOCK_METHOD(crf::expected<bool>, setProfileJointAccelerations, (
        const JointAccelerations&), (override));

    MOCK_METHOD(crf::expected<bool>, setProfileTaskVelocity, (
        const TaskVelocity&), (override));
    MOCK_METHOD(crf::expected<bool>, setProfileTaskAcceleration, (
        const TaskAcceleration&), (override));

    MOCK_METHOD(crf::expected<bool>, setGravity, ((const std::array<double, 3>&)), (override));

    MOCK_METHOD(crf::expected<bool>, softStop, (), (override));
    MOCK_METHOD(crf::expected<bool>, hardStop, (), (override));

    MOCK_METHOD(crf::expected<bool>, setBrakes, (std::vector<bool>), (override));
    MOCK_METHOD(crf::expected<std::vector<bool>>, getBrakes, (), (override));

    MOCK_METHOD(std::set<Code>, robotStatus, (), (override));
    MOCK_METHOD(crf::expected<bool>, resetFaultState, (), (override));
    MOCK_METHOD(std::shared_ptr<RobotConfiguration>, getConfiguration, (), (override));
};

}  // namespace crf::actuators::robot
