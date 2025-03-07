/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/

#pragma once

#include <vector>
#include <set>
#include <memory>

#include <gmock/gmock.h>

#include "MotionController/IMotionController.hpp"

namespace crf::control::motioncontroller {

class MotionControllerMock : public IMotionController {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(void, startJointsControlLoop, (), (override));
    MOCK_METHOD(void, startTaskControlLoop, (), (override));

    MOCK_METHOD(crf::expected<bool>, appendPath,
        (const std::vector<crf::utility::types::JointPositions>& pos), (override));
    MOCK_METHOD(crf::expected<bool>, appendPath,
        (const std::vector<crf::utility::types::TaskPose>&,
        const crf::control::motioncontroller::TrajectoryExecutionMethod&,
        const crf::control::motioncontroller::PointReferenceFrame&), (override));

    MOCK_METHOD(crf::expected<bool>, setVelocity,
        (const crf::utility::types::JointVelocities&), (override));
    MOCK_METHOD(crf::expected<bool>, setVelocity,
        (const crf::utility::types::TaskVelocity&,
        const crf::control::motioncontroller::PointReferenceFrame&), (override));

    MOCK_METHOD(crf::expected<bool>, setTorque,
        (const crf::utility::types::JointForceTorques&), (override));
    MOCK_METHOD(crf::expected<bool>, setTorque,
        (const crf::utility::types::TaskForceTorque&,
        const crf::control::motioncontroller::PointReferenceFrame&), (override));

    MOCK_METHOD(crf::expected<bool>, setProfileVelocity,
        (const utility::types::JointVelocities&), (override));
    MOCK_METHOD(crf::expected<bool>, setProfileVelocity,
        (const utility::types::TaskVelocity&), (override));
    MOCK_METHOD(crf::expected<bool>, setProfileAcceleration,
        (const utility::types::JointAccelerations&), (override));
    MOCK_METHOD(crf::expected<bool>, setProfileAcceleration,
        (const utility::types::TaskAcceleration&), (override));

    MOCK_METHOD(void, softStop, (), (override));
    MOCK_METHOD(void, hardStop, (), (override));

    MOCK_METHOD(crf::expected<bool>, isTrajectoryRunning, (), (override));
    MOCK_METHOD(crf::utility::types::Signals, getSignals, (), (override));
    MOCK_METHOD(std::set<crf::Code>, getStatus, (), (override));

    MOCK_METHOD(crf::expected<bool>, setParameters,
        (const nlohmann::json&), (override));
    MOCK_METHOD(nlohmann::json, getCurrentParameters, (), (override));
    MOCK_METHOD(nlohmann::json, getParametersDefinition, (), (const, override));
    MOCK_METHOD(std::shared_ptr<crf::actuators::robot::RobotConfiguration>, getConfiguration,
        (), (override));
};

}  // namespace crf::control::motioncontroller
