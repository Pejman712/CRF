/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
*/

#pragma once

#include <gmock/gmock.h>
#include <vector>

#include "RobotArmController/IRobotArmController.hpp"

namespace crf::control::robotarmcontroller {

class RobotArmControllerMock : public IRobotArmController {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));


    MOCK_METHOD(std::future<bool>, setPosition,
        (const crf::utility::types::JointPositions& pos), (override));
    MOCK_METHOD(std::future<bool>, setPosition,
        (const std::vector<crf::utility::types::JointPositions>& pos), (override));
    MOCK_METHOD(std::future<bool>, setPosition,
        (const crf::utility::types::TaskPose&,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod,
        crf::control::robotarmcontroller::PointReferenceFrame), (override));
    MOCK_METHOD(std::future<bool>, setPosition,
        (const std::vector<crf::utility::types::TaskPose>&,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod,
        crf::control::robotarmcontroller::PointReferenceFrame), (override));

    MOCK_METHOD(bool, setVelocity,
        (const crf::utility::types::JointVelocities&), (override));
    MOCK_METHOD(bool, setVelocity,
        (const std::vector<crf::utility::types::JointVelocities>&), (override));
    MOCK_METHOD(bool, setVelocity,
        (const crf::utility::types::TaskVelocity&,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod,
        crf::control::robotarmcontroller::PointReferenceFrame), (override));
    MOCK_METHOD(bool, setVelocity,
        (const std::vector<crf::utility::types::TaskVelocity>&,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod,
        crf::control::robotarmcontroller::PointReferenceFrame), (override));

    MOCK_METHOD(bool, setAcceleration,
        (const crf::utility::types::JointAccelerations&), (override));
    MOCK_METHOD(bool, setAcceleration,
        (const std::vector<crf::utility::types::JointAccelerations>&), (override));
    MOCK_METHOD(bool, setAcceleration,
        (const crf::utility::types::TaskAcceleration&,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod,
        crf::control::robotarmcontroller::PointReferenceFrame), (override));
    MOCK_METHOD(bool, setAcceleration,
        (const std::vector<crf::utility::types::TaskAcceleration>&,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod,
        crf::control::robotarmcontroller::PointReferenceFrame), (override));

    MOCK_METHOD(bool, interruptTrajectory, (), (override));

    MOCK_METHOD(crf::utility::types::JointPositions, getJointPositions, (), (override));
    MOCK_METHOD(crf::utility::types::JointVelocities, getJointVelocities, (), (override));
    MOCK_METHOD(crf::utility::types::JointAccelerations, getJointAccelerations, (), (override));
    MOCK_METHOD(crf::utility::types::JointForceTorques, getJointForceTorques, (), (override));
    MOCK_METHOD(crf::utility::types::TaskPose, getTaskPose, (), (override));
    MOCK_METHOD(crf::utility::types::TaskVelocity, getTaskVelocity, (), (override));
    MOCK_METHOD(crf::utility::types::TaskAcceleration, getTaskAcceleration, (), (override));

    MOCK_METHOD(bool, setJointsMaximumVelocity,
        (const utility::types::JointVelocities&), (override));
    MOCK_METHOD(bool, setJointsMaximumAcceleration,
        (const utility::types::JointAccelerations&), (override));
    MOCK_METHOD(bool, setTaskMaximumVelocity,
        (const utility::types::TaskVelocity&), (override));
    MOCK_METHOD(bool, setTaskMaximumAcceleration,
        (const utility::types::TaskAcceleration&), (override));
};

}  // namespace crf::control::robotarmcontroller
