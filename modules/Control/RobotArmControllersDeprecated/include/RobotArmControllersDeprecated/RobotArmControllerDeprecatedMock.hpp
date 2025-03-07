/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <vector>

#include "RobotArmControllersDeprecated/IRobotArmController.hpp"

namespace crf {
namespace applications {
namespace robotarmcontroller {

class RobotArmControllerDeprecatedMock : public IRobotArmController {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());

    MOCK_METHOD1(executeJointsTrajectory,
        std::future<bool>(
            const std::vector<utility::types::JointPositions>&));
    MOCK_METHOD1(executeJointsTrajectory,
        std::future<bool>(
            const std::shared_ptr<algorithms::trajectorygenerator::IJointsTrajectoryGenerator>&));

    MOCK_METHOD1(executeTaskTrajectory,
        std::future<bool>(const std::vector<utility::types::TaskPose>&));
    MOCK_METHOD1(executeTaskTrajectoryLinear,
        std::future<bool>(const std::vector<utility::types::TaskPose>&));
    MOCK_METHOD1(executeTaskTrajectory,
        std::future<bool>(const std::shared_ptr<algorithms::trajectorygenerator::ITaskTrajectoryGenerator>&)); // NOLINT

    MOCK_METHOD0(interruptTrajectory,
        bool());

    MOCK_METHOD1(setJointPositions,
        bool(const utility::types::JointPositions&));
    MOCK_METHOD1(setTaskPose,
        bool(const utility::types::TaskPose&));
    MOCK_METHOD1(setJointVelocities,
        bool(const utility::types::JointVelocities&));
    MOCK_METHOD2(setTaskVelocity,
        bool(const utility::types::TaskVelocity&, bool));

    MOCK_METHOD0(getJointPositions,
        utility::types::JointPositions());
    MOCK_METHOD0(getJointVelocities,
        utility::types::JointVelocities());
    MOCK_METHOD0(getJointForceTorques,
        crf::utility::types::JointForceTorques());
    MOCK_METHOD0(getTaskPose,
        utility::types::TaskPose());
    MOCK_METHOD0(getTaskVelocity,
        utility::types::TaskVelocity());

    MOCK_METHOD1(setJointsMaximumVelocity,
        bool(const utility::types::JointVelocities&));
    MOCK_METHOD1(setJointsMaximumAcceleration,
        bool(const utility::types::JointAccelerations&));
    MOCK_METHOD1(setTaskMaximumVelocity,
        bool(const utility::types::TaskVelocity&));
    MOCK_METHOD1(setTaskMaximumAcceleration,
        bool(const utility::types::TaskAcceleration&));
};

}  // namespace robotarmcontroller
}  // namespace applications
}  // namespace crf
