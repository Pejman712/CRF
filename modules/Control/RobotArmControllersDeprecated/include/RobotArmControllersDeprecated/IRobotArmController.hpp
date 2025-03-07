#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <future>
#include <memory>
#include <vector>

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/Types.hpp"
#include "TrajectoryGenerator/TaskLinearTrajectory.hpp"
#include "TrajectoryGenerator/ITaskTrajectoryGenerator.hpp"
#include "TrajectoryGenerator/IJointsTrajectoryGenerator.hpp"

namespace crf {
namespace applications {
namespace robotarmcontroller {

class IRobotArmController : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IRobotArmController() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    virtual std::future<bool> executeJointsTrajectory(
        const std::vector<utility::types::JointPositions>&) = 0;
    virtual std::future<bool> executeJointsTrajectory(
        const std::shared_ptr<algorithms::trajectorygenerator::IJointsTrajectoryGenerator>&) = 0;

    virtual std::future<bool> executeTaskTrajectory(
        const std::vector<utility::types::TaskPose>&) = 0;
    virtual std::future<bool> executeTaskTrajectoryLinear(
        const std::vector<utility::types::TaskPose>&) = 0;
    virtual std::future<bool> executeTaskTrajectory(
        const std::shared_ptr<algorithms::trajectorygenerator::ITaskTrajectoryGenerator>&) = 0;

    virtual bool interruptTrajectory() = 0;

    virtual bool setJointPositions(const utility::types::JointPositions&) = 0;
    virtual bool setTaskPose(const utility::types::TaskPose&) = 0;

    virtual bool setJointVelocities(const utility::types::JointVelocities&) = 0;
    virtual bool setTaskVelocity(const utility::types::TaskVelocity&, bool) = 0;

    virtual utility::types::JointPositions getJointPositions() = 0;
    virtual utility::types::JointVelocities getJointVelocities() = 0;
    virtual crf::utility::types::JointForceTorques getJointForceTorques() = 0;
    virtual utility::types::TaskPose getTaskPose() = 0;
    virtual utility::types::TaskVelocity getTaskVelocity() = 0;

    virtual bool setJointsMaximumVelocity(const utility::types::JointVelocities&) = 0;
    virtual bool setJointsMaximumAcceleration(const utility::types::JointAccelerations&) = 0;
    virtual bool setTaskMaximumVelocity(const utility::types::TaskVelocity&) = 0;
    virtual bool setTaskMaximumAcceleration(
        const utility::types::TaskAcceleration&) = 0;
};

}  // namespace robotarmcontroller
}  // namespace applications
}  // namespace crf
