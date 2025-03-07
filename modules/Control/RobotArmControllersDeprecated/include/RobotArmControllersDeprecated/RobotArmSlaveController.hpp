#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "RobotArmControllersDeprecated/IRobotArmController.hpp"
#include "Types/Types.hpp"
#include "RobotArm/IRobotArm.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"
#include "ClosedLoopController/IClosedLoopController.hpp"
#include "EventLogger/EventLogger.hpp"

#include <memory>
#include <thread>
#include <vector>

namespace crf {
namespace applications {
namespace robotarmcontroller {

class RobotArmSlaveController : public IRobotArmController {
 public:
    RobotArmSlaveController() = delete;
    RobotArmSlaveController(
            std::shared_ptr<robots::robotarm::IRobotArm>,
            std::shared_ptr<robots::robotarmkinematics::IRobotArmKinematics>,
            std::shared_ptr<algorithms::closedloopcontroller::IClosedLoopController>);
    RobotArmSlaveController(const RobotArmSlaveController&) = default;
    ~RobotArmSlaveController() override;

    bool initialize() override;
    bool deinitialize() override;

    bool setJointPositions(const utility::types::JointPositions&) override;
    bool setTaskPose(const utility::types::TaskPose&) override;

    utility::types::JointPositions getJointPositions() override;
    utility::types::TaskPose getTaskPose() override;

    // NOT IMPLEMENTED FUNCTIONS
    std::future<bool> executeJointsTrajectory(
            const std::vector<utility::types::JointPositions>&) override;
    std::future<bool> executeJointsTrajectory(
            const std::shared_ptr<algorithms::trajectorygenerator::IJointsTrajectoryGenerator>&) override;  // NOLINT
    std::future<bool> executeTaskTrajectory(
            const std::vector<utility::types::TaskPose>&) override;
    std::future<bool> executeTaskTrajectoryLinear(
            const std::vector<utility::types::TaskPose>&) override;
    std::future<bool> executeTaskTrajectory(
            const std::shared_ptr<algorithms::trajectorygenerator::ITaskTrajectoryGenerator>&) override;  // NOLINT
    bool interruptTrajectory() override;
    bool setJointsMaximumVelocity(const utility::types::JointVelocities&) override;
    bool setJointsMaximumAcceleration(const utility::types::JointAccelerations&) override;
    bool setTaskMaximumVelocity(const utility::types::TaskVelocity&) override;
    bool setTaskMaximumAcceleration(const utility::types::TaskAcceleration&) override;
    bool setJointVelocities(const utility::types::JointVelocities&) override;
    bool setTaskVelocity(const utility::types::TaskVelocity&, bool) override;
    utility::types::JointVelocities getJointVelocities() override;
    crf::utility::types::JointForceTorques getJointForceTorques() override;
    utility::types::TaskVelocity getTaskVelocity() override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<robots::robotarm::IRobotArm> arm_;
    std::shared_ptr<robots::robotarmkinematics::IRobotArmKinematics> kinematics_;
    std::shared_ptr<algorithms::closedloopcontroller::IClosedLoopController> positionController_;
    std::shared_ptr<robots::robotarm::RobotArmConfiguration> configuration_;
    const int jointsCount_;
    std::chrono::microseconds rtLoopTime_;
    bool stopControlLoop_;
    std::thread controlLoopThread_;
    const float movementThreshold_;
    utility::types::JointPositions currentJointPositions_;
    utility::types::JointVelocities currentJointVelocities_;
    utility::types::JointPositions targetJointPositions_;
    void controlLoop();
    utility::types::JointVelocities calculateJointVelocities();
};

}  // namespace robotarmcontroller
}  // namespace applications
}  // namespace crf
