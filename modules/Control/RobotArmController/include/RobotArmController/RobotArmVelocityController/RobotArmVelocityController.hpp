/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO
 *         Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *
 *  ==================================================================================================
 */

#pragma once

#include <Eigen/Dense>
#include <future>
#include <vector>
#include <memory>

#include "RobotArmController/IRobotArmController.hpp"
#include "Types/Types.hpp"
#include "TrajectoryGeneratorDeprecated/IJointsTrajectoryGenerator.hpp"
#include "TrajectoryGeneratorDeprecated/ITaskTrajectoryGenerator.hpp"

#include "RobotArm/IRobotArm.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"

#include "EventLogger/EventLogger.hpp"
#include "TrajectoryGeneratorDeprecated/JointsTimeOptimalTrajectory.hpp"
#include "TrajectoryGeneratorDeprecated/TaskTimeOptimalTrajectory.hpp"
#include "TrajectoryGeneratorDeprecated/TaskLinearTrajectory.hpp"
#include "ClosedLoopController/PIDController.hpp"
#include "GraphPlot/GraphPlot.hpp"

#define CONTROLLER_ENABLE_LOGGING "CONTROLLER_ENABLE_LOGGING"

namespace crf::control::robotarmcontroller {

/*
 * @brief
 */
class RobotArmVelocityController : public IRobotArmController {
 public:
    explicit RobotArmVelocityController(std::shared_ptr<crf::actuators::robotarm::IRobotArm>);
    ~RobotArmVelocityController() override;

    bool initialize() override;
    bool deinitialize() override;

    std::future<bool> setPosition(
        const crf::utility::types::JointPositions& position) override;
    std::future<bool> setPosition(
        const std::vector<crf::utility::types::JointPositions>& positions) override;
    std::future<bool> setPosition(
        const crf::utility::types::TaskPose& position,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;
    std::future<bool> setPosition(
        const std::vector<crf::utility::types::TaskPose>& positions,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;

    bool setVelocity(const crf::utility::types::JointVelocities& velocity) override;
    bool setVelocity(
        const std::vector<crf::utility::types::JointVelocities>& velocities) override;
    bool setVelocity(
        const crf::utility::types::TaskVelocity& velocity,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;
    bool setVelocity(
        const std::vector<crf::utility::types::TaskVelocity>& velocities,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;

    bool setAcceleration(
        const crf::utility::types::JointAccelerations& acceleration) override;
    bool setAcceleration(
        const std::vector<crf::utility::types::JointAccelerations>& accelerations) override;
    bool setAcceleration(
        const crf::utility::types::TaskAcceleration& acceleration,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;
    bool setAcceleration(
        const std::vector<crf::utility::types::TaskAcceleration>& accelerations,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) override;

    bool interruptTrajectory() override;

    crf::utility::types::JointPositions getJointPositions() override;
    crf::utility::types::TaskPose getTaskPose() override;
    crf::utility::types::JointVelocities getJointVelocities() override;
    crf::utility::types::TaskVelocity getTaskVelocity() override;
    crf::utility::types::JointAccelerations getJointAccelerations() override;
    crf::utility::types::TaskAcceleration getTaskAcceleration() override;
    crf::utility::types::JointForceTorques getJointForceTorques() override;

    bool setJointsMaximumVelocity(
        const crf::utility::types::JointVelocities& velocity) override;
    bool setTaskMaximumVelocity(
        const crf::utility::types::TaskVelocity& velocity) override;
    bool setJointsMaximumAcceleration(
        const crf::utility::types::JointAccelerations& acceleration) override;
    bool setTaskMaximumAcceleration(
        const crf::utility::types::TaskAcceleration& acceleration) override;

 private:
    std::shared_ptr<crf::actuators::robotarm::IRobotArm> arm_;
    std::shared_ptr<crf::control::robotarmkinematics::IRobotArmKinematics> kinematics_;
    std::shared_ptr<crf::control::closedloopcontroller::PIDController> jointPositionsPIDController_;
    std::shared_ptr<crf::control::closedloopcontroller::PIDController> taskPosePIDController_;
    bool logging_;

    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> configuration_;
    std::vector<crf::actuators::robotarm::JointLimits> jointsConfiguration_;
    unsigned int numberJoints_;
    utility::types::JointVelocities targetJointVelocities_;

    std::chrono::microseconds rtLoopTime_;

    std::mutex currentTaskStatusMutex_;
    std::mutex currentJointsStatusMutex_;
    std::mutex targetVelocityMutex_;

    std::thread controlLoopThread_;

    utility::types::JointVelocities jointsMaximumVelocity_;
    utility::types::JointAccelerations jointsMaximumAcceleration_;
    utility::types::TaskVelocity taskMaximumVelocity_;
    utility::types::TaskAcceleration taskMaximumAcceleration_;

    utility::types::JointPositions currentJointPositions_;
    utility::types::JointVelocities currentJointVelocities_;
    crf::utility::types::JointForceTorques currentJointForceTorques_;
    utility::types::TaskPose currentTaskPose_;
    utility::types::TaskVelocity currentTaskVelocity_;

    utility::types::TaskVelocity taskVelocityToJoints_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastCommandTime_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdateTime_;
    std::atomic<bool> stopControlLoop_;
    std::atomic<bool> interruptTrajectory_;
    std::atomic<bool> armMoving_;

    crf::control::robotarmcontroller::TrajectoryExecutionMethod trajectoryMethod_;
    const std::chrono::milliseconds velocityCommandInterval_;
    bool initialized_;

    utility::types::JointPositions targetPosition_;
    utility::types::TaskPose targetPositionTask_;
    utility::types::JointVelocities targetVelocity_;
    utility::types::TaskVelocity targetVelocityTask_;
    std::vector<utility::types::JointPositions> recordTargetJointPos_;
    std::vector<utility::types::JointVelocities> recordTargetJointVel_;
    std::vector<utility::types::TaskPose> recordTargetTaskPos_;
    std::vector<utility::types::TaskVelocity> recordTargetTaskVel_;
    std::vector<utility::types::JointPositions> recordJointPos_;
    std::vector<utility::types::JointVelocities> recordJointVel_;
    std::vector<utility::types::TaskPose> recordTaskPos_;
    std::vector<utility::types::TaskVelocity> recordTaskVel_;

    utility::logger::EventLogger logger_;

    bool setJointVelocities(const crf::utility::types::JointVelocities& velocity,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method);
    bool setTaskVelocity(const crf::utility::types::TaskVelocity& velocity,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame);

    bool executeJointsTrajectory(
        const std::shared_ptr<crf::control::trajectorygeneratordeprecated::IJointsTrajectoryGenerator>& trajectory);  // NOLINT
    bool executeJointsTrajectory(
        const std::shared_ptr<crf::control::trajectorygeneratordeprecated::ITaskTrajectoryGenerator>& trajectory);  // NOLINT
    bool executeTaskTrajectory(
        const std::shared_ptr<crf::control::trajectorygeneratordeprecated::ITaskTrajectoryGenerator>& trajectory);  // NOLINT

    void controlLoop();
    bool updateArmStatus();

    utility::types::JointVelocities jointVelocitiesControl();
    utility::types::JointVelocities taskVelocityControl();
};

}  // namespace crf::control::robotarmcontroller
