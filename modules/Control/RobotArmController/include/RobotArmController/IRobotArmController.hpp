/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *          Alejandro Diaz Rosales CERN EN/SMM/MRO 2020
 *          Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <future>
#include <vector>
#include <memory>

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/Types.hpp"
#include "RobotArmController/TrajectoryExecutionMethod.hpp"
#include "RobotArmController/PointReferenceFrame.hpp"

namespace crf::control::robotarmcontroller {

class IRobotArmController : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IRobotArmController() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;
    /**
     * @brief Function to set a position on the robot arm
     * @param Position in joints that we want to achieve
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual std::future<bool> setPosition(
        const crf::utility::types::JointPositions& position) = 0;
    /**
     * @brief Function to set a path of position on the robot arm
     * @param Vector of positions in joints that we want to achieve
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual std::future<bool> setPosition(
        const std::vector<crf::utility::types::JointPositions>& positions) = 0;
    /**
     * @brief Function to set a position on the robot arm
     * @param Task position we want to achieve
     * @param Type of trajectory we want to reach the point with (Joint, Task, ...)
     * @param Point of reference for the task movement (Global, TCP)
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual std::future<bool> setPosition(
        const crf::utility::types::TaskPose& position,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) = 0;
    /**
     * @brief Function to set a a path of positions on the robot arm
     * @param Vector of task positions we want to achieve
     * @param Type of trajectory we want to reach the point with (Joint, Task, ...)
     * @param Point of reference for the task movement (Global, TCP)
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual std::future<bool> setPosition(
        const std::vector<crf::utility::types::TaskPose>& positions,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) = 0;
    /**
     * @brief Function to set a velocity on the robot arm
     * @param Velocity in joints that we want to achieve
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setVelocity(
        const crf::utility::types::JointVelocities& velocity) = 0;
    /**
     * @brief Function to set a path of velocities on the robot arm
     * @param Vector of velocities in joints that we want to achieve
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setVelocity(
        const std::vector<crf::utility::types::JointVelocities>& velocities) = 0;
    /**
     * @brief Function to set a Velocity on the robot arm
     * @param Vector of task velocities we want to achieve
     * @param Type of trajectory we want to reach the velocity with (Joint, Task, ...)
     * @param Point of reference for the task movement (Global, TCP)
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setVelocity(
        const crf::utility::types::TaskVelocity& velocity,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) = 0;
    /**
     * @brief Function to set a path of velocities on the robot arm
     * @param Vector of task velocities we want to achieve
     * @param Type of trajectory we want to reach the velocity with (Joint, Task, ...)
     * @param Point of reference for the task movement (Global, TCP)
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setVelocity(
        const std::vector<crf::utility::types::TaskVelocity>& velocities,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) = 0;
    /**
     * @brief Function to set an acceleration on the robot arm
     * @param Acceleration in joints that we want to achieve
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setAcceleration(
        const crf::utility::types::JointAccelerations& acceleration) = 0;
    /**
     * @brief Function to set a path of accelerations on the robot arm
     * @param Vector of accelerations in joints that we want to achieve
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setAcceleration(
        const std::vector<crf::utility::types::JointAccelerations>& accelerations) = 0;
    /**
     * @brief Function to set an acceleration on the robot arm
     * @param Task acceleration we want to achieve
     * @param Type of trajectory we want to reach the acceleration with (Joint, Task, ...)
     * @param Point of reference for the task movement (Global, TCP)
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setAcceleration(
        const crf::utility::types::TaskAcceleration& acceleration,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) = 0;
    /**
     * @brief Function to set a path of accelerations on the robot arm
     * @param Vector of task accelerations we want to achieve
     * @param Type of trajectory we want to reach the acceleration with (Joint, Task, ...)
     * @param Point of reference for the task movement (Global, TCP)
     * @return Future value while the movement is in progress, bool once finished
     */
    virtual bool setAcceleration(
        const std::vector<crf::utility::types::TaskAcceleration>& accelerations,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
        crf::control::robotarmcontroller::PointReferenceFrame frame) = 0;
    /**
     * @brief Function to stop an on-going trajectory
     * @return True if the trajectory could be stopped
     * @return False otherwise
     */
    virtual bool interruptTrajectory() = 0;

    /**
     * @brief Obtain the current joint position of the robot arm
     * @return Current joint position
     */
    virtual crf::utility::types::JointPositions getJointPositions() = 0;
    /**
     * @brief Obtain the current task position of the robot arm
     * @return Current task position
     */
    virtual crf::utility::types::TaskPose getTaskPose() = 0;
    /**
     * @brief Obtain the current joints velocity of the robot arm
     * @return Current joints velocity
     */
    virtual crf::utility::types::JointVelocities getJointVelocities() = 0;
    /**
     * @brief Obtain the current task velocity of the robot arm
     * @return Current task velocity
     */
    virtual crf::utility::types::TaskVelocity getTaskVelocity() = 0;
    /**
     * @brief Obtain the current joints acceleration of the robot arm
     * @return Current joints acceleration
     */
    virtual crf::utility::types::JointAccelerations getJointAccelerations() = 0;
    /**
     * @brief Obtain the current task acceleration of the robot arm
     * @return Current task acceleration
     */
    virtual crf::utility::types::TaskAcceleration getTaskAcceleration() = 0;
    /**
     * @brief Obtain the current joints torque of the robot arm
     * @return Current joints torque
     */
    virtual crf::utility::types::JointForceTorques getJointForceTorques() = 0;
    /**
     * @brief Function to set the maximum joints velocity
     * @return True if the change was successful
     * @return False otherwise
     */
    virtual bool setJointsMaximumVelocity(
        const crf::utility::types::JointVelocities& velocity) = 0;
    /**
     * @brief Function to set the maximum task velocity
     * @return True if the change was successful
     * @return False otherwise
     */
    virtual bool setTaskMaximumVelocity(
        const crf::utility::types::TaskVelocity& velocity) = 0;
    /**
     * @brief Function to set the maximum joints acceleration
     * @return True if the change was successful
     * @return False otherwise
     */
    virtual bool setJointsMaximumAcceleration(
        const crf::utility::types::JointAccelerations& acceleration) = 0;
    /**
     * @brief Function to set the maximum task acceleration
     * @return True if the change was successful
     * @return False otherwise
     */
    virtual bool setTaskMaximumAcceleration(
        const crf::utility::types::TaskAcceleration& acceleration) = 0;
};

}  // namespace crf::control::robotarmcontroller
