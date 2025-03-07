/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <set>
#include <memory>

#include "crf/expected.hpp"

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/Signals.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "MotionController/PointReferenceFrame.hpp"
#include "MotionController/TrajectoryExecutionMethod.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;

using crf::utility::types::JointVelocities;
using crf::utility::types::TaskVelocity;

using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskAcceleration;

using crf::utility::types::JointForceTorques;
using crf::utility::types::TaskForceTorque;

using crf::utility::types::JointSignals;
using crf::utility::types::TaskSignals;
using crf::utility::types::Signals;

namespace crf::control::motioncontroller {

/**
 * @ingroup group_motion_controller
 * @brief IMotionController interface class. Base for all the motion controllers.
 *
 */
class IMotionController: public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IMotionController() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief This function forces the execution of the control loop in joint space, even when no
     * movement is commanded or pending. If this function is called the only way to stop the
     * control loop is with the call of a stop method.
     */
    virtual void startJointsControlLoop() = 0;
    /**
     * @brief This function forces the execution of the control loop in task space, even when no
     * movement is commanded or pending. If this function is called the only way to stop the
     * control loop is with the call of a stop method.
     */
    virtual void startTaskControlLoop() = 0;

    /**
     * @brief Method to append a path of Joint positions. This path will be executed as
     * soon as the previous path has finished. If the previous path is still running
     * then it goes into a queue.
     *
     * @param path path of Joint Positions to be added
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> appendPath(
        const std::vector<JointPositions>& path) = 0;
    /**
     * @brief Method to append a path of Task positions. This path will be executed as
     * soon as the previous path has finished. If the previous path is still running
     * then it goes into a queue.
     *
     * @param path path of Task Positions to be added
     * @param method executing method prefered, in Task or Joint space
     * @param frame frame of refernce used for the points, Global or TCP
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> appendPath(
        const std::vector<TaskPose>& path,
        const TrajectoryExecutionMethod& method,
        const PointReferenceFrame& frame) = 0;

    /**
     * @brief Set a Joint Velocity on the robot.
     *
     * @param velocity The velocity to be set as reference
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setVelocity(
        const JointVelocities& velocity) = 0;
    /**
     * @brief Set the a Task velocity on the robot
     *
     * @param velocity The velocity to be set as reference
     * @param frame The frame of reference used for the points, Global or TCP
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setVelocity(
        const TaskVelocity& velocity,
        const PointReferenceFrame& frame) = 0;

    /**
     * @brief Set the Joint Torque on the robot
     *
     * @param torque The torque to be set as reference
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setTorque(
        const JointForceTorques& torque) = 0;
    /**
     * @brief Set the Task Torque on the robot
     *
     * @param torque The torque to be set as reference
     * @param frame Frame of reference for Task Space, options are Global or TCP from
     * the enum PointReferenceFrame
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setTorque(
        const TaskForceTorque& torque,
        const PointReferenceFrame& frame) = 0;

    /**
     * @brief Set the Joint Profile Velocity for the future appended Joint paths
     *
     * @param velocity The maximum velocity set for the profile
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setProfileVelocity(const JointVelocities& velocity) = 0;
    /**
     * @brief Set the Task Profile Velocity for the future appended Task paths
     *
     * @param velocity The maximum velocity set for the profile
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setProfileVelocity(const TaskVelocity& velocity) = 0;

    /**
     * @brief Set the Joint Profile Acceleration for the future appended Joint paths
     *
     * @param acceleration The maximum acceleration set for the profile
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setProfileAcceleration(const JointAccelerations& acceleration) = 0;
    /**
     * @brief Set the Task Profile Acceleration for the future appended Task paths
     *
     * @param acceleration The maximum acceleration set for the profile
     * @return crf::expected<bool> will return true if everything went correctly
     * or an error code if somehting failed
     */
    virtual crf::expected<bool> setProfileAcceleration(const TaskAcceleration& acceleration) = 0;

    /**
     * @brief Method to smoothly stop the control loop and stop the whole robot
     * whithout damaging the robot
     *
     */
    virtual void softStop() = 0;
    /**
     * @brief Method to force stop the robot. It will stop the control loop as fast
     * as possible and might damage the robot in the process
     *
     */
    virtual void hardStop() = 0;

    /**
     * @brief Depending on the implemenation, each controller will have a different
     * set of parameters to be tuned or modified. This method allows a JSON object to
     * be sent to the controller and interpreted as to change this values.
     *
     * @param params JSON with all the objects to be changed
     * @return true If the change was succesfull
     * @return false otherwise
     */
    virtual crf::expected<bool> setParameters(const nlohmann::json& params) = 0;
    /**
     * @brief Get the Current Parameters. It returns the current values of the parameters
     * set by the user in a JSON format. If the user has not modified them it will return
     * the default values.
     *
     * @return nlohmann::json with the fields and values
     */
    virtual nlohmann::json getCurrentParameters() = 0;
    /**
     * @brief Get the Parameters Definition. It returns a JSON objects with the fields available
     * and the type that should be provided in them.
     *
     * @return nlohmann::json with the fields and types
     */
    virtual nlohmann::json getParametersDefinition() const = 0;

    /**
     * @brief Method to let the user know if the robot is moving towards an appended position
     * @return true if the robot is inside a planned trajectory
     * @return false if it's already finished
     */
    virtual crf::expected<bool> isTrajectoryRunning() = 0;

    /**
     * @brief Get the Signals. It returns the values from the robot. This includes task
     * space and joint space values
     *
     * @return Signals An struct with the values of joints and task space of the
     * robot derivatives
     */
    virtual Signals getSignals() = 0;
    /**
     * @brief Get the status. It returns the status from the controller and the robot.
     * This includes problems on the controller itself or the robot, Inverse Kinematics, ...
     *
     * @return A set of error codes that describe the status. If everything is working fine
     * only the status OK should be read. The status resets after a new moving commnand is sent.
     */
    virtual std::set<crf::Code> getStatus() = 0;
    /**
     * @brief Request the robot configuration file
     * @return The robot configuration file
     */
    virtual std::shared_ptr<crf::actuators::robot::RobotConfiguration> getConfiguration() = 0;
};

}  // namespace crf::control::motioncontroller
