/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2022
 *         Hannes Gamper CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>
#include <set>

#include "crf/expected.hpp"

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/Types.hpp"
#include "Robot/RobotConfiguration.hpp"

namespace crf::actuators::robot {

/**
 * @ingroup group_robot
 * @brief An Interface for all robots facilitating generalized and easy communication in order to
 *        move the robot, read robot status, enable brakes and more.
 */
class IRobot: public utility::commoninterfaces::IInitializable {
 public:
    ~IRobot() override = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Requests the current joint positions from the robot
     * @return The joints position
     */
    virtual crf::expected<crf::utility::types::JointPositions> getJointPositions() = 0;

    /**
     * @brief Requests the current joint velocity from the robot
     * @return The joints velocity
     */
    virtual crf::expected<crf::utility::types::JointVelocities> getJointVelocities() = 0;

    /**
     * @brief Requests the current joint acceleration from the robot
     * @return The joints acceleration
     */
    virtual crf::expected<crf::utility::types::JointAccelerations> getJointAccelerations() = 0;

    /**
     * @brief Requests the current joint torque from the robot
     * @return The joints torque
     */
    virtual crf::expected<crf::utility::types::JointForceTorques> getJointForceTorques() = 0;

    /**
     * @brief Requests the current task positions from the robot
     * @return The task position
     */
    virtual crf::expected<crf::utility::types::TaskPose> getTaskPose() = 0;

    /**
     * @brief Requests the current task velocity from the robot
     * @return The task velocity
     */
    virtual crf::expected<crf::utility::types::TaskVelocity> getTaskVelocity() = 0;

    /**
     * @brief Requests the current task acceleration from the robot
     * @return The task acceleration
     */
    virtual crf::expected<crf::utility::types::TaskAcceleration> getTaskAcceleration() = 0;

    /**
     * @brief Requests the current task force & torque (wrench) from the robot
     * @return The task force & torque (wrench)
     */
    virtual crf::expected<crf::utility::types::TaskForceTorque> getTaskForceTorque() = 0;

    /**
     * @brief Set the desired joints position
     * @param isSmoothTrajectory Defining if the desired joints positions are part of a smooth
     *        trajectory. If not, the robot uses its internal trajectory planning.
     * @param jointPositions The desired joints position
     * @param jointVelocities [Optional] The desired joints velocity
     * @param jointAccelerations [Optional] The desired joints acceleration
     * @return True in case of success and False in case of problems.
     */
    virtual crf::expected<bool> setJointPositions(const bool& isSmoothTrajectory,
        const crf::utility::types::JointPositions& jointPositions,
        const crf::utility::types::JointVelocities& jointVelocities = crf::utility::types::JointVelocities(),  // NOLINT
        const crf::utility::types::JointAccelerations& jointAccelerations = crf::utility::types::JointAccelerations()) = 0;  // NOLINT

    /**
     * @brief Set the desired joints velocity
     * @param isSmoothTrajectory Defining if the desired joints velocity are part of a smooth
     *        trajectory. If not, the robot uses its internal trajectory planning.
     * @param jointVelocities The desired joints velocity
     * @param jointAccelerations [Optional] The desired joints acceleration
     * @return True in case of success and False in case of problems.
     */
    virtual crf::expected<bool> setJointVelocities(const bool& isSmoothTrajectory,
        const crf::utility::types::JointVelocities& jointVelocities,
        const crf::utility::types::JointAccelerations& jointAccelerations = crf::utility::types::JointAccelerations()) = 0;  // NOLINT

    /**
     * @brief Set the desired joints torque
     * @param isSmoothTrajectory Defining if the desired joints torque are part of a smooth
     *        trajectory. If not, the robot uses its internal trajectory planning.
     * @param JointForceTorques The desired joints torque
     * @return True in case of success and False in case of problems.
     */
    virtual crf::expected<bool> setJointForceTorques(const bool& isSmoothTrajectory,
        const crf::utility::types::JointForceTorques& jointForceTorques) = 0;

    /**
     * @brief Set the desired task position
     * @param isSmoothTrajectory Defining if the desired task positions are part of a smooth
     *        trajectory. If not, the robot uses its internal trajectory planning.
     * @param TaskPose The desired task position
     * @param taskVelocity [Optional] The desired task velocity
     * @param taskAcceleration [Optional] The desired task acceleration
     * @return True in case of success and False in case of problems.
     */
    virtual crf::expected<bool> setTaskPose(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskPose& taskPose,
        const crf::utility::types::TaskVelocity& taskVelocity = crf::utility::types::TaskVelocity(),  // NOLINT
        const crf::utility::types::TaskAcceleration& taskAcceleration = crf::utility::types::TaskAcceleration()) = 0;  // NOLINT

    /**
     * @brief Set the desired task velocity
     * @param isSmoothTrajectory Defining if the desired task velocities are part of a smooth
     *        trajectory. If not, the robot uses its internal trajectory planning.
     * @param taskVelocity The desired task velocity
     * @param taskAcceleration [Optional] The desired task acceleration
     * @return True in case of success and False in case of problems.
     */
    virtual crf::expected<bool> setTaskVelocity(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskVelocity& taskVelocity,
        const crf::utility::types::TaskAcceleration& taskAcceleration = crf::utility::types::TaskAcceleration()) = 0;  // NOLINT

    /**
     * @brief Set the desired task torque
     * @param isSmoothTrajectory Defining if the desired task torque are part of a smooth
     *        trajectory. If not, the robot uses its internal trajectory planning.
     * @param JointForceTorques The desired task torque
     * @return True in case of success and False in case of problems.
     */
    virtual crf::expected<bool> setTaskForceTorque(const bool& isSmoothTrajectory,
        const crf::utility::types::TaskForceTorque& taskForceTorque) = 0;

    /**
     * @brief Request the current value of profileJointVelocities used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @return The profileJointVelocities
     */
    virtual crf::expected<crf::utility::types::JointVelocities> getProfileJointVelocities() = 0;

    /**
     * @brief Request the current value of profileJointAccelerations used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @return The profileJointAccelerations
     */
    virtual crf::expected<crf::utility::types::JointAccelerations> getProfileJointAccelerations() = 0;  // NOLINT

    /**
     * @brief Request the current value of profileTaskVelocity used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @return The profileTaskVelocity
     */
    virtual crf::expected<crf::utility::types::TaskVelocity> getProfileTaskVelocity() = 0;

    /**
     * @brief Request the current value of profileTaskAcceleration used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @return The profileTaskAcceleration
     */
    virtual crf::expected<crf::utility::types::TaskAcceleration> getProfileTaskAcceleration() = 0;

    /**
     * @brief Set the current value of profileJointVelocities used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @param jointVelocities The desired profileJointVelocities for internal trajectory planning
     * @return True if the update was successful, false otherwise.
     */
    virtual crf::expected<bool> setProfileJointVelocities(
        const crf::utility::types::JointVelocities& jointVelocities) = 0;

    /**
     * @brief Set the current value of profileJointAccelerations used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @param jointAccelerations The desired profileJointAccelerations for internal trajectory planning
     * @return True if the update was successful, false otherwise.
     */
    virtual crf::expected<bool> setProfileJointAccelerations(
        const crf::utility::types::JointAccelerations& jointAccelerations) = 0;

    /**
     * @brief Set the current value of profileTaskVelocity used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @param taskVelocity The desired profileTaskVelocity for internal trajectory planning
     * @return True if the update was successful, false otherwise.
     */
    virtual crf::expected<bool> setProfileTaskVelocity(
        const crf::utility::types::TaskVelocity& taskVelocity) = 0;

    /**
     * @brief Set the current value of profileTaskAcceleration used for internal trajectory
     *        planning in case of isSmoothTrajectory = false.
     * @param taskAcceleration The desired profileTaskAcceleration for internal trajectory planning
     * @return True if the update was successful, false otherwise.
     */
    virtual crf::expected<bool> setProfileTaskAcceleration(
        const crf::utility::types::TaskAcceleration& taskAcceleration) = 0;

    /**
     * @brief Set the gravity vector with respect to the inertial frame of the robot.
     * @param gravity A 3 dimensional unit vector representing the direction of the gravity, scaled
     *        with the acceleration, e.g. 9.81 m/s^2.
     * @return True if the update was successful, false otherwise.
     */
    virtual crf::expected<bool> setGravity(const std::array<double, 3>& gravity) = 0;

    /**
     * @brief Stops the movement with a certain acceleration that is not harmful to the robot.
     * @return True if successful, false otherwise.
     */
    virtual crf::expected<bool> softStop() = 0;

    /**
     * @brief Stops the robot as fast as possible (emergency stop), possibly using mechanical brakes
     *        or accelerations that will harm the robot.
     * @return True if successful, false otherwise.
     */
    virtual crf::expected<bool> hardStop() = 0;

    /**
     * @brief Enable or Disable robot brakes
     * @param brakesStatus A boolean vector enabling or disabling the brakes. The index corresponds
     *        to the joint number.
     * @return True if successful, false otherwise.
     */
    virtual crf::expected<bool> setBrakes(std::vector<bool> brakesStatus) = 0;

    /**
     * @brief Request the robot brakes status
     * @return Vector of bloolean describing if the brakes are enabled or disabled. The index
     *         corresponds to the joint number.
     */
    virtual crf::expected<std::vector<bool>> getBrakes() = 0;

    /**
     * @brief Request the robot status.
     * @return std::set<Code> The robot status
     */
    virtual std::set<Code> robotStatus() = 0;

    /**
     * @brief Reset the fault state of the robot
     * @return True if successful, false otherwise.
     */
    virtual crf::expected<bool> resetFaultState() = 0;

    /**
     * @brief Request the robot configuration file
     * @return The robot configuration file
     */
    virtual std::shared_ptr<RobotConfiguration> getConfiguration() = 0;
};

}  // namespace crf::actuators::robot
