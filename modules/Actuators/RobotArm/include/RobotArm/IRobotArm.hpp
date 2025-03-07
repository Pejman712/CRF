/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <memory>
#include <vector>

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/Types.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"

namespace crf::actuators::robotarm {

class IRobotArm: public utility::commoninterfaces::IInitializable {
 public:
    ~IRobotArm() override = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;
    /*
     * Returns:
     *  - position in radians of each joint on success
     *  - boost::none on failure (e.g. API failure)
     */
    virtual boost::optional<crf::utility::types::JointPositions> getJointPositions() = 0;
    /*
     * Returns:
     *  - position in radians/second of each joint on success
     *  - boost::none on failure (e.g. API failure)
     */
    virtual boost::optional<crf::utility::types::JointVelocities> getJointVelocities() = 0;
    /*
     * Returns:
     *  - torque in Nm/rad of each joint on success
     *  - boost::none on failure (e.g. torque info not available)
     */
    virtual boost::optional<crf::utility::types::JointForceTorques> getJointForceTorques() = 0;
    /*
     * Returns:
     *  - transformation matrix between the base and the end effector of the robot (on success)
     *    the unit of the translation column is meters
     *  - boost::none on failure (e.g. inverse kinematics failed or unavailable)
     */
    virtual boost::optional<crf::utility::types::TaskPose> getTaskPose() = 0;
    /*
     * Returns:
     *  - Task velocity with respect to the base frame in meters and radians
     *  - boost::none on failure (e.g. inverse kinematics failed or unavailable)
     */
    virtual boost::optional<crf::utility::types::TaskVelocity> getTaskVelocity() = 0;
    /*
     * Returns:
     *  - torque in Nm/rad of XYZ and RPY
     *  - boost::none on failure (e.g. torque info not available)
     */
    virtual boost::optional<crf::utility::types::TaskForceTorque> getTaskForceTorque() = 0;
    /*
     * Makes the robot move each joint to the desired position in radians.
     * Velocity of each joint is implementation-defined.
     * Returns true if possible
     * Returns false if it failed (e.g. values out of limits)
     */
    virtual bool setJointPositions(const crf::utility::types::JointPositions& jointPositions) = 0;
    /*
     * Makes the robot move each joint with the desired velocities in radians/second.
     * Joint accelerations are implementation-defined.
     * Returns true if possible
     * Returns false if it failed (e.g. values out of limits)
     */
    virtual bool setJointVelocities(const crf::utility::types::JointVelocities& jointVelocities) = 0; // NOLINT
    /*
     * Applies the desired torque to each joint in Nm/rad.
     * Returns true if possible
     * Returns false if it failed (e.g. operation not supported)
     */
    virtual bool setJointForceTorques(const crf::utility::types::JointForceTorques& jointForceTorques) = 0; // NOLINT
    /*
     * Makes the end effector of the robot move to the desired point in the space.
     * Position is defined as the transformation between base and effector (meters, radians)
     * Returns true if possible
     * Returns false if it failed (e.g. inverse kinematics failure)
     */
    virtual bool setTaskPose(const crf::utility::types::TaskPose& position) = 0;
    /*
     * Makes the end effector of the robot move with the desired velocity in space.
     * Velocity can be applied w.r.t. the base frame (TCP=false) or the end effector
     * frame (TCP=true)
     * Returns true if possible
     * Returns false if it failed (e.g. inverse kinematics failure)
     */
    virtual bool setTaskVelocity(const crf::utility::types::TaskVelocity& velocity, bool TCP) = 0; // NOLINT
    /*
     * Cancel all movements and stops the robot arm
     * E.g. by setting the velocities to 0 or by applying mechanical brakes.
     * Exact behavior is implementation-defined
     * Returns true if possible
     * Returns false if it failed (e.g. communication failure)
     */
    virtual bool stopArm() = 0;
    /*
     * @brief Enable the brakes of all the joints
     *
     * @return true if brakes were correctly enabled
     * @return false if brakes could not be enabled
     */
    virtual bool enableBrakes() = 0;
    /*
     * @brief Disable the brakes of all the joints
     *
     * @return true if brakes were correctly disabled
     * @return false if brakes could not be disabled
     */
    virtual bool disableBrakes() = 0;
    /*
     * @brief Get the robot arm configuration object
     *
     * @return std::shared_ptr<RobotArmConfiguration>
     */
    virtual std::shared_ptr<RobotArmConfiguration> getConfiguration() = 0;
};

}  // namespace crf::actuators::robotarm
