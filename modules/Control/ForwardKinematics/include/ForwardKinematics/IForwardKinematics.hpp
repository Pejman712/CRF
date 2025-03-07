/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
*/

#pragma once

#include <optional>

#include "Types/Types.hpp"

namespace crf::control::forwardkinematics {

/**
 * @ingroup group_forward_kinematics
 * @brief Interface for the Forward Kinematics.
 *
 */
class IForwardKinematics {
 public:
    virtual ~IForwardKinematics() = default;

    /**
     * @brief Function to get the end-effector position knowing the joint positions of the robot
     *        arm.
     *
     * @param jointPositions are the specified position values of the joints.
     * @return TaskPose variable that contains the position and orientation of the end-effector
     *         or the custom task space elemts.
     * @return std::nullopt on failure.
     */
    virtual std::optional<crf::utility::types::TaskPose> getPose(
        const crf::utility::types::JointPositions& jointPositions) = 0;

    /**
     * @brief Function to get the end-effector velocity knowing the joint positions and velocities
     *        of the robot arm.
     *
     * @param jointPositions are the specified position values of the robot joints.
     * @param jointVelocities are the specified velocity values of the robot joints.
     * @return TaskVelocity variable that contains the linear and angular velocities of the
     *         end-effector or the custom elements.
     * @return std::nullopt on failure.
     */
    virtual std::optional<crf::utility::types::TaskVelocity> getVelocity(
        const crf::utility::types::JointPositions& jointPositions,
        const crf::utility::types::JointVelocities& jointVelocities) = 0;

    /**
     * @brief Function to get the end-effector acceleration knowing the joint positions, velocities
     *        and accelerations of the robot arm.
     *
     * @param jointPositions are the specified position values of the robot joints.
     * @param jointVelocities are the specified velocity values of the robot joints.
     * @param jointAccelerations are the specified acceleration values of the robot joints.
     * @return TaskAcceleration variable that contains the linear and angular accelerations of the
     *         end-effector or the custom elements.
     * @return std::nullopt on failure
     */
    virtual std::optional<crf::utility::types::TaskAcceleration> getAcceleration(
        const crf::utility::types::JointPositions& jointPositions,
        const crf::utility::types::JointVelocities& jointVelocities,
        const crf::utility::types::JointAccelerations& jointAccelerations) = 0;
};

}  // namespace crf::control::forwardkinematics
