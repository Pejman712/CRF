/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#pragma once

#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "ForwardKinematics/IForwardKinematics.hpp"
#include "KinematicChain/IKinematicChain.hpp"
#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"
#include "KinematicChain/DescriptionTypes.hpp"

#include "Types/Types.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

namespace crf::control::forwardkinematics {

/**
 * @ingroup group_kinematic_chain_forward_kinematics
 * @brief It returns the end-effector position from specified values of the joint positions,
 *        from a loaded kinematic chain object of a specific robot.
 *
 * @param kinChain kinematic chain object.
 */
class KinChainForwardKinematics : public IForwardKinematics {
 public:
    explicit KinChainForwardKinematics(
        const std::shared_ptr<crf::math::kinematicchain::IKinematicChain>& kinChain);
    ~KinChainForwardKinematics() override;

    /**
     * @brief Function to get the end-effector position knowing the joint positions of the robot
     *        arm.
     *
     * @param jointPositions are the specified position values of the joints.
     * @return TaskPose variable that contains the position and orientation of the
     * end-effector or the custom task space elemts.
     * @return std::nullopt on failure.
     */
    std::optional<TaskPose> getPose(const JointPositions& jointPositions) override;

    /**
     * @brief (NOT IMPLEMENTED) Function to get the end-effector velocity knowing the joint
     *        positions and velocities of the robot arm.
     *
     * @param jointPositions are the specified position values of the robot joints.
     * @param jointVelocities are the specified velocity values of the robot joints.
     * @throw runtime_error "Not implemented".
     */
    std::optional<TaskVelocity> getVelocity(
        const crf::utility::types::JointPositions& jointPositions,
        const crf::utility::types::JointVelocities& jointVelocities) override;

    /**
     * @brief (NOT IMPLEMENTED) Function to get the end-effector acceleration knowing the joint
     *        positions, velocities and accelerations of the robot arm.
     *
     * @param jointPositions are the specified position values of the robot joints.
     * @param jointVelocities are the specified velocity values of the robot joints.
     * @param jointAccelerations are the specified acceleration values of the robot joints.
     * @throw runtime_error "Not implemented".
     */
    std::optional<TaskAcceleration> getAcceleration(
        const crf::utility::types::JointPositions& jointPositions,
        const crf::utility::types::JointVelocities& jointVelocities,
        const crf::utility::types::JointAccelerations& jointAccelerations) override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<math::kinematicchain::IKinematicChain> kinChain_;
};

}  // namespace crf::control::forwardkinematics
