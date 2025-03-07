/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#pragma once

#include <tuple>

#include "Types/Types.hpp"
#include "InverseKinematics/ResultFlags.hpp"
#include "InverseKinematics/ResultsIK.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

namespace crf::control::inversekinematics {

class IOpenLoopInverseKinematics {
 public:
    virtual ~IOpenLoopInverseKinematics() = default;

    /**
     * @brief Function to get the joint positions, velocities and accelerations of the robot arm
     *        knowing the end-effector position, velocity and acceleration and the current
     *        joints position of the robot.
     * 
     * @param qAttr is a jointPositions variable that represents the desired joints position for
     *        the this type of objective function. qAttr must have the same size as q and qd.
     *        It will only affect in case of using the DesiredJointPositions Objective Function.
     *        qAttr can contain std::nan("") in the elements that doesn't have a desired position.
     * @param qRobotActual is the set of current joints position.
     * @param z is the selected end-effector position and orientation coordinates (or the custom
     *        elements).
     * @param zd is the selected end-effector linear and angular velocities (or the custom
     *        elements).
     * @param zd is the selected end-effector linear and angular accelerations (or the custom
     *        elements).
     * @return The obtained values of the position, velocity and acceleration of the joints
     *         and a message about the obtained results.
     */
    virtual std::tuple<JointPositions, JointVelocities, JointAccelerations, ResultFlags> getResults(
        const JointPositions& qAttr,
        const JointPositions& qRobotActual,
        const TaskPose& z,
        const TaskVelocity& zd = TaskVelocity(),
        const TaskAcceleration& zdd = TaskAcceleration()) = 0;

    /**
     * @brief Function to get all the values of the interesting parameters and results.
     * 
     * @param qAttr is a jointPositions variable that represents the desired joints position for
     *        the this type of objective function. qAttr must have the same size as q and qd.
     *        It will only affect in case of using the DesiredJointPositions Objective Function.
     *        qAttr can contain std::nan("") in the elements that doesn't have a desired position.
     * @param qRobotActual is the set of current joints position.
     * @param z is the selected end-effector position and orientation coordinates (or the custom
     *        elements).
     * @param zd is the selected end-effector linear and angular velocities (or the custom
     *        elements).
     * @param zd is the selected end-effector linear and angular accelerations (or the custom
     *        elements).
     * @return struct of all the interesting parameters and results.
     */
    virtual ResultsIK getExtendedResults(
        const JointPositions& qAttr,
        const JointPositions& qRobotActual,
        const TaskPose& z,
        const TaskVelocity& zd = TaskVelocity(),
        const TaskAcceleration& zdd = TaskAcceleration()) = 0;
};

}  // namespace crf::control::inversekinematics
