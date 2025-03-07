/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#pragma once

#include <vector>

#include <Eigen/Dense>

#include "InverseKinematics/ResultFlags.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

namespace crf::control::inversekinematics {

/**
 * @brief Struct for storing the results of inverse kinematics
 */

class ResultsIK {
 public:
    ResultsIK();
    ResultsIK(const ResultsIK&);
    ~ResultsIK();

    /**
     * @brief Assignment operator
     * 
     * @param other ResultsIK to assign to this object
     * @return ResultsIK& reference to this object
     */
    ResultsIK& operator=(const ResultsIK& other);

    /**
     * @brief Store in the struct the desired position of the end-effector
     *
     * @param Desired position of the end-effector
     */
    void zDesired(TaskPose input);
    /**
     * @brief Return the desired position of the end-effector stored in the struct
     *
     * @return Desired position of the end-effector
     */
    TaskPose zDesired() const;

    /**
     * @brief Store in the struct the desired velocity of the end-effector
     *
     * @param Desired velocity of the end-effector
     */
    void zdDesired(TaskVelocity input);

    /**
     * @brief Return the desired velocity of the end-effector stored in the struct
     *
     * @return Desired velocity of the end-effector
     */
    TaskVelocity zdDesired() const;

    /**
     * @brief Store in the struct the desired acceleration of the end-effector
     *
     * @param Desired acceleration of the end-effector
     */
    void zddDesired(TaskAcceleration input);
    /**
     * @brief Return the desired acceleration of the end-effector stored in the struct
     *
     * @return Desired acceleration of the end-effector
     */
    TaskAcceleration zddDesired() const;

    /**
     * @brief Store in the struct the obtained positions of the joints
     *
     * @param Obtained joints positions
     */
    void qResult(JointPositions input);
    /**
     * @brief Return the obtained positions of the joints stored in the struct
     *
     * @return Obtained joints positions
     */
    JointPositions qResult() const;

    /**
     * @brief Store in the struct the obtained velocities of the joints
     *
     * @param Obtained joints velocities
     */
    void qdResult(JointVelocities input);
    /**
     * @brief Return the obtained velocities of the joints stored in the struct
     *
     * @return Obtained joints velocities
     */
    JointVelocities qdResult() const;

    /**
     * @brief Store in the struct the obtained accelerations of the joints
     *
     * @param Obtained joints accelerations
     */
    void qddResult(JointAccelerations input);
    /**
     * @brief Return the obtained accelerations of the joints stored in the struct
     *
     * @return Obtained joints accelerations
     */
    JointAccelerations qddResult() const;

    /**
     * @brief Store in the struct the information about the result obtained
     *
     * @param The corresponding flag
     */
    void flag(ResultFlags input);
    /**
     * @brief Return the information about the result obtained
     *
     * @return The corresponding flag
     */
    ResultFlags flag() const;

    /**
     * @brief Store in the struct the error between the real position of the end-efector and the
     *        desired one.
     *
     * @param The calculated error.
     */
    void zError(std::vector<double> input);
    /**
     * @brief Return the error between the real position of the end-efector and the
     *        desired one 
     *
     * @return The calculated error.
     */
    std::vector<double> zError() const;

    /**
     * @brief Store in the struct the kinematic manipulability
     *
     * @param The kinematic manipulability
     */
    void kinematicManipulability(double input);
    /**
     * @brief Return the kinematic manipulability stored in the struct
     *
     * @return The kinematic manipulability
     */
    double kinematicManipulability() const;

    /**
     * @brief Store in the struct the gradients of the penalties
     *
     * @param The gradients of the penalties
     */
    void penaltyGradients(Eigen::MatrixXd input);
    /**
     * @brief Return the gradients of the penalties stored in the struct
     *
     * @return The gradients of the penalties
     */
    Eigen::MatrixXd penaltyGradients() const;

 private:
    TaskPose zDesired_;
    TaskVelocity zdDesired_;
    TaskAcceleration zddDesired_;
    JointPositions qResult_;
    JointVelocities qdResult_;
    JointAccelerations qddResult_;
    ResultFlags flag_;
    std::vector<double> zError_;
    double kinematicManipulability_;
    Eigen::MatrixXd penaltyGradients_;
};

}  // namespace crf::control::inversekinematics
