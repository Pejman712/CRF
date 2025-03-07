/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <vector>
#include <tuple>
#include <utility>

#include "EventLogger/EventLogger.hpp"
#include "InverseKinematics/IClosedLoopInverseKinematics.hpp"
#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "DistanceMeasures/TaskPose.hpp"
#include "InverseKinematics/OptOLIK/OptOLIK.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskSpace;
namespace crf::control::inversekinematics {

/**
 * @brief It returns the joints position and velocity from specified values of the end-effector
 *        positions and velocities; executing an optimized close loop.
 * 
 * @param qInitial is the set of beginning joints position.
 * @param time used in the Euler integral to obtain the joints position from the joints velocity.
 *        It must match with the time expended between one desired position and the following one.
 * @param robotArmConf object that contains the forward kinematics, the jacobian and the lengths
 *        between each joint of the robot, among other parameters.
 * @param diagW is the main diagonal of the weight matrix. The size must match with the number of
 *        joints of the robot. The identity matrix of weight will not affect the result. Joints with
 *        big weight will have less movement than joints with less weight. 
 * @param objFun is a vector that contains the smart pointers to the selected objective functions.
 *        In case of no objective function used, the size of the vector must be 0.
 * @param tolerance is set of tolerances usually for the positions in x, y and z coordinates and the
 *        cardan angles (it can be constructed also in custom mode, taking care that, in this case,
 *        the TaskPose DistanceMeasures will be the substract of both TaskPoses).
 *        The introduced values must be positive, even if they represent the positive and negative
 *        tolerance.
 * @param K is the gain that will be applied in the computed error.
 * @param kinManip0 is the initial manipulability.
 * @param alpha0 is the initial damping factor.
 */

class OptCLIK : public IClosedLoopInverseKinematics {
 public:
    OptCLIK(
        JointPositions qInitial,
        std::chrono::microseconds time,
        std::shared_ptr<crf::actuators::robot::RobotConfiguration> robotArmConf,
        std::vector<double> diagW,
        std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFun =  // NOLINT
            std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>>(0),  // NOLINT
        TaskPose tolerance =
            TaskPose({0.0001, 0.0001, 0.0001},
                     crf::math::rotation::CardanXYZ({0.0001, 0.0001, 0.0001})),
        double K = 500,
        double kinManip0 = 0.001,
        double alpha0 = 0.001);
    ~OptCLIK() override;

    std::tuple<JointPositions, JointVelocities, JointAccelerations, ResultFlags> getResults(
        const JointPositions& qAttr,
        const TaskPose& z,
        const TaskVelocity& zd,
        const TaskAcceleration& zdd = TaskAcceleration()) override;
    ResultsIK getExtendedResults(
        const JointPositions& qAttr,
        const TaskPose& z,
        const TaskVelocity& zd,
        const TaskAcceleration& zdd = TaskAcceleration()) override;
    void updateInitialJointPositions(const JointPositions& qActual) override;

 private:
    utility::logger::EventLogger logger_;
    JointPositions qInitial_;
    std::chrono::duration<double> time_;
    TaskSpace taskSpace_;
    double K_;
    unsigned int numberOfJoints_;
    Eigen::Vector<double, 6> zErrorTolerance_;
    std::shared_ptr<crf::control::forwardkinematics::IForwardKinematics> forwardKinematics_;
    std::shared_ptr<crf::control::inversekinematics::OptOLIK> optOLIK_;
};

}  // namespace crf::control::inversekinematics
