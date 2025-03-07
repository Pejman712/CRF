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

#include "Jacobian/IJacobian.hpp"
#include "InverseKinematics/IOpenLoopInverseKinematics.hpp"
#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "Robot/RobotConfiguration.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::math::jacobian::IJacobian;


namespace crf::control::inversekinematics {

/**
 * @brief It return the joints velocity from specified values of the end-effector velocities,
 *        knowing the actual joint positions.
 *        In this class, the z input of getResults and getExtendedResults doesn't interact inside
 *        the functions, so it can be set as TaskPose().
 * 
 * @param robotArmConf object that contains the forward kinematics, the jacobian and the lengths
 *        between each joint of the robot, among other parameters.
 * @param diagW is the main diagonal of the weight matrix. The size must match with the number of
 *        joints of the robot. The identity matrix of weight will not affect the result. Joints with
 *        big weight will have less movement than joints with less weight.
 * @param objFun is a vector that contains the smart pointers to the selected objective functions.
 *        In case of no objective function used, the size of the vector must be 0.
 * @param kinManip0 is the initial manipulability.
 * @param alpha0 is the initial damping factor.
 */

class OptOLIK : public IOpenLoopInverseKinematics {
 public:
    OptOLIK(
        std::shared_ptr<crf::actuators::robot::RobotConfiguration> robotArmConf,
        std::vector<double> diagW,
        std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFun =  // NOLINT
            std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>>(0),  // NOLINT
        double kinManip0 = 0.001,
        double alpha0 = 0.001);
    ~OptOLIK() override;

    std::tuple<JointPositions, JointVelocities, JointAccelerations, ResultFlags> getResults(
        const JointPositions& qAttr,
        const crf::utility::types::JointPositions& qRobotActual,
        const crf::utility::types::TaskPose& z,
        const crf::utility::types::TaskVelocity& zd,
        const crf::utility::types::TaskAcceleration& zdd = TaskAcceleration()) override;
    ResultsIK getExtendedResults(
        const JointPositions& qAttr,
        const crf::utility::types::JointPositions& qRobotActual,
        const crf::utility::types::TaskPose& z,
        const crf::utility::types::TaskVelocity& zd,
        const crf::utility::types::TaskAcceleration& zdd = TaskAcceleration()) override;

 private:
    utility::logger::EventLogger logger_;
    Eigen::VectorXd diagW_;
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objectiveFunction_;  // NOLINT
    TaskSpace taskSpace_;
    double kinManip0_;
    double alpha0_;
    unsigned int numberOfJoints_;
    std::shared_ptr<IJacobian> jacobian_;
};

}  // namespace crf::control::inversekinematics
