/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include "InverseKinematics/OptOLIK/OptOLIK.hpp"

namespace crf::control::inversekinematics {

OptOLIK::OptOLIK(
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> robotArmConf,
    std::vector<double> diagW,
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFun,  // NOLINT
    double kinManip0,
    double alpha0):
    logger_("OptOLIK"),
    objectiveFunction_(objFun),
    kinManip0_(kinManip0),
    alpha0_(alpha0) {
    logger_->debug("CTor");

    if (kinManip0_ <= 0.0 || alpha0_ <= 0.0) {
        throw std::invalid_argument("The input parameters K, kinManip0 and alpha0 cannot be 0 or "
            "negative");
    }

    /**
     * @todo Change from config to jacobian directly in the CTor
     */
    jacobian_ = robotArmConf->getJacobian();
    numberOfJoints_ = robotArmConf->getJointSpaceDoF();
    taskSpace_ = robotArmConf->getTaskSpace();
    diagW_.resize(numberOfJoints_);

    if (diagW.size() != numberOfJoints_) {
        throw std::invalid_argument(
            "Size of the W diagonal is not equal to number of joints");
    }

    for (unsigned int i = 0; i < numberOfJoints_; i++) {
        if (diagW[i] <= 0.0) {
            throw std::invalid_argument("The elements of the diagonal W cannot be 0 or negative");
        }
        diagW_(i) = diagW[i];
    }
}

OptOLIK::~OptOLIK() {
    logger_->debug("DTor");
}

std::tuple<JointPositions, JointVelocities, JointAccelerations, ResultFlags> OptOLIK::getResults(
    const JointPositions& qAttr,
    const JointPositions& qRobotActual,
    const TaskPose& z,
    const TaskVelocity& zd,
    const TaskAcceleration& zdd) {
    logger_->debug("getResults");

    ResultsIK extendedResult(getExtendedResults(qAttr, qRobotActual, TaskPose(), zd));

    return std::make_tuple(
        extendedResult.qResult(), extendedResult.qdResult(), extendedResult.qddResult(),
        extendedResult.flag());
}

ResultsIK OptOLIK::getExtendedResults(
    const JointPositions& qAttr,
    const JointPositions& qRobotActual,
    const TaskPose& z,
    const TaskVelocity& zd,
    const TaskAcceleration& zdd) {
    logger_->debug("getExtendedResults");
    if (qRobotActual.size() != numberOfJoints_ || qAttr.size() != numberOfJoints_) {
        throw std::invalid_argument("Input JointPositions parameter not valid");
    }
    unsigned int dimensionsTaskSpace = taskSpace_.dimension();
    if (jacobian_->rows() != dimensionsTaskSpace) {
        throw std::invalid_argument("Number of Task space dimensions not matching");
    }
    ResultFlags flag = crf::control::inversekinematics::ResultFlags::success;
    Eigen::MatrixXd jacobian = jacobian_->evaluate(qRobotActual);
    Eigen::MatrixXd jacobianT = jacobian.transpose();
    // Jacobian can not return nullopt because the size of qRobotActual was checked before.
    double manipKin = jacobian_->getKinematicManipulability(qRobotActual);
    double alpha;
    if (kinManip0_ > manipKin) {
        alpha = alpha0_ * pow(1 - (manipKin/kinManip0_), 2.0);
        flag = crf::control::inversekinematics::ResultFlags::lowManipulability;
        logger_->warn("The robot is close to a singularity");
    } else {
        alpha = 0;
    }
    // Constructing the matrixW from the diagonalW
    Eigen::MatrixXd matrixW = Eigen::MatrixXd::Identity(numberOfJoints_, numberOfJoints_);
    matrixW.diagonal() = diagW_;
    Eigen::MatrixXd matrixWInverse = matrixW.inverse();
    // Size of jacobianPInv: (numberOfJoints_, number of space dimensions = jacobian.rows())
    Eigen::MatrixXd jacobianPInv = matrixWInverse * jacobianT * (jacobian * matrixWInverse *
        jacobianT + alpha * Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows())).inverse();

    // Computing the total of the gradients
    Eigen::MatrixXd additionOfPenaltyGradients = Eigen::MatrixXd::Zero(numberOfJoints_, 1);
    if (objectiveFunction_.size() != 0) {
        for (unsigned int h = 0; h < objectiveFunction_.size(); h++) {
            Eigen::MatrixXd penaltyGradient = objectiveFunction_[h]->getGradient(
                qRobotActual, qAttr);
            for (unsigned int i = 0; i < penaltyGradient.rows(); i++) {
                for (unsigned int j = 0; j < penaltyGradient.cols(); j++) {
                    if (std::isnan(penaltyGradient(i, j))) penaltyGradient(i, j) = 0.0;
                }
            }
            additionOfPenaltyGradients += penaltyGradient;
        }
    }
    // Computing the joints velocity
    Eigen::Matrix<double, Eigen::Dynamic, 6> reductionMatrix = taskSpace_.getNoRowsMatrix();
    JointVelocities qd(jacobianPInv * reductionMatrix * zd.raw() +
        (Eigen::MatrixXd::Identity(numberOfJoints_, numberOfJoints_) - jacobianPInv * jacobian) *
        matrixWInverse * additionOfPenaltyGradients);
    for (unsigned int i = 0; i < numberOfJoints_; i++) {
        if (std::isnan(qd[i])) {
            flag = crf::control::inversekinematics::ResultFlags::workspaceViolation;
            logger_->error("The desired end-effector is outside the robot workspace");
            break;
        }
    }
    ResultsIK extendedResult;
    extendedResult.zdDesired(zd);
    extendedResult.qdResult(qd);
    extendedResult.kinematicManipulability(manipKin);
    extendedResult.penaltyGradients(additionOfPenaltyGradients);
    extendedResult.flag(flag);
    return extendedResult;
}

}  // namespace crf::control::inversekinematics
