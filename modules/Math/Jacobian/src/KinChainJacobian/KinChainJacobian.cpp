/* © Copyright CERN 2023. All rights reserved. This software is released under a
 * CERN proprietary software license. Any permission to use it shall be granted
 * in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO 2023
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "Jacobian/KinChainJacobian/KinChainJacobian.hpp"

namespace crf::math::jacobian {

KinChainJacobian::KinChainJacobian(std::shared_ptr<IKinematicChain> kinChain,
    const TaskSpace& taskSpace) :
    kinChain_(kinChain),
    chainSize_(kinChain->getChainSize()),
    numWheels_(kinChain_->getNumWheels()),
    taskSpace_(taskSpace),
    logger_("KinChainJacobian") {
    logger_->debug("CTor");
    kinematicchain::DescriptionType kinChainType = kinChain_->getType();
    rowsJMatrix_ = taskSpace_.dimension();
    columnsJMatrix_ = chainSize_ + numWheels_;
    if (rowsJMatrix_ > columnsJMatrix_ ||
        (kinChainType != kinematicchain::DescriptionType::Arm &&
         rowsJMatrix_ + 1 > columnsJMatrix_)) {
        throw std::invalid_argument("Jacobian can't contain more rows than columns"
            "(more space dimensions than robot DOF)");
    }
    if (kinChainType == kinematicchain::DescriptionType::Platform ||
        kinChainType == kinematicchain::DescriptionType::Combined) {
        JMatrix_ = Eigen::MatrixXd(rowsJMatrix_, columnsJMatrix_);
        Eigen::MatrixXd Jplatform(rowsJMatrix_, numWheels_);

        Jplatform = platformJacobian();

        Eigen::MatrixXd Jmanipulator(rowsJMatrix_, chainSize_);
        JMatrix_ << Jplatform, Jmanipulator;
    } else {
        JMatrix_ = Eigen::MatrixXd(rowsJMatrix_, columnsJMatrix_);
    }
}

KinChainJacobian::~KinChainJacobian() {
    logger_->debug("DTor");
}

Eigen::MatrixXd KinChainJacobian::evaluate(const JointPositions& qValues) {
    logger_->debug("evaluate {}", qValues);
    if (qValues.size() != columnsJMatrix_) {
        throw std::invalid_argument(
            "KinChainJacobian - evaluate - The dimensions "
            "of the input joint position "
            "does not match the jacobian dimensions."
            "The dimension of q vector (" +
            std::to_string(qValues.size()) +
            ") is different than the length of vectors "
            "defined in the constructor (" +
            std::to_string(columnsJMatrix_) + ").");
    }

    Eigen::Matrix<double, Eigen::Dynamic, 6> reductionMatrix = taskSpace_.getNoRowsMatrix();
    Eigen::VectorXd JColSixDimensional(6, 1);
    Eigen::Vector3d In_J;
    Eigen::Vector3d Ir_JE;
    kinChain_->setJointPositions(qValues);
    for (unsigned int i = 0; i < chainSize_; i++) {
        int jointType = kinChain_->getJointType(i);
        JColSixDimensional = Eigen::VectorXd::Zero(6, 1);
        In_J = kinChain_->getAxis(kinematicchain::Axes::IJ, i);
        Ir_JE = kinChain_->getTranslation(kinematicchain::Translations::IJE, i);
        if (jointType == 1 || jointType == 2) {  // 1 for REVOLUTE, 2 for CONTINUOUS
            JColSixDimensional.head(3) = In_J.cross(Ir_JE);
            JColSixDimensional.tail<3>() = In_J;
        } else if (jointType == 3) {  // 3 for PRISMATIC
            JColSixDimensional.head(3) = In_J;
        } else {
            throw std::runtime_error("Unknown or unimplemented joint type.");
        }

        JMatrix_.col(i + numWheels_) = reductionMatrix * JColSixDimensional;
    }

    return JMatrix_;
}

double KinChainJacobian::getKinematicManipulability(const JointPositions& qValues) {
    logger_->debug("getKinematicManipulability");
    if (qValues.size() != columnsJMatrix_) {
        throw std::invalid_argument(
            "KinChainJacobian - getKinematicManipulability - "
            "The dimensions of the input "
            "joint position does not match the jacobian dimensions."
            "The dimension of q vector (" +
            std::to_string(qValues.size()) +
            ") is different than the length vectors "
            "defined in the constructor (" +
            std::to_string(columnsJMatrix_) + ").");
    }
    Eigen::MatrixXd j = evaluate(qValues);
    return std::sqrt((j * j.transpose()).determinant());
}

unsigned int KinChainJacobian::rows() const {
    return rowsJMatrix_;
}

unsigned int KinChainJacobian::cols() const {
    return columnsJMatrix_;
}

// Private

Eigen::MatrixXd KinChainJacobian::platformJacobian() {
    double platformL = kinChain_->getPlatformL();
    double platformW = kinChain_->getPlatformW();
    double wheelRadius = kinChain_->getWheelRadius();
    Eigen::Matrix<double, Eigen::Dynamic, 6> reductionMatrix = taskSpace_.getNoRowsMatrix();

    Eigen::MatrixXd JplatformSixDimensionalUnscaledUnoriented(6, numWheels_);
    Eigen::MatrixXd Jplatform(rowsJMatrix_, numWheels_);

    double absw = 2 / (platformL + platformW);  // absolute value of angular velocity
    Eigen::MatrixXd orientation = Eigen::MatrixXd::Zero(numWheels_, numWheels_);
    for (int wheelN = 0; wheelN < static_cast<int>(numWheels_); wheelN++) {
        orientation(wheelN, wheelN) = std::round(
            Eigen::Vector3d(0, 1, 0).dot(kinChain_->getAxis(kinematicchain::Axes::IW, wheelN)));
    }
    JplatformSixDimensionalUnscaledUnoriented << 1, 1, 1, 1,
                                                 1, -1, 1, -1,
                                                 0, 0, 0, 0,
                                                 0, 0, 0, 0,
                                                 0, 0, 0, 0,
                                                 absw, -absw, -absw, absw;

    Jplatform =
        wheelRadius / 4 * reductionMatrix * JplatformSixDimensionalUnscaledUnoriented * orientation;

    return Jplatform;
}

}  // namespace crf::math::jacobian
