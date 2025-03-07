/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include "Types/TaskTypes/TaskPose.hpp"

namespace crf::utility::types {

TaskPose::TaskPose() :
    position_({0.0, 0.0, 0.0}),
    orientation_() {
}

TaskPose::TaskPose(const Eigen::Matrix4d& homogeneousTransformationMatrix, const double& accuracy) :
    position_(homogeneousTransformationMatrix.block<3, 1>(0, 3)),
    orientation_(homogeneousTransformationMatrix.block<3, 3>(0, 0), accuracy) {
    Eigen::RowVector4d correctFourthRow({0, 0, 0, 1});
    bool isFourthRowValid =
        ((homogeneousTransformationMatrix.block<1, 4>(3, 0) - correctFourthRow)
             .lpNorm<Eigen::Infinity>() < accuracy);
    if (!(isFourthRowValid)) {
        Eigen::Vector4d actualFourthRow = homogeneousTransformationMatrix.block<1, 4>(3, 0);
        Eigen::Vector4d difference =
            homogeneousTransformationMatrix.block<1, 4>(3, 0) - correctFourthRow;
        std::stringstream throwStringStream;
        throwStringStream << std::setprecision(16);
        throwStringStream
            << "TaskPose(Eigen::Matrix4d): Matrix provided in the constructor has not correct "
               "up to accuracy forth row. It is\n"
            << actualFourthRow << ",\nwich differs from\n"
            << correctFourthRow << ",\nby the vector\n " << difference << " of norm "
            << difference.norm() << " but the accuracy is" << accuracy << ".";
        throw std::invalid_argument(throwStringStream.str());
    }
}

TaskPose::TaskPose(const Eigen::Vector3d& position, const Orientation& orientation) :
    position_(position),
    orientation_(orientation) {
}

TaskPose::TaskPose(
    const Eigen::Vector3d& position,
    const Eigen::Quaternion<double>& quaternion,
    const double& accuracy) :
    position_(position),
    orientation_(quaternion, accuracy) {
}

TaskPose::TaskPose(
    const Eigen::Vector3d& position,
    const Eigen::Matrix3d& rotationMatrix,
    const double& accuracy) :
    position_(position),
    orientation_(rotationMatrix, accuracy) {
}

TaskPose::TaskPose(const Eigen::Vector3d& position, const Eigen::AngleAxisd& angleAxis) :
    position_(position),
    orientation_(angleAxis) {
}

TaskPose::TaskPose(const Eigen::Vector3d& position, const CardanXYZ& cardanXYZ) :
    position_(position),
    orientation_(cardanXYZ) {
}

TaskPose::TaskPose(const Eigen::Vector3d& position, const EulerZXZ& eulerZXZ) :
    position_(position),
    orientation_(eulerZXZ) {
}

void TaskPose::setHomogeneousTransformationMatrix(
    const Eigen::Matrix4d& homogeneousTransformationMatrix,
    const double& accuracy) {
    Eigen::RowVector4d correctFourthRow({0, 0, 0, 1});
    bool isFourthRowValid =
        ((homogeneousTransformationMatrix.block<1, 4>(3, 0) - correctFourthRow)
             .lpNorm<Eigen::Infinity>() < accuracy);
    if (!(isFourthRowValid)) {
        Eigen::Vector4d difference =
            homogeneousTransformationMatrix.block<1, 4>(3, 0) - correctFourthRow;
        Eigen::Vector4d actualFourthRow = homogeneousTransformationMatrix.block<1, 4>(3, 0);
        std::stringstream throwStringStream;
        throwStringStream << std::setprecision(16);
        throwStringStream << "TaskPose::sethomogeneousTransformationMatrix: Matrix provided in the "
                             "constructor has not correct up to accuracy forth row. It is\n"
                          << actualFourthRow << ",\nwich differs from\n"
                          << correctFourthRow << ",\nby the vector\n " << difference << " of norm "
                          << difference.norm() << " but the accuracy is" << accuracy << ".";
        throw std::invalid_argument(throwStringStream.str());
    }
    position_ = homogeneousTransformationMatrix.block<3, 1>(0, 3);
    orientation_.setMatrix(homogeneousTransformationMatrix.block<3, 3>(0, 0), accuracy);
}

void TaskPose::setPosition(const Eigen::Vector3d& position) {
    position_ = position;
}

void TaskPose::setOrientation(const Orientation& orientation) {
    orientation_ = orientation;
}

void TaskPose::setQuaternion(const Eigen::Quaterniond& quaternion, const double& accuracy) {
    orientation_.setQuaternion(quaternion, accuracy);
}

void TaskPose::setRotationMatrix(const Eigen::Matrix3d& rotationMatrix, const double& accuracy) {
    orientation_.setMatrix(rotationMatrix, accuracy);
}

void TaskPose::setAngleAxis(const Eigen::AngleAxisd& angleAxis) {
    orientation_.setAngleAxis(angleAxis);
}

void TaskPose::setCardanXYZ(const CardanXYZ& cardanXYZ) {
    orientation_.setCardanXYZ(cardanXYZ);
}

void TaskPose::setEulerZXZ(const EulerZXZ& eulerZXZ) {
    orientation_.setEulerZXZ(eulerZXZ);
}

Eigen::Matrix4d TaskPose::getHomogeneousTransformationMatrix() const {
    Eigen::Matrix4d homogeneousTransformationMatrix(Eigen::Matrix4d::Zero());
    homogeneousTransformationMatrix.block<1, 4>(3, 0) = Eigen::RowVector4d({0, 0, 0, 1});
    homogeneousTransformationMatrix.block<3, 1>(0, 3) = position_;
    homogeneousTransformationMatrix.block<3, 3>(0, 0) = orientation_.getMatrix();
    return homogeneousTransformationMatrix;
}

Eigen::Vector3d TaskPose::getPosition() const {
    return position_;
}

Orientation TaskPose::getOrientation() const {
    return orientation_;
}

Eigen::Quaterniond TaskPose::getQuaternion() const {
    return orientation_.getQuaternion();
}

Eigen::Matrix3d TaskPose::getRotationMatrix() const {
    return orientation_.getMatrix();
}

Eigen::AngleAxisd TaskPose::getAngleAxis() const {
    return orientation_.getAngleAxis();
}

CardanXYZ TaskPose::getCardanXYZ() const {
    return orientation_.getCardanXYZ();
}

EulerZXZ TaskPose::getEulerZXZ() const {
    return orientation_.getEulerZXZ();
}

OrientationRepresentation TaskPose::getOrientationRepresentation() const {
    return orientation_.getRepresentation();
}

std::ostream& operator<<(std::ostream& os, const TaskPose& taskPose) {
    os << std::fixed << std::setprecision(15);
    os << "Position:\n";
    os << taskPose.getPosition() << "\n";
    os << "Orientation:\n";
    os << taskPose.getOrientation();
    return os;
}

}  // namespace crf::utility::types
