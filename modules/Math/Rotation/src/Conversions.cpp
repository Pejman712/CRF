/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <Eigen/Dense>

#include "Rotation/Conversions.hpp"

namespace crf::math::rotation {

Eigen::Quaterniond quaternionFromMatrix(const Eigen::Matrix3d& matrix) {
    return Eigen::Quaterniond(matrix);
}

Eigen::Quaterniond quaternionFromAngleAxis(const Eigen::AngleAxisd& angleAxis) {
    return Eigen::Quaterniond(angleAxis);
}

Eigen::Quaterniond quaternionFromCardanXYZ(const CardanXYZ& cardanXYZ) {
    return Eigen::Quaterniond(matrixFromCardanXYZ(cardanXYZ));
}

Eigen::Quaterniond quaternionFromEulerZXZ(const EulerZXZ& eulerZXZ) {
    return Eigen::Quaterniond(matrixFromEulerZXZ(eulerZXZ));
}

Eigen::Matrix3d matrixFromQuaternion(const Eigen::Quaterniond& quaternion) {
    return quaternion.toRotationMatrix();
}

Eigen::Matrix3d matrixFromAngleAxis(const Eigen::AngleAxisd& angleAxis) {
    return angleAxis.toRotationMatrix();
}

Eigen::Matrix3d matrixFromCardanXYZ(const CardanXYZ& cardanXYZ) {
    return Eigen::Matrix3d(
        Eigen::AngleAxisd(cardanXYZ[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(cardanXYZ[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(cardanXYZ[0], Eigen::Vector3d::UnitX()));
}

Eigen::Matrix3d matrixFromEulerZXZ(const EulerZXZ& eulerZXZ) {
    return Eigen::Matrix3d(
        Eigen::AngleAxisd(eulerZXZ[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(eulerZXZ[1], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(eulerZXZ[0], Eigen::Vector3d::UnitZ()));
}

Eigen::AngleAxisd angleAxisFromQuaternion(const Eigen::Quaterniond& quaternion) {
    return Eigen::AngleAxisd(quaternion);
}

Eigen::AngleAxisd angleAxisFromMatrix(const Eigen::Matrix3d& matrix) {
    return Eigen::AngleAxisd(matrix);
}

Eigen::AngleAxisd angleAxisFromCardanXYZ(const CardanXYZ& cardanXYZ) {
    return Eigen::AngleAxisd(matrixFromCardanXYZ(cardanXYZ));
}

Eigen::AngleAxisd angleAxisFromEulerZXZ(const EulerZXZ& eulerZXZ) {
    return Eigen::AngleAxisd(matrixFromEulerZXZ(eulerZXZ));
}

CardanXYZ cardanXYZFromQuaternion(const Eigen::Quaterniond& quaternion) {
    Eigen::Matrix3d matrix(quaternion.toRotationMatrix());
    return cardanXYZFromMatrix(matrix);
}

CardanXYZ cardanXYZFromMatrix(const Eigen::Matrix3d& matrix) {
    Eigen::Vector3d cardanXYZEigen = matrix.eulerAngles(2, 1, 0);
    CardanXYZ cardanXYZ;
    cardanXYZ[2] = cardanXYZEigen(0);
    cardanXYZ[1] = cardanXYZEigen(1);
    cardanXYZ[0] = cardanXYZEigen(2);
    return cardanXYZ;
}

CardanXYZ cardanXYZFromAngleAxis(const Eigen::AngleAxisd& angleAxis) {
    return cardanXYZFromMatrix(angleAxis.toRotationMatrix());
}

CardanXYZ cardanXYZFromEulerZXZ(const EulerZXZ& eulerZXZ) {
    return cardanXYZFromMatrix(matrixFromEulerZXZ(eulerZXZ));
}

EulerZXZ eulerZXZFromQuaternion(const Eigen::Quaterniond& quaternion) {
    Eigen::Matrix3d matrix(quaternion.toRotationMatrix());
    return eulerZXZFromMatrix(matrix);
}

EulerZXZ eulerZXZFromMatrix(const Eigen::Matrix3d& matrix) {
    Eigen::Vector3d eulerZXZEigen = matrix.eulerAngles(2, 0, 2);
    EulerZXZ eulerZXZ;
    eulerZXZ[2] = eulerZXZEigen(0);
    eulerZXZ[1] = eulerZXZEigen(1);
    eulerZXZ[0] = eulerZXZEigen(2);
    return eulerZXZ;
}

EulerZXZ eulerZXZFromAngleAxis(const Eigen::AngleAxisd& angleAxis) {
    return eulerZXZFromMatrix(angleAxis.toRotationMatrix());
}

EulerZXZ eulerZXZFromCardanXYZ(const CardanXYZ& cardanXYZ) {
    return eulerZXZFromMatrix(matrixFromCardanXYZ(cardanXYZ));
}

Eigen::Quaterniond quaternionFromArray(const std::array<double, 4>& array) {
    return Eigen::Quaterniond({array[0], array[1], array[2], array[3]});
}

std::array<double, 4> arrayFromQuaternion(const Eigen::Quaterniond& quaternion) {
    return std::array<double, 4>({quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()});
}

Eigen::Matrix3d matrixFromArray(const std::array<std::array<double, 3>, 3>& array) {
    Eigen::Matrix3d matrix;
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            matrix(i, j) = array[i][j];
        }
    }
    return matrix;
}

std::array<std::array<double, 3>, 3> arrayFromMatrix(const Eigen::Matrix3d& matrix) {
    std::array<std::array<double, 3>, 3> array;
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            array[i][j] = matrix(i, j);
        }
    }
    return array;
}

Eigen::AngleAxisd angleAxisFromArray(const std::array<double, 4>& array) {
    return Eigen::AngleAxisd(array[0], Eigen::Vector3d({array[1], array[2], array[3]}));
}

std::array<double, 4> arrayFromAngleAxis(const Eigen::AngleAxisd& angleAxis) {
    return std::array<double, 4>(
        {angleAxis.angle(), angleAxis.axis()(0), angleAxis.axis()(1), angleAxis.axis()(2)});
}

}  // namespace crf::math::rotation
