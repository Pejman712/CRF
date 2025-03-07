/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <Eigen/Dense>

#include "Rotation/IsValidRotation.hpp"

namespace crf::math::rotation {

bool isUnitaryQuaternion(const Eigen::Quaterniond& quaternion, const double& accuracy) {
    return abs(quaternion.norm() - 1) < accuracy;
}

bool isRotationMatrix(const Eigen::Matrix3d& matrix, const double& accuracy) {
    if ((matrix * matrix.transpose() - Eigen::Matrix3d::Identity()).lpNorm<Eigen::Infinity>() >=
        accuracy) {
        return false;
    }
    if (abs(matrix.determinant() - 1) >= accuracy) {
        return false;
    }
    return true;
}

}  // namespace crf::math::rotation
