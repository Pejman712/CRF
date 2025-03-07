/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include "Rotation/Representations.hpp"

#include <Eigen/Dense>

namespace crf::math::rotation {
/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining quaternion from matrix.
 * Does not check whether input matrix is a rotation matrix.
 */
Eigen::Quaterniond quaternionFromMatrix(const Eigen::Matrix3d& matrix);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining quaternion from angleAxis.
 */
Eigen::Quaterniond quaternionFromAngleAxis(const Eigen::AngleAxisd& angleAxis);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining quaternion from cardanXYZ.
 */
Eigen::Quaterniond quaternionFromCardanXYZ(const CardanXYZ& cardanXYZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining quaternion from eulerZXZ.
 */
Eigen::Quaterniond quaternionFromEulerZXZ(const EulerZXZ& eulerZXZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining matrix from quaternion.
 * Does not check whether input quaternion is an unitary quaternion.
 */
Eigen::Matrix3d matrixFromQuaternion(const Eigen::Quaterniond& quaternion);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining matrix from angleAxis.
 */
Eigen::Matrix3d matrixFromAngleAxis(const Eigen::AngleAxisd& angleAxis);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining matrix from cardanXYZ.
 */
Eigen::Matrix3d matrixFromCardanXYZ(const CardanXYZ& cardanXYZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining matrix from eulerZXZ.
 */
Eigen::Matrix3d matrixFromEulerZXZ(const EulerZXZ& eulerZXZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining angleAxis from quaternion.
 * Does not check whether input quaternion is an unitary quaternion.
 */
Eigen::AngleAxisd angleAxisFromQuaternion(const Eigen::Quaterniond& quaternion);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining angleAxis from matrix.
 * Does not check whether input matrix is a rotation matrix.
 */
Eigen::AngleAxisd angleAxisFromMatrix(const Eigen::Matrix3d& matrix);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining angleAxis from cardanXYZ.
 */
Eigen::AngleAxisd angleAxisFromCardanXYZ(const CardanXYZ& cardanXYZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining angleAxis from eulerZXZ.
 */
Eigen::AngleAxisd angleAxisFromEulerZXZ(const EulerZXZ& eulerZXZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining cardanXYZ from quaternion.
 * Does not check whether input quaternion is an unitary quaternion.
 */
CardanXYZ cardanXYZFromQuaternion(const Eigen::Quaterniond& quaternion);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining cardanXYZ from matrix.
 * Does not check whether input matrix is a rotation matrix.
 */
CardanXYZ cardanXYZFromMatrix(const Eigen::Matrix3d& matrix);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining cardanXYZ from angleAxis.
 */
CardanXYZ cardanXYZFromAngleAxis(const Eigen::AngleAxisd& angleAxis);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining cardanXYZ from eulerZXZ.
 */
CardanXYZ cardanXYZFromEulerZXZ(const EulerZXZ& eulerZXZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining eulerZXZ from quaternion.
 * Does not check whether input quaternion is an unitary quaternion.
 */
EulerZXZ eulerZXZFromQuaternion(const Eigen::Quaterniond& quaternion);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining eulerZXZ from matrix.
 * Does not check whether input matrix is a rotation matrix.
 */
EulerZXZ eulerZXZFromMatrix(const Eigen::Matrix3d& matrix);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining eulerZXZ from angleAxis.
 */
EulerZXZ eulerZXZFromAngleAxis(const Eigen::AngleAxisd& angleAxis);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining eulerZXZ from cardanXYZ.
 */
EulerZXZ eulerZXZFromCardanXYZ(const CardanXYZ& cardanXYZ);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining Eigen Quaterniond from std array.
 */
Eigen::Quaterniond quaternionFromArray(const std::array<double, 4>& array);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining std array from Eigen Quaterniond.
 */
std::array<double, 4> arrayFromQuaternion(const Eigen::Quaterniond& quaternion);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining Eigen Matrix3d from std array.
 */
Eigen::Matrix3d matrixFromArray(const std::array<std::array<double, 3>, 3>& array);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining std array from Eigen Matrix3d.
 */
std::array<std::array<double, 3>, 3> arrayFromMatrix(const Eigen::Matrix3d& matrix);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining Eigen AngleAxisd from std array.
 */
Eigen::AngleAxisd angleAxisFromArray(const std::array<double, 4>& array);

/**
 * @ingroup group_rotation_conversions
 * @brief Function for obtaining std array from Eigen AngleAxisd.
 */
std::array<double, 4> arrayFromAngleAxis(const Eigen::AngleAxisd& angleAxis);

}  // namespace crf::math::rotation
