/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "Rotation/RotationClass.hpp"

namespace crf::math::rotation {

/**
 * @ingroup group_rotation_comparison
 * @brief Returns true iff quaternions represent the same rotation
 * up to the parameter accuracy.
 */
bool areAlmostEqual(
    const Eigen::Quaterniond& quaternion1,
    const Eigen::Quaterniond& quaternion2,
    const double& accuracy = 1e-12);

/**
 * @ingroup group_rotation_comparison
 * @brief Takes abs of maximum of the coordinates of matrix1 - matrix2
 * and compares if it is less than the parameter accuracy.
 */
bool areAlmostEqual(
    const Eigen::Matrix3d& matrix1,
    const Eigen::Matrix3d& matrix2,
    const double& accuracy = 1e-12);

/**
 * @ingroup group_rotation_comparison
 * @brief Compares angles and axes scaled by the corresponding angles of the parameters if they
 * are the same up to the parameter accuracy.
 */
bool areAlmostEqual(
    const Eigen::AngleAxisd& angleAxis1,
    const Eigen::AngleAxisd& angleAxis2,
    const double& accuracy = 1e-12);

/**
 * @ingroup group_rotation_comparison
 * @brief Compares on each coordinate if parameters are the same
 * up to the parameter accuracy.
 */
bool areAlmostEqual(
    const CardanXYZ& cardanXYZ1,
    const CardanXYZ& cardanXYZ2,
    const double& accuracy = 1e-12);

/**
 * @ingroup group_rotation_comparison
 * @brief Compares on each coordinate if parameters are the same
 * up to the parameter accuracy.
 */
bool areAlmostEqual(
    const EulerZXZ& eulerZXZ1,
    const EulerZXZ& eulerZXZ2,
    const double& accuracy = 1e-12);

/**
 * @ingroup group_rotation_comparison
 * @brief Function always compares rotations in quaternion representation.
 * Function takes quaternion representation of the parameters rotation1 and rotation2,
 * then checks if they are the same as a rotations, up
 * to parameter accuracy.
 */
bool areAlmostEqual(
    const Rotation& rotation1,
    const Rotation& rotation2,
    const double& accuracy = 1e-12);

}  // namespace crf::math::rotation
