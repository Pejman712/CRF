/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <Eigen/Dense>
#include <iostream>

#include "Rotation/Representations.hpp"

namespace crf::math::rotation {

/**
 * @ingroup group_rotation_is_valid_rotation
 * @brief Checks if the parameter quaternion is a unitary quaternon up to the parameter accuracy.
 */
bool isUnitaryQuaternion(const Eigen::Quaterniond& quaternion, const double& accuracy = 1e-12);

/**
 * @ingroup group_rotation_is_valid_rotation
 * @brief Checks if the parameter matrix is a rotation matrix up to the parameter accuracy.
 */
bool isRotationMatrix(const Eigen::Matrix3d& matrix, const double& accuracy = 1e-12);

}  // namespace crf::math::rotation
