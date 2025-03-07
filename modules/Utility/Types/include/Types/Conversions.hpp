/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include "Rotation/Conversions.hpp"

#include <Eigen/Dense>
#include <vector>

using crf::math::rotation::quaternionFromArray;
using crf::math::rotation::arrayFromQuaternion;
using crf::math::rotation::angleAxisFromArray;
using crf::math::rotation::arrayFromAngleAxis;

namespace crf::utility::types {

/**
 * @ingroup group_types_conversions
 * Function for obtaining std vector from Eigen VectorXd.
 */
std::vector<double> stdVectorFromEigenVector(const Eigen::VectorXd& eigenVector);

/**
 * @ingroup group_types_conversions
 * Function for obtaining Eigen VectorXd from std vector.
 */
Eigen::VectorXd eigenVectorFromStdVector(const std::vector<double>& stdVector);

/**
 * @ingroup group_types_conversions
 * Function for obtaining std array from Eigen Vector.
 * Supports sizes 3, 4 and 6.
 */
template <size_t size>
std::array<double, size> stdArrayFromEigenVector(
    const Eigen::Vector<double, static_cast<int>(size)>& eigenVector);

/**
 * @ingroup group_types_conversions
 * Function for obtaining Eigen Vector from std array.
 * Supports sizes 3, 4 and 6.
 */
template <size_t size>
Eigen::Vector<double, static_cast<int>(size)> eigenVectorFromStdArray(
    const std::array<double, size>& stdArray);

}  // namespace crf::utility::types
