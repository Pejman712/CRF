/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "Rotation/Rotation.hpp"

using crf::math::rotation::Rotation;
using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

namespace crf::math::distancemeasures {

/**
 * @ingroup group_distance_measures_rotation
 * @brief Distance measure calculated by using quaternion representation.
 */
Eigen::Vector<double, 3> byQuaternion(
    const Rotation& rotation1,
    const Rotation& rotation2);

/**
 * @ingroup group_distance_measures_rotation
 * @brief Distance measure calculated by using matrix representation.
 */
Eigen::Vector<double, 3> byRotationMatrix(
    const Rotation& rotation1,
    const Rotation& rotation2);

/**
 * @ingroup group_distance_measures_rotation
 * @brief Distance measure calculated as coordinate-wise substraction.
 */
Eigen::Vector<double, 3> byCardanXYZ(
    const Rotation& rotation1,
    const Rotation& rotation2);

/**
 * @ingroup group_distance_measures_rotation
 * @brief Distance measure calculated as coordinate-wise substraction.
 */
Eigen::Vector<double, 3> byEulerZXZ(
    const Rotation& rotation1,
    const Rotation& rotation2);

}  // namespace crf::math::distancemeasures
