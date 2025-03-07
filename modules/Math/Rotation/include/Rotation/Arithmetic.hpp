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
 * @ingroup group_rotation_arithmetic
 * @brief Takes quaternion representations of the parameters 'rotation1' and 'rotation2',
 * multiplies them in order: rotation1*rotation2 and returns the result as the Rotation object
 * in the RotationRepresentation::Quaternion representation.
 */
Rotation multiply(const Rotation& rotation1, const Rotation& rotation2);

/**
 * @ingroup group_rotation_arithmetic
 * @brief Takes quaternion representaion of the parameter 'rotation', inverts it and returns
 * the result as the Rotation object in the RotationRepresentation::Quaternion representation.
 */
Rotation invert(const Rotation& rotation);

/**
 * @ingroup group_rotation_arithmetic
 * @brief Returns angular velocity, traveling with wich will go from identity rotation
 * to the parameter 'rotation' in time 1.
 */
Eigen::Vector3d angularVelocityFromRotation(const Rotation& rotation);

/**
 * @ingroup group_rotation_arithmetic
 * @brief Returns rotation to which traveling with the parameter 'angularVelocity' will go
 * in time 1.
 * @param accuracy -- Minimal norm of the angular velocity, below which it will be treated as
 * a zero vector.
 */
Rotation rotationFromAngularVelocity(
    const Eigen::Vector3d& angularVelocity,
    const double& accuracy = 1e-14);

}  // namespace crf::math::rotation
