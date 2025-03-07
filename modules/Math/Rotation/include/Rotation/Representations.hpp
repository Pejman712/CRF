/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <iomanip>

#include "Rotation/EulerAngles.hpp"

namespace crf::math::rotation {

/**
 * @ingroup group_rotation_representations
 * @brief enum class listing different representations of rotations
 */
enum class RotationRepresentation {
    Quaternion = 0,

    Matrix = 1,

    AngleAxis = 2,

    CardanXYZ = 3,

    EulerZXZ = 4
};

using OrientationRepresentation = RotationRepresentation;

/**
 * @ingroup group_rotation_representations
 * @brief Creates display string of the parameter 'rotationRepresentation' and writes it
 *        to the parameter 'os'.
*/
std::ostream& operator<<(std::ostream& os, const RotationRepresentation& rotationRepresentation);

}  // namespace crf::math::rotation
