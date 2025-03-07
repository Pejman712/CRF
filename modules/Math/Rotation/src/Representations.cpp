/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include "Rotation/Conversions.hpp"

namespace crf::math::rotation {

std::ostream& operator<<(std::ostream& os, const RotationRepresentation& rotationRepresentation) {
    switch (rotationRepresentation) {
        case RotationRepresentation::Quaternion:
            os << "RotationRepresentation::Quaternion";
            break;
        case RotationRepresentation::Matrix:
            os << "RotationRepresentation::Matrix";
            break;
        case RotationRepresentation::AngleAxis:
            os << "RotationRepresentation::AngleAxis";
            break;
        case RotationRepresentation::CardanXYZ:
            os << "RotationRepresentation::CardanXYZ";
            break;
        case RotationRepresentation::EulerZXZ:
            os << "RotationRepresentation::EulerZXZ";
            break;
        default:
            throw std::logic_error("operator<<(std::ostream&, RotationRepressentation): "
                                   "rotationRepresentation invalid");
            break;
    }
    return os;
}

}  // namespace crf::math::rotation
