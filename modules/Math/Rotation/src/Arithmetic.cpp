/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Rotation/Arithmetic.hpp"

namespace crf::math::rotation {

Rotation multiply(const Rotation& rotation1, const Rotation& rotation2) {
    Rotation product;
    product.setQuaternion(rotation1.getQuaternion() * rotation2.getQuaternion());
    return product;
}

Rotation invert(const Rotation& rotation) {
    Rotation inverse;
    inverse.setQuaternion(rotation.getQuaternion().inverse());
    return inverse;
}

Eigen::Vector3d angularVelocityFromRotation(const Rotation& rotation) {
    Eigen::AngleAxisd angleAxis = rotation.getAngleAxis();
    Eigen::Vector3d angularVelocity = angleAxis.angle() * angleAxis.axis();
    return angularVelocity;
}

Rotation rotationFromAngularVelocity(
    const Eigen::Vector3d& angularVelocity,
    const double& accuracy) {
    double angle = angularVelocity.norm();
    if (angle < accuracy) {
        return Rotation(Eigen::AngleAxisd(0.0, Eigen::Vector3d(1.0, 0.0, 0.0)));
    }
    Eigen::Vector3d axis = angularVelocity / angle;
    Eigen::AngleAxisd angleAxis(angle, axis);
    return Rotation(angleAxis);
}

}  // namespace crf::math::rotation
