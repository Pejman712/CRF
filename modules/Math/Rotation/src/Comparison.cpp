/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Rotation/Comparison.hpp"

namespace crf::math::rotation {

bool areAlmostEqual(
    const Eigen::Quaterniond& quaternion1,
    const Eigen::Quaterniond& quaternion2,
    const double& accuracy) {
    const double w1 = quaternion1.w();
    const double x1 = quaternion1.x();
    const double y1 = quaternion1.y();
    const double z1 = quaternion1.z();
    const double w2 = quaternion2.w();
    const double x2 = quaternion2.x();
    const double y2 = quaternion2.y();
    const double z2 = quaternion2.z();
    const bool quaternion1EqualsQuaternion2 =
        (abs(w1 - w2) < accuracy && abs(x1 - x2) < accuracy && abs(y1 - y2) < accuracy &&
         abs(z1 - z2) < accuracy);
    const bool quaternion1EqualsMinusQuaternion2 =
        (abs(w1 - -(w2)) < accuracy && abs(x1 - (-x2)) < accuracy && abs(y1 - (-y2)) < accuracy &&
         abs(z1 - (-z2)) < accuracy);
    return quaternion1EqualsQuaternion2 || quaternion1EqualsMinusQuaternion2;
}

bool areAlmostEqual(
    const Eigen::Matrix3d& matrix1,
    const Eigen::Matrix3d& matrix2,
    const double& accuracy) {
    return ((matrix1 - matrix2).lpNorm<Eigen::Infinity>() < accuracy);
}

bool areAlmostEqual(
    const Eigen::AngleAxisd& angleAxis1,
    const Eigen::AngleAxisd& angleAxis2,
    const double& accuracy) {
    const double angle1 = angleAxis1.angle();
    const Eigen::Vector3d axis1 = angleAxis1.axis();
    const Eigen::Vector3d scaledAxis1 = axis1 * angle1;
    const double angle2 = angleAxis2.angle();
    const Eigen::Vector3d axis2 = angleAxis2.axis();
    const Eigen::Vector3d scaledAxis2 = axis2 * angle2;
    const bool angle1EqualsAngle2 = abs(angle1 - angle2) < accuracy;
    const bool axis1EqualsAxis2 =
        (scaledAxis1 - scaledAxis2).lpNorm<Eigen::Infinity>() < accuracy;
    return angle1EqualsAngle2 && axis1EqualsAxis2;
}

bool areAlmostEqual(
    const CardanXYZ& cardanXYZ1,
    const CardanXYZ& cardanXYZ2,
    const double& accuracy) {
    for (std::size_t i = 0; i < 3; i++) {
        if (abs(cardanXYZ1[i] - cardanXYZ2[i]) >= accuracy) {
            return false;
        }
    }
    return true;
}

bool areAlmostEqual(const EulerZXZ& eulerZXZ1, const EulerZXZ& eulerZXZ2, const double& accuracy) {
    for (std::size_t i = 0; i < 3; i++) {
        if (abs(eulerZXZ1[i] - eulerZXZ2[i]) >= accuracy) {
            return false;
        }
    }
    return true;
}

bool areAlmostEqual(const Rotation& rotation1, const Rotation& rotation2, const double& accuracy) {
    return areAlmostEqual(rotation1.getQuaternion(), rotation2.getQuaternion(), accuracy);
}

}  // namespace crf::math::rotation
