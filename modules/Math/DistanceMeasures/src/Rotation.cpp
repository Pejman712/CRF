/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "DistanceMeasures/Rotation.hpp"

namespace crf::math::distancemeasures {

Eigen::Vector<double, 3> byQuaternion(
    const Rotation& rotation1,
    const Rotation& rotation2) {

    Eigen::Vector<double, 3> distanceMeasure;

    Eigen::Quaterniond quaternion1 = rotation1.getQuaternion();
    Eigen::Quaterniond quaternion2 = rotation2.getQuaternion();
    double scaledAngle1 = quaternion1.w();
    double scaledAngle2 = quaternion2.w();
    Eigen::Vector3d scaledAxis1({quaternion1.x(), quaternion1.y(), quaternion1.z()});
    Eigen::Vector3d scaledAxis2({quaternion2.x(), quaternion2.y(), quaternion2.z()});
    distanceMeasure.segment<3>(0) =
        scaledAngle1 * scaledAxis2 - scaledAngle2 * scaledAxis1 - scaledAxis2.cross(scaledAxis1);

    return distanceMeasure;
}

Eigen::Vector<double, 3> byRotationMatrix(
    const Rotation& rotation1,
    const Rotation& rotation2) {
    Eigen::Matrix3d matrix1 = rotation1.getMatrix();
    Eigen::Matrix3d matrix2 = rotation2.getMatrix();

    Eigen::Vector<double, 3> error;

    Eigen::Vector3d n2 = matrix2.col(0).head<3>();
    Eigen::Vector3d n1 = matrix1.col(0).head<3>();
    Eigen::Vector3d s2 = matrix2.col(1).head<3>();
    Eigen::Vector3d s1 = matrix1.col(1).head<3>();
    Eigen::Vector3d a2 = matrix2.col(2).head<3>();
    Eigen::Vector3d a1 = matrix1.col(2).head<3>();

    error.segment<3>(0) = 0.5 * (n1.cross(n2) + s1.cross(s2) + a1.cross(a2));

    return error;
}

Eigen::Vector<double, 3> byCardanXYZ(
    const Rotation& rotation1,
    const Rotation& rotation2) {
    CardanXYZ cardan1 = rotation1.getCardanXYZ();
    CardanXYZ cardan2 = rotation2.getCardanXYZ();

    Eigen::Vector<double, 3> distanceMeasure;
    for (int i = 0; i < 3; i++) {
        distanceMeasure(i) = cardan2[i] - cardan1[i];
    }

    return distanceMeasure;
}

Eigen::Vector<double, 3> byEulerZXZ(
    const Rotation& rotation1,
    const Rotation& rotation2) {
    EulerZXZ euler1 = rotation1.getEulerZXZ();
    EulerZXZ euler2 = rotation2.getEulerZXZ();

    Eigen::Vector<double, 3> distanceMeasure;
    for (int i = 0; i < 3; i++) {
        distanceMeasure(i) = euler2[i] - euler1[i];
    }

    return distanceMeasure;
}

}  // namespace crf::math::distancemeasures
