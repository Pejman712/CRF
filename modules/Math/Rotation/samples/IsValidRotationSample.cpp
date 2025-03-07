/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/IsValidRotation.hpp"

using crf::math::rotation::isUnitaryQuaternion;
using crf::math::rotation::isRotationMatrix;

int main() {
    /**
     * Quaternion can be checked, if it is a unitary quaternion up to a specified accuracy.
    */

    Eigen::Quaterniond quaternion1({1.0, 0.0, 0.0, 0.0});
    /**
     * This quaternion is unitary up to the floating point number accuracy:
    */
    isUnitaryQuaternion(quaternion1, 1e-15);  // true

    Eigen::Quaterniond quaternion2({0.644, 0.368, 0.644, 0.184});
    /**
     * This quaternion is unitary up to the 1e-4 accuracy,
     * but not up to the 1e-5 accuracy:
    */
    isUnitaryQuaternion(quaternion2, 1e-4);  // true
    isUnitaryQuaternion(quaternion2, 1e-5);  // false

    /**
     * Matrix can be checked, if it is a rotation matrix up to a specified accuracy.
    */

    Eigen::Matrix3d matrix1(Eigen::Matrix3d::Identity());
    /**
     * This matrix is a rotation matrix up to the floating point number accuracy:
    */
    isRotationMatrix(matrix1, 1e-15);  // true

    Eigen::Matrix3d matrix2;
    matrix2 << 0.4157869320692789,  -0.9023592869214287, -0.1134413699981963,
               -0.0173036611331485, -0.1325610914208059,  0.9910237839490472,
               -0.9092974268256818, -0.4100917877109334, -0.0707312888348917;
    /**
     * This matrix is a rotation matrix up to the 1e-14 accuracy,
     * but not up to the 1e-15 accuracy:
    */
    isRotationMatrix(matrix2, 1e-14);  // true
    isRotationMatrix(matrix2, 1e-15);  // false

    return 0;
}
