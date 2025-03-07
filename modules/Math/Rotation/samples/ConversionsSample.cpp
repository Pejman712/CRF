/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Conversions.hpp"

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;

using crf::math::rotation::quaternionFromAngleAxis;
using crf::math::rotation::quaternionFromMatrix;
using crf::math::rotation::quaternionFromCardanXYZ;
using crf::math::rotation::quaternionFromEulerZXZ;

using crf::math::rotation::angleAxisFromQuaternion;
using crf::math::rotation::angleAxisFromMatrix;
using crf::math::rotation::angleAxisFromCardanXYZ;
using crf::math::rotation::angleAxisFromEulerZXZ;

using crf::math::rotation::matrixFromQuaternion;
using crf::math::rotation::matrixFromAngleAxis;
using crf::math::rotation::matrixFromCardanXYZ;
using crf::math::rotation::matrixFromEulerZXZ;

using crf::math::rotation::cardanXYZFromQuaternion;
using crf::math::rotation::cardanXYZFromAngleAxis;
using crf::math::rotation::cardanXYZFromMatrix;
using crf::math::rotation::cardanXYZFromEulerZXZ;

using crf::math::rotation::eulerZXZFromQuaternion;
using crf::math::rotation::eulerZXZFromAngleAxis;
using crf::math::rotation::eulerZXZFromMatrix;
using crf::math::rotation::eulerZXZFromCardanXYZ;

int main() {
    /**
     * Sample rotation representations.
    */
    Eigen::Quaterniond quaternion1(
        {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799});
    Eigen::Quaterniond minusQuaternion1(
        {-0.5505666516902112, 0.6362152372281484, -0.3613804315900029, -0.4018839604029799});
    Eigen::Matrix3d matrix1(
        {{0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
         {-0.0173036611331485, -0.1325610914209059, 0.9910237839490472},
         {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}});
    Eigen::AngleAxisd angleAxis1(
        1.9755068924749106,
        Eigen::Vector3d({-0.7621249848254679, 0.4328991822668132, 0.4814185346426796}));
    CardanXYZ cardanXYZ1({1.4, 2.0, 3.1});
    EulerZXZ eulerZXZ1({1.1471123464639490, -1.6415867259093058, 0.1139727950424842});

    /**
     * There is a conversion function between each pair of representations.
    */
    quaternionFromAngleAxis(angleAxis1);
    quaternionFromMatrix(matrix1);
    quaternionFromCardanXYZ(cardanXYZ1);
    quaternionFromEulerZXZ(eulerZXZ1);

    angleAxisFromQuaternion(quaternion1);
    angleAxisFromMatrix(matrix1);
    angleAxisFromCardanXYZ(cardanXYZ1);
    angleAxisFromEulerZXZ(eulerZXZ1);

    matrixFromQuaternion(quaternion1);
    matrixFromAngleAxis(angleAxis1);
    matrixFromCardanXYZ(cardanXYZ1);
    matrixFromEulerZXZ(eulerZXZ1);

    cardanXYZFromQuaternion(quaternion1);
    cardanXYZFromAngleAxis(angleAxis1);
    cardanXYZFromMatrix(matrix1);
    cardanXYZFromEulerZXZ(eulerZXZ1);

    eulerZXZFromQuaternion(quaternion1);
    eulerZXZFromAngleAxis(angleAxis1);
    eulerZXZFromMatrix(matrix1);
    eulerZXZFromCardanXYZ(cardanXYZ1);

    return 0;
}
