/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Comparison.hpp"

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::Rotation;

using crf::math::rotation::areAlmostEqual;

int main() {
    /**
     * Sample representations and rotations.
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
    Eigen::Quaterniond quaternion2(
        {0.3232781413160111, 0.5504467585659593, 0.7697351635084868, 0.0027179753598398});
    Eigen::Matrix3d matrix2(
        {{-0.1849992186629872, 0.8456391273700271, 0.5006693073825711},
         {0.8491537754599140, 0.3940019571883435, -0.3517105675892164},
         {-0.4946849044758272, 0.3600790524212897, -0.7909677119144169}});
    Eigen::AngleAxisd angleAxis2(
        2.4832094549611980,
        Eigen::Vector3d({0.5816806901502302, 0.8134121496309343, 0.0028722010957822}));
    CardanXYZ cardanXYZ2({2.7143908348394090, 0.5174722288733832, 1.7853074093465211});
    EulerZXZ eulerZXZ2({-0.9415926535897930, 2.4831853071795869, 0.9584073464102065});
    Rotation rotationQuaternion1(quaternion1);
    Rotation rotationAngleAxis1(angleAxis1);
    Rotation rotationQuaternion2(quaternion2);

    /**
     * Accuracy is defaulted to 1e-12. In all the types it can be passed as follows:
    */
    areAlmostEqual(quaternion1, quaternion2, 1e-7);  // false

    /**
     * Comparison of quaternions by the function crf::math:rotation::areAlmostEqual,
     * gives true on, up to the accuracy (default 1e-12), the pair of the same quaternions
     * and the pair of quaternion q and quaternion -q.
    */
    areAlmostEqual(quaternion1, quaternion1);       // true
    areAlmostEqual(quaternion1, minusQuaternion1);  // true
    areAlmostEqual(quaternion1, quaternion2);       // false

    /**
     * Comparison of matrices.
    */
    areAlmostEqual(matrix1, matrix1);  // true
    areAlmostEqual(matrix1, matrix2);  // false

    /**
     * Comparison of AngleAxis is done between angles of both arguments and
     * axes scaled by the angles.
    */
    areAlmostEqual(angleAxis1, angleAxis1);  // true
    areAlmostEqual(
        Eigen::AngleAxisd(0.0, Eigen::Vector3d({1.0, 0.0, 0.0})),
        Eigen::AngleAxisd(0.0, Eigen::Vector3d({0.0, 1.0, 0.0})));  // true
    areAlmostEqual(angleAxis1, angleAxis2);                         // false

    /**
     * Comparison of CardanXYZ.
    */
    areAlmostEqual(cardanXYZ1, cardanXYZ1);  // true
    areAlmostEqual(cardanXYZ1, cardanXYZ2);  // false

    /**
     * Comparison of EulerZXZ.
    */
    areAlmostEqual(eulerZXZ1, eulerZXZ1);  // true
    areAlmostEqual(eulerZXZ1, eulerZXZ2);  // false

    /**
     * Comparison of Rotation. Can be done regardless of representations.
    */
    areAlmostEqual(rotationQuaternion1, rotationAngleAxis1);   // true
    areAlmostEqual(rotationQuaternion1, rotationQuaternion2);  // false

    return 0;
}
