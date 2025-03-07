/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/RotationClass.hpp"

using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::Rotation;
using crf::math::rotation::RotationRepresentation;
using crf::math::rotation::Orientation;
using crf::math::rotation::OrientationRepresentation;

int main() {
    /**
     * Identities in all supported representations as an example of objects from
     * which Rotaiton class can be constructed or their value can be set.
     */

    Eigen::Quaterniond quaternion({1.0, 0.0, 0.0, 0.0});
    Eigen::Matrix3d matrix(Eigen::Matrix3d::Identity());
    Eigen::AngleAxisd angleAxis(1.0, Eigen::Vector3d({1.0, 0.0, 0.0}));
    CardanXYZ cardanXYZ({0.0, 0.0, 0.0});
    EulerZXZ eulerZXZ({0.0, 0.0, 0.0});

    /**
     * Default constructor.
     * Creates an identity rotation in the quaternion representation.
     */
    Rotation rotation0;

    /**
     * Constructors from different representations.
     * Create corresponding rotation in corresponding representation.
     * Quaternion and matrix need to be, respectively, unitary quaternion and rotation matrix.
     */
    Rotation rotation1(quaternion);
    Rotation rotation2(matrix);
    Rotation rotation3(angleAxis);
    Rotation rotation4(cardanXYZ);
    Rotation rotation5(eulerZXZ);
    /**
     * Constructors from quaternion and matrix can be set with the parameter accuracy,
     * adjusting how accurate quaternion or matrix should be to representing the rotation.
     * The default is 1e-12.
    */
    Rotation rotation1_1(quaternion, 1e-7);
    Rotation rotation2_1(matrix, 1e-7);

    /**
     * Setters
     * Sets the value of the rotation and changes to the representation of the parameter.
     * Quaternion and matrix need to be, respectively, unitary quaternion and rotation matrix.
    */
    rotation1.setEulerZXZ(eulerZXZ);
    rotation2.setMatrix(matrix);
    rotation5.setAngleAxis(angleAxis);
    rotation3.setCardanXYZ(cardanXYZ);
    rotation4.setQuaternion(quaternion);
    /**
     * Setters from quaternion and matrix can be set with the parameter accuracy,
     * adjusting how accurate quaternion or matrix should be to representing the rotation.
     * The default is 1e-12.
    */
    rotation2.setMatrix(matrix, 1e-7);
    rotation4.setQuaternion(quaternion, 1e-7);

    /**
     * Getters
     * Gets desired representation regardless of the current representation of the Rotation
     * object.
     * Const member functions. Does not change anything inside the class, nor the value,
     * nor the representation.
    */
    matrix = rotation3.getMatrix();
    quaternion = rotation2.getQuaternion();
    cardanXYZ = rotation1.getCardanXYZ();
    angleAxis = rotation4.getAngleAxis();
    eulerZXZ = rotation5.getEulerZXZ();
    RotationRepresentation representation1 = rotation1.getRepresentation();

    /**
     * There is also an alias to call Rotation and RotationRepresentation,
     * respectively Orientation and OrientationRepresentation when it is
     * more suitable in the context.
    */
    Orientation orientation1(quaternion);
    OrientationRepresentation representation2 = orientation1.getRepresentation();

    /**
     * Print to stream operator. Prints rotation in accordance to its representation.
    */
    std::cout << rotation0 << std::endl;
    std::cout << rotation1 << std::endl;
    std::cout << rotation2 << std::endl;
    std::cout << rotation3 << std::endl;
    std::cout << rotation4 << std::endl;
    std::cout << rotation5 << std::endl;

    return 0;
}
