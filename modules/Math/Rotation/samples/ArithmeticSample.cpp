/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Arithmetic.hpp"

using crf::math::rotation::Rotation;
using crf::math::rotation::angularVelocityFromRotation;
using crf::math::rotation::rotationFromAngularVelocity;

int main() {
    /**
     * Sample rotations and angular velocities
    */
    Rotation rotation1(Eigen::Quaterniond{
        0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799});
    Rotation rotation2(Eigen::AngleAxisd(
        2.4832094549611980,
        Eigen::Vector3d({0.5816806901502302, 0.8134121496309343, 0.0028722010957822})));
    Eigen::Vector3d angularVelocity1({-1.505583160450048, 0.855195318314842, 0.951045633351785});

    /**
     * Multiplication.
    */
    Rotation product = multiply(rotation1, rotation2);

    /**
     * Inversion.
    */
    Rotation inverse = invert(rotation1);

    /**
     * Angular velocity from rotation
    */
    Eigen::Vector3d angularVelocity = angularVelocityFromRotation(rotation1);

    /**
     * Rotation from angular velocity
    */
    Rotation rotation = rotationFromAngularVelocity(angularVelocity1);

    return 0;
}
