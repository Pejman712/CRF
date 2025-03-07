/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <math.h>

#include "DistanceMeasures/Rotation.hpp"

using crf::math::rotation::Rotation;

int main(int argc, char** argv) {
    // Example 1
    Rotation desiredRotation(Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7e-4);
    std::cout << "desiredRotation: " << desiredRotation << std::endl;
    Rotation currentRotation(Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7e-4);
    std::cout << "currentRotation: " << currentRotation << std::endl;

    Eigen::Vector<double, 3> cardanDistanceMeasure =
        crf::math::distancemeasures::byCardanXYZ(currentRotation, desiredRotation);
    std::cout << "distance measure computed with cardan method = [" << cardanDistanceMeasure(0)
              << ", " << cardanDistanceMeasure(1) << ", " << cardanDistanceMeasure(2) << ", "
              << "]" << std::endl;

    Eigen::Vector<double, 3> eulerDistanceMeasure =
        crf::math::distancemeasures::byEulerZXZ(currentRotation, desiredRotation);
    std::cout << "distance measure computed with eulerZXZ method = [" << eulerDistanceMeasure(0)
              << ", " << eulerDistanceMeasure(1) << ", " << eulerDistanceMeasure(2) << ", "
              << "]" << std::endl;

    Eigen::Vector<double, 3> axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentRotation, desiredRotation);
    std::cout << "distance measure computed with axisAngle method = ["
              << axisAngleDistanceMeasure(0) << ", " << axisAngleDistanceMeasure(1) << ", "
              << axisAngleDistanceMeasure(2) << ", "
              << "]" << std::endl;

    Eigen::Vector<double, 3> quaternionDistanceMeasure =
        crf::math::distancemeasures::byQuaternion(currentRotation, desiredRotation);
    std::cout << "distance measure computed with quaternion method = ["
              << quaternionDistanceMeasure(0) << ", " << quaternionDistanceMeasure(1) << ", "
              << quaternionDistanceMeasure(2) << "]" << std::endl;

    // Example 2
    desiredRotation = Rotation(
        crf::math::rotation::CardanXYZ({10 * M_PI / 180, 5 * M_PI / 180, -10 * M_PI / 180}));
    std::cout << "desiredRotation: " << desiredRotation << std::endl;
    currentRotation = Rotation(crf::math::rotation::CardanXYZ({0, 0, 0}));
    std::cout << "currentRotation: " << currentRotation << std::endl;

    cardanDistanceMeasure =
        crf::math::distancemeasures::byCardanXYZ(currentRotation, desiredRotation);
    std::cout << "distance measure computed with cardan method = [" << cardanDistanceMeasure(0)
              << ", " << cardanDistanceMeasure(1) << ", " << cardanDistanceMeasure(2) << "]"
              << std::endl;

    eulerDistanceMeasure =
        crf::math::distancemeasures::byEulerZXZ(currentRotation, desiredRotation);
    std::cout << "distance measure computed with eulerZXZ method = [" << eulerDistanceMeasure(0)
              << ", " << eulerDistanceMeasure(1) << ", " << eulerDistanceMeasure(2) << "]"
              << std::endl;

    axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentRotation, desiredRotation);
    std::cout << "distance measure computed with axisAngle method = ["
              << axisAngleDistanceMeasure(0) << ", " << axisAngleDistanceMeasure(1) << ", "
              << axisAngleDistanceMeasure(2) << "]" << std::endl;

    quaternionDistanceMeasure =
        crf::math::distancemeasures::byQuaternion(currentRotation, desiredRotation);
    std::cout << "distance measure computed with quaternion method = ["
              << quaternionDistanceMeasure(0) << ", " << quaternionDistanceMeasure(1) << ", "
              << quaternionDistanceMeasure(2) << "]" << std::endl;
}
