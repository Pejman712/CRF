/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <math.h>

#include "DistanceMeasures/TaskPose.hpp"

using crf::utility::types::TaskPose;

int main(int argc, char** argv) {
    // Example 1
    TaskPose desiredPose(
        {0.2, 0.3, 0.4}, Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7e-4);
    std::cout << "desiredPose: " << desiredPose << std::endl;
    TaskPose currentPose(
        {0.2, 0.3, 0.4}, Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7e-4);
    std::cout << "currentPose: " << currentPose << std::endl;

    Eigen::Vector<double, 6> cardanDistanceMeasure =
        crf::math::distancemeasures::byCardanXYZ(currentPose, desiredPose);
    std::cout << "distance measure computed with cardan method = [" << cardanDistanceMeasure(0)
              << ", " << cardanDistanceMeasure(1) << ", " << cardanDistanceMeasure(2) << ", "
              << cardanDistanceMeasure(3) << ", " << cardanDistanceMeasure(4) << ", "
              << cardanDistanceMeasure(5) << "]" << std::endl;

    Eigen::Vector<double, 6> eulerDistanceMeasure =
        crf::math::distancemeasures::byEulerZXZ(currentPose, desiredPose);
    std::cout << "distance measure computed with eulerZXZ method = [" << eulerDistanceMeasure(0)
              << ", " << eulerDistanceMeasure(1) << ", " << eulerDistanceMeasure(2) << ", "
              << eulerDistanceMeasure(3) << ", " << eulerDistanceMeasure(4) << ", "
              << eulerDistanceMeasure(5) << "]" << std::endl;

    Eigen::Vector<double, 6> axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentPose, desiredPose);
    std::cout << "distance measure computed with axisAngle method = ["
              << axisAngleDistanceMeasure(0) << ", " << axisAngleDistanceMeasure(1) << ", "
              << axisAngleDistanceMeasure(2) << ", " << axisAngleDistanceMeasure(3) << ", "
              << axisAngleDistanceMeasure(4) << ", " << axisAngleDistanceMeasure(5) << "]"
              << std::endl;

    Eigen::Vector<double, 6> quaternionDistanceMeasure =
        crf::math::distancemeasures::byQuaternion(currentPose, desiredPose);
    std::cout << "distance measure computed with quaternion method = ["
              << quaternionDistanceMeasure(0) << ", " << quaternionDistanceMeasure(1) << ", "
              << quaternionDistanceMeasure(2) << ", " << quaternionDistanceMeasure(3) << ", "
              << quaternionDistanceMeasure(4) << ", " << quaternionDistanceMeasure(5) << "]"
              << std::endl;

    // Example 2
    desiredPose = TaskPose(
        {0, 0, 0},
        crf::math::rotation::CardanXYZ({10 * M_PI / 180, 5 * M_PI / 180, -10 * M_PI / 180}));
    std::cout << "desiredPose: " << desiredPose << std::endl;
    currentPose = TaskPose({0, 0, 0}, crf::math::rotation::CardanXYZ({0, 0, 0}));
    std::cout << "currentPose: " << currentPose << std::endl;

    cardanDistanceMeasure = crf::math::distancemeasures::byCardanXYZ(currentPose, desiredPose);
    std::cout << "distance measure computed with cardan method = [" << cardanDistanceMeasure(0)
              << ", " << cardanDistanceMeasure(1) << ", " << cardanDistanceMeasure(2) << ", "
              << cardanDistanceMeasure(3) << ", " << cardanDistanceMeasure(4) << ", "
              << cardanDistanceMeasure(5) << "]" << std::endl;

    eulerDistanceMeasure = crf::math::distancemeasures::byEulerZXZ(currentPose, desiredPose);
    std::cout << "distance measure computed with eulerZXZ method = [" << eulerDistanceMeasure(0)
              << ", " << eulerDistanceMeasure(1) << ", " << eulerDistanceMeasure(2) << ", "
              << eulerDistanceMeasure(3) << ", " << eulerDistanceMeasure(4) << ", "
              << eulerDistanceMeasure(5) << "]" << std::endl;

    axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentPose, desiredPose);
    std::cout << "distance measure computed with axisAngle method = ["
              << axisAngleDistanceMeasure(0) << ", " << axisAngleDistanceMeasure(1) << ", "
              << axisAngleDistanceMeasure(2) << ", " << axisAngleDistanceMeasure(3) << ", "
              << axisAngleDistanceMeasure(4) << ", " << axisAngleDistanceMeasure(5) << "]"
              << std::endl;

    quaternionDistanceMeasure = crf::math::distancemeasures::byQuaternion(currentPose, desiredPose);
    std::cout << "distance measure computed with quaternion method = ["
              << quaternionDistanceMeasure(0) << ", " << quaternionDistanceMeasure(1) << ", "
              << quaternionDistanceMeasure(2) << ", " << quaternionDistanceMeasure(3) << ", "
              << quaternionDistanceMeasure(4) << ", " << quaternionDistanceMeasure(5) << "]"
              << std::endl;
}
