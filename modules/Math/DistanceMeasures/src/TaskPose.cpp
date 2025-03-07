/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "DistanceMeasures/TaskPose.hpp"

namespace crf::math::distancemeasures {

Eigen::Vector<double, 6> byQuaternion(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2) {
    Eigen::Vector<double, 6> distanceMeasure;
    distanceMeasure.segment<3>(0) = taskPose2.getPosition() - taskPose1.getPosition();

    distanceMeasure.segment<3>(3) =
        byQuaternion(taskPose1.getOrientation(), taskPose2.getOrientation());

    return distanceMeasure;
}

Eigen::Vector<double, 6> byRotationMatrix(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2) {
    Eigen::Vector<double, 6> distanceMeasure;
    distanceMeasure.segment<3>(0) = taskPose2.getPosition() - taskPose1.getPosition();

    distanceMeasure.segment<3>(3) =
        byRotationMatrix(taskPose1.getOrientation(), taskPose2.getOrientation());

    return distanceMeasure;
}

Eigen::Vector<double, 6> byCardanXYZ(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2) {
    Eigen::Vector<double, 6> distanceMeasure;
    distanceMeasure.segment<3>(0) = taskPose2.getPosition() - taskPose1.getPosition();

    distanceMeasure.segment<3>(3) =
        byCardanXYZ(taskPose1.getOrientation(), taskPose2.getOrientation());

    return distanceMeasure;
}

Eigen::Vector<double, 6> byEulerZXZ(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2) {
    Eigen::Vector<double, 6> distanceMeasure;
    distanceMeasure.segment<3>(0) = taskPose2.getPosition() - taskPose1.getPosition();

    distanceMeasure.segment<3>(3) =
        byEulerZXZ(taskPose1.getOrientation(), taskPose2.getOrientation());

    return distanceMeasure;
}

}  // namespace crf::math::distancemeasures
