/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "DistanceMeasures/Rotation.hpp"
#include "Types/TaskTypes/TaskTypes.hpp"

using crf::utility::types::TaskPose;

namespace crf::math::distancemeasures {

/**
 * @ingroup group_distance_measures_task_pose
 * @brief Distance measure calculated as coordinate-wise substraction on the linear part
 * and by using quaternion representation in the angular part.
 */
Eigen::Vector<double, 6> byQuaternion(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2);

/**
 * @ingroup group_distance_measures_task_pose
 * @brief Distance measure calculated as coordinate-wise substraction on the linear part
 * and by using matrix representation in the angular part.
 */
Eigen::Vector<double, 6> byRotationMatrix(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2);

/**
 * @ingroup group_distance_measures_task_pose
 * @brief Distance measure calculated as coordinate-wise substraction.
 */
Eigen::Vector<double, 6> byCardanXYZ(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2);

/**
 * @ingroup group_distance_measures_task_pose
 * @brief Distance measure calculated as coordinate-wise substraction.
 */
Eigen::Vector<double, 6> byEulerZXZ(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2);

}  // namespace crf::math::distancemeasures
