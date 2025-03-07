/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "Types/JointTypes/JointTypes.hpp"
#include "Types/TaskTypes/TaskTypes.hpp"

using crf::math::rotation::angularVelocityFromRotation;
using crf::math::rotation::rotationFromAngularVelocity;

namespace crf::utility::types {

/**
 * @ingroup group_types_arithmetic
 * @brief Multiplies parameters 'taskPose1' and 'taskPose2' in order taskPose1*taskPose2.
 * It treats task poses as elements of SE(3) group, i.e. for taskPose1 = (position1, orientation1),
 * taskPose2 = (position2, taskPose2), we have
 * taskPose1*taskPose2 = (position1 + orientation1*position2, orientation1*orientation2).
 */
TaskPose multiply(const TaskPose& taskPose1, const TaskPose& taskPose2);

/**
 * @ingroup group_types_arithmetic
 * @brief Inverts parameter 'taskPose' and returns the result.
 * It treats a task pose as an element of SE(3) group, i.e. for taskPose = (position, orientation),
 * taskPose^-1 = (-orientation^-1*position, orientation^-1).
 */
TaskPose invert(const TaskPose& taskPose);

}  // namespace crf::utility::types
