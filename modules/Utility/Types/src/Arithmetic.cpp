/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Types/Arithmetic.hpp"

namespace crf::utility::types {

TaskPose multiply(const TaskPose& taskPose1, const TaskPose& taskPose2) {
    TaskPose product;
    product.setPosition(
        taskPose1.getPosition() + taskPose1.getQuaternion() * taskPose2.getPosition());
    product.setOrientation(multiply(taskPose1.getOrientation(), taskPose2.getOrientation()));
    return product;
}

TaskPose invert(const TaskPose& taskPose) {
    TaskPose inverse;
    inverse.setPosition(-(taskPose.getQuaternion().inverse() * taskPose.getPosition()));
    inverse.setOrientation(invert(taskPose.getOrientation()));
    return inverse;
}

}  // namespace crf::utility::types
