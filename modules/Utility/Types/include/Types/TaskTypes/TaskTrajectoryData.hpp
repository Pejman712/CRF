/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include "Types/TaskTypes/TaskPose.hpp"
#include "Types/TaskTypes/TaskVelocity.hpp"
#include "Types/TaskTypes/TaskAcceleration.hpp"

namespace crf::utility::types {

/**
 * @ingroup group_task_types
 * @brief Legacy struct to aggregate task data into one structure.
 */
struct TaskTrajectoryData {
    TaskPose pose;
    TaskVelocity velocity;
    TaskAcceleration acceleration;
};

}  // namespace crf::utility::types
