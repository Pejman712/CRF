/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include "crf/expected.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "Types/TaskTypes/TaskVelocity.hpp"
#include "Types/TaskTypes/TaskAcceleration.hpp"
#include "Types/TaskTypes/TaskForceTorque.hpp"

namespace crf::utility::types {

/**
 * @ingroup group_task_types
 * @brief Struct for aggregating crf::expected<> signals of task data into one structure.
 */
struct TaskSignals {
    crf::expected<TaskPose> pose;
    crf::expected<TaskVelocity> velocity;
    crf::expected<TaskAcceleration> acceleration;
    crf::expected<TaskForceTorque> forceTorque;
};

}  // namespace crf::utility::types
