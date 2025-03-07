/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include "Types/TaskTypes/Vector6d.hpp"

namespace crf::utility::types {

/**
 * @ingroup group_task_types
 * @brief Class for representing and strongly typing task force and torque as
 * a six dimensional vector, with coordinates
 * [fx, fy, fz, tx, ty, tz].
 */
class TaskForceTorque : public Vector6d {
    using Vector6d::Vector6d;

 public:
    using Vector6d::operator=;
};

}  // namespace crf::utility::types
