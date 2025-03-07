/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include "Types/JointTypes/JointSignals.hpp"
#include "Types/TaskTypes/TaskSignals.hpp"

namespace crf::utility::types {
/**
 * @ingroup group_types_signals
 * @brief Struct for aggregating JointSignals and TaskSignals.
*/
struct Signals {
    JointSignals joints;
    TaskSignals task;
};

}  // namespace crf::utility::types
