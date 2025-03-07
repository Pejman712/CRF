/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include "crf/expected.hpp"
#include "Types/JointTypes/JointPositions.hpp"
#include "Types/JointTypes/JointVelocities.hpp"
#include "Types/JointTypes/JointAccelerations.hpp"
#include "Types/JointTypes/JointForceTorques.hpp"

namespace crf::utility::types {

/**
 * @ingroup group_joint_types
 * @brief Struct for aggregating crf::expected<> signals of joint data into one structure.
 */
struct JointSignals {
    crf::expected<JointPositions> positions;
    crf::expected<JointVelocities> velocities;
    crf::expected<JointAccelerations> accelerations;
    crf::expected<JointForceTorques> forceTorques;
};

}  // namespace crf::utility::types
