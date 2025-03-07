/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

namespace crf::actuators::robot {

/**
 * @brief Enum to select the position control to be used
 *
 */
enum class PositionMode {
    ProfilePositionMode = 1,
    CyclicSyncPositionMode = 2,
    InterpolatedPositionMode = 3
};

/**
 * @brief Enum to select the velocity control to be used
 *
 */
enum class VelocityMode {
    ProfileVelocityMode = 1,
    CyclicSyncVelocityMode = 2,
    VelocityMode = 3,
};

/**
 * @brief Enum to select the torque control to be used
 *
 */
enum class TorqueMode {
    ProfileTorqueMode = 1,
    CyclicSyncTorqueMode = 2
};

}  // namespace crf::actuators::robot
